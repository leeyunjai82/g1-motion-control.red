"""
Unitree G1 IK Motion Editor - Backend Server
Version: 6.3 (IK + Rotation + 단일 손 동글 + lifespan + 시작 시 허리 0 리셋)

구조:
- 팔 제어: IK (XYZ 좌표 + RPY 회전)
- 걷기: LocoClientWrapper
- 손: HandController (단일 동글)
"""

import uvicorn
from fastapi import FastAPI
from fastapi.responses import HTMLResponse, FileResponse
from pydantic import BaseModel
from fastapi.middleware.cors import CORSMiddleware
from contextlib import asynccontextmanager
import os, time, asyncio
from typing import List, Optional
import numpy as np
import pinocchio as pin

USE_HAND_CONTROL = True

from unitree_sdk2py.core.channel import ChannelFactoryInitialize

from ctrl.arm_controller_wrapper import ArmControllerWrapper, LocoClientWrapper
print("✅ arm_controller_wrapper 로드 성공")

# 손 제어
hand_controller = None
available_hand_motions = []

if USE_HAND_CONTROL:
    try:
        from ctrl.mandro3 import HandController, motions
        available_hand_motions = list(motions.keys())
        print(f"✅ 손 제어 라이브러리 로드 성공. 모션: {len(available_hand_motions)}개")
    except ImportError as e:
        print(f"⚠️ 손 제어 라이브러리 없음: {e}")
        USE_HAND_CONTROL = False


# ==========================================
# 헬퍼 함수
# ==========================================

def rpy_to_quaternion(roll_deg, pitch_deg, yaw_deg):
    roll  = np.radians(roll_deg)
    pitch = np.radians(pitch_deg)
    yaw   = np.radians(yaw_deg)

    cr, sr = np.cos(roll/2),  np.sin(roll/2)
    cp, sp = np.cos(pitch/2), np.sin(pitch/2)
    cy, sy = np.cos(yaw/2),   np.sin(yaw/2)

    w = cr*cp*cy + sr*sp*sy
    x = sr*cp*cy - cr*sp*sy
    y = cr*sp*cy + sr*cp*sy
    z = cr*cp*sy - sr*sp*cy

    return pin.Quaternion(w, x, y, z).normalized()


def move_hands_with_rotation(left_xyz, right_xyz, left_rpy, right_rpy, duration, frequency=100):
    left_rot  = rpy_to_quaternion(*left_rpy)  if left_rpy  and any(v != 0 for v in left_rpy)  else None
    right_rot = rpy_to_quaternion(*right_rpy) if right_rpy and any(v != 0 for v in right_rpy) else None
    arm_wrapper.move_hands(left_xyz, right_xyz, left_rot, right_rot, duration, frequency)


# ==========================================
# Pydantic 모델
# ==========================================

class IKMoveCommand(BaseModel):
    left_xyz:   List[float]
    right_xyz:  List[float]
    left_rpy:   Optional[List[float]] = None
    right_rpy:  Optional[List[float]] = None
    duration:   float = 1.0

class LocoCommand(BaseModel):
    direction: str

class HandCommand(BaseModel):
    hand:    str
    motion:  str
    release: Optional[bool] = False

class LocomotionData(BaseModel):
    direction: str

class HandMotionData(BaseModel):
    hand:   str
    motion: str

class MotionFrame(BaseModel):
    duration:   float
    left_xyz:   Optional[List[float]]   = None
    right_xyz:  Optional[List[float]]   = None
    left_rpy:   Optional[List[float]]   = None
    right_rpy:  Optional[List[float]]   = None
    locomotion: Optional[LocomotionData]  = None
    hand_motion: Optional[HandMotionData] = None


# ==========================================
# 전역 상태
# ==========================================
arm_wrapper:  Optional[ArmControllerWrapper] = None
loco_wrapper: Optional[LocoClientWrapper]    = None
STOP_REQUESTED = False

current_ik_position = {"left": [0.1, 0.2, 0.2], "right": [0.1, -0.2, 0.2]}
current_rpy         = {"left": [0.0, 0.0, 0.0], "right": [0.0, 0.0, 0.0]}


# ==========================================
# Lifespan
# ==========================================
@asynccontextmanager
async def lifespan(app: FastAPI):
    global hand_controller, arm_wrapper, loco_wrapper

    print("--- IK Motion Editor 서버 시작 (v6.3) ---")
    print("=" * 50)
    print("  - 팔 제어: IK (XYZ + RPY)")
    print("  - 걷기: LocoClientWrapper")
    print("  - 손: HandController (단일 동글)")
    print("=" * 50)

    ChannelFactoryInitialize(0)

    try:
        loco_wrapper = LocoClientWrapper()
        print("✅ LocoClientWrapper 초기화 성공")
    except Exception as e:
        print(f"⚠️ LocoClientWrapper 실패: {e}")

    try:
        arm_wrapper = ArmControllerWrapper(
            motion_mode=True,
            simulation_mode=False,
            visualization=False,
            use_motor_control=True
        )
        arm_wrapper.start()
        print("✅ ArmControllerWrapper 초기화 성공")

        # 시작 시 waist 0으로 명시적 리셋 (이전 실행 잔류값 정리)
        print("[로봇] waist 0으로 리셋")
        arm_wrapper.move_waist_smooth(yaw=0.0, roll=0.0, pitch=0.0, duration=2.0)
        time.sleep(2.0)
    except Exception as e:
        print(f"⚠️ ArmControllerWrapper 실패: {e}")

    if USE_HAND_CONTROL:
        try:
            hand_controller = HandController('/dev/ttyACM0')
            print("✅ 손 컨트롤러 연결 성공 (/dev/ttyACM0)")
        except Exception as e:
            print(f"⚠️ 손 컨트롤러 연결 실패: {e}")

    await asyncio.sleep(3)
    await emergency_stop()
    print("[시스템] 준비 완료")

    yield

    if arm_wrapper:
        arm_wrapper.go_home()
    print("--- 서버 종료 ---")


# ==========================================
# FastAPI 앱
# ==========================================
app = FastAPI(
    title="G1 IK Motion Editor",
    version="6.3.0",
    lifespan=lifespan,
)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


# ==========================================
# 손 제어 함수
# ==========================================
def execute_hand_motion_sync(hand: str, motion: str, release: bool = False):
    if not USE_HAND_CONTROL or not hand_controller:
        return
    try:
        if release:
            hand_controller.send_release(selector=hand)
        else:
            hand_controller.send_motion(motion, selector=hand)
    except Exception as e:
        print(f"[Hand] 에러: {e}")


async def execute_hand_motion(hand: str, motion: str, release: bool = False):
    loop = asyncio.get_running_loop()
    await loop.run_in_executor(None, execute_hand_motion_sync, hand, motion, release)


async def emergency_stop():
    global current_ik_position, current_rpy
    print("!!! 긴급 정지 !!!")

    if loco_wrapper:
        loco_wrapper.stop()

    if arm_wrapper:
        home_left  = [0.1,  0.2, 0.2]
        home_right = [0.1, -0.2, 0.2]
        home_rpy   = [0.0,  0.0, 0.0]

        loop = asyncio.get_running_loop()
        await loop.run_in_executor(
            None, move_hands_with_rotation,
            home_left, home_right, home_rpy, home_rpy, 1.0, 100
        )
        current_ik_position = {"left": home_left, "right": home_right}
        current_rpy         = {"left": home_rpy.copy(), "right": home_rpy.copy()}

    if USE_HAND_CONTROL and hand_controller:
        try:
            hand_controller.send_motion('unfold_a', selector='both')
        except Exception as e:
            print(f"[Hand 긴급정지] 에러: {e}")

    print("!!! 긴급 정지 완료 !!!")


# ==========================================
# 손 API
# ==========================================

@app.get("/hand_motions")
async def get_hand_motions():
    connected = hand_controller is not None
    return {
        "enabled":        USE_HAND_CONTROL,
        "left_connected":  connected,
        "right_connected": connected,
        "single_dongle":   True,
        "motions":         available_hand_motions
    }


@app.post("/set_hand")
async def set_hand(command: HandCommand):
    if not USE_HAND_CONTROL:
        return {"status": "disabled"}
    if not hand_controller:
        return {"status": "error", "message": "Hand controller not connected"}
    if command.motion not in available_hand_motions:
        return {"status": "error", "message": f"Unknown motion: {command.motion}"}
    await execute_hand_motion(command.hand, command.motion, command.release)
    return {"status": "success"}


# ==========================================
# IK API
# ==========================================

@app.get("/ik_position")
async def get_ik_position():
    return {
        "status":    "success",
        "left_xyz":  current_ik_position["left"],
        "right_xyz": current_ik_position["right"],
        "left_rpy":  current_rpy["left"],
        "right_rpy": current_rpy["right"],
    }


@app.post("/set_ik")
async def set_ik(command: IKMoveCommand):
    global current_ik_position, current_rpy

    if not arm_wrapper:
        return {"status": "error", "message": "ArmControllerWrapper not initialized"}
    if len(command.left_xyz) != 3 or len(command.right_xyz) != 3:
        return {"status": "error", "message": "XYZ must have 3 elements"}

    left_rpy  = command.left_rpy  or [0.0, 0.0, 0.0]
    right_rpy = command.right_rpy or [0.0, 0.0, 0.0]

    print(f"[IK] L={command.left_xyz} R={command.right_xyz}")
    print(f"[IK] L_rpy={left_rpy} R_rpy={right_rpy} dur={command.duration}")

    try:
        loop = asyncio.get_running_loop()
        await loop.run_in_executor(
            None, move_hands_with_rotation,
            command.left_xyz, command.right_xyz,
            left_rpy, right_rpy, command.duration, 100
        )
        current_ik_position = {"left": command.left_xyz, "right": command.right_xyz}
        current_rpy         = {"left": left_rpy, "right": right_rpy}
        return {"status": "success"}
    except Exception as e:
        print(f"[IK Error] {e}")
        return {"status": "error", "message": str(e)}


# ==========================================
# Locomotion API
# ==========================================

last_loco_command = {"direction": None, "timestamp": 0}
loco_lock = asyncio.Lock()

@app.post("/set_loco_motion")
async def set_loco_motion(command: LocoCommand):
    global last_loco_command

    if not loco_wrapper:
        return {"status": "error", "message": "LocoClientWrapper not initialized"}

    async with loco_lock:
        now = time.time()
        if (command.direction == last_loco_command["direction"] and
                now - last_loco_command["timestamp"] < 0.1):
            return {"status": "skipped"}
        last_loco_command = {"direction": command.direction, "timestamp": now}

    try:
        loop = asyncio.get_running_loop()
        direction_map = {
            "forward":    loco_wrapper.forward,
            "backward":   loco_wrapper.backward,
            "left":       loco_wrapper.left,
            "right":      loco_wrapper.right,
            "turn_left":  loco_wrapper.turn_left,
            "turn_right": loco_wrapper.turn_right,
            "stop":       loco_wrapper.stop,
        }
        if command.direction in direction_map:
            await loop.run_in_executor(None, direction_map[command.direction])
        return {"status": "success"}
    except Exception as e:
        return {"status": "error", "message": str(e)}


# ==========================================
# 모션 시퀀스 API
# ==========================================

@app.post("/set_motion")
async def set_motion(motion_sequence: List[MotionFrame]):
    global STOP_REQUESTED, current_ik_position, current_rpy
    STOP_REQUESTED = False

    print(f"[모션] 시작: {len(motion_sequence)}개 프레임")
    loop = asyncio.get_running_loop()

    for i, frame in enumerate(motion_sequence):
        if STOP_REQUESTED:
            print(f"[모션] 중단: 프레임 {i+1}")
            break

        print(f"[모션] 프레임 {i+1}/{len(motion_sequence)} ({frame.duration}초)")

        # 손 모션 - Future로 처리
        hand_future = None
        if frame.hand_motion and USE_HAND_CONTROL and hand_controller:
            hand_future = loop.run_in_executor(
                None, execute_hand_motion_sync,
                frame.hand_motion.hand, frame.hand_motion.motion, False
            )

        # IK 이동
        if frame.left_xyz and frame.right_xyz and arm_wrapper:
            left_rpy  = frame.left_rpy  or [0.0, 0.0, 0.0]
            right_rpy = frame.right_rpy or [0.0, 0.0, 0.0]

            await loop.run_in_executor(
                None, move_hands_with_rotation,
                frame.left_xyz, frame.right_xyz,
                left_rpy, right_rpy, frame.duration, 100
            )
            current_ik_position = {"left": frame.left_xyz, "right": frame.right_xyz}
            current_rpy         = {"left": left_rpy, "right": right_rpy}

        # 걷기
        if frame.locomotion and loco_wrapper:
            direction_map = {
                "forward":    loco_wrapper.forward,
                "backward":   loco_wrapper.backward,
                "left":       loco_wrapper.left,
                "right":      loco_wrapper.right,
                "turn_left":  loco_wrapper.turn_left,
                "turn_right": loco_wrapper.turn_right,
            }
            method = direction_map.get(frame.locomotion.direction)
            if method:
                start = time.time()
                while time.time() - start < frame.duration:
                    if STOP_REQUESTED: break
                    method()
                    await asyncio.sleep(0.02)
                if not STOP_REQUESTED:
                    loco_wrapper.stop()

        elif not (frame.left_xyz and frame.right_xyz):
            await asyncio.sleep(frame.duration)

        if hand_future:
            await hand_future

    if STOP_REQUESTED:
        await emergency_stop()
        STOP_REQUESTED = False
    else:
        print("[모션] 완료")
        if loco_wrapper:
            loco_wrapper.stop()

    return {"status": "success"}


@app.post("/stop_motion")
async def stop_motion():
    global STOP_REQUESTED
    print("[정지] 요청")
    STOP_REQUESTED = True
    await emergency_stop()
    return {"status": "success"}


@app.get("/", response_class=HTMLResponse)
async def read_root():
    html_file_path = os.path.join(os.path.dirname(__file__), "simulator_ik.html")
    if os.path.exists(html_file_path):
        return FileResponse(html_file_path)
    return HTMLResponse("<h1>Error: simulator_ik.html not found</h1>")


if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)
