"""
Unitree G1 Motion Editor - Backend Server
Version: 5.3 (단일 손 동글 통합 + 시작 시 허리 0 리셋)

구조:
- 팔 제어 (15~28): ArmControllerWrapper 사용
  - move_joint_smooth(): 단일 관절 보간 이동
  - move_joints_smooth(): 전체 관절 보간 이동
- 허리 제어 (전역 0~2): ArmControllerWrapper.move_waist_smooth() 사용
- 걷기: LocoClientWrapper 사용
- 손 제어: HandController (단일 동글, left/right/both selector)
"""

import uvicorn
from fastapi import FastAPI
from fastapi.responses import HTMLResponse, FileResponse
from pydantic import BaseModel
from fastapi.middleware.cors import CORSMiddleware
import os, time, asyncio, threading
from typing import List, Optional
import numpy as np

# --- 전역 설정 ---
USE_HAND_CONTROL = True

from unitree_sdk2py.core.channel import ChannelFactoryInitialize

# --- Wrapper 임포트 ---
from ctrl.arm_controller_wrapper import (
    ArmControllerWrapper,
    LocoClientWrapper,
    JOINT_INFO,
    JOINT_NAMES,
    GLOBAL_TO_INTERNAL
)
print("✅ arm_controller_wrapper 로드 성공")

# --- 손 제어 라이브러리 임포트 ---
hand_controller = None
available_hand_motions = []

if USE_HAND_CONTROL:
    try:
        from ctrl.mandro3 import HandController, motions
        available_hand_motions = list(motions.keys())
        print(f"✅ 손 제어 라이브러리 로드 성공. 사용 가능한 모션: {len(available_hand_motions)}개")
    except ImportError as e:
        print(f"⚠️ 손 제어 라이브러리를 찾을 수 없습니다: {e}")
        USE_HAND_CONTROL = False


# --- Pydantic 모델 ---
class MotorCommand(BaseModel):
    motor_index: int
    target_degree: float
    duration: float = 1.0

class AllMotorsCommand(BaseModel):
    target_degrees: List[float]  # 14개 (팔만)
    duration: float = 1.0

class WaistCommand(BaseModel):
    yaw: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    duration: float = 1.0

class LocoCommand(BaseModel):
    direction: str

class MotorTarget(BaseModel):
    motor_index: int
    target_degree: float

class PoseData(BaseModel):
    targets: List[MotorTarget]

class LocomotionData(BaseModel):
    direction: str

class HandMotionData(BaseModel):
    hand: str   # 'left', 'right', 'both'
    motion: str

class MotionFrame(BaseModel):
    duration: float
    pose: Optional[PoseData] = None
    locomotion: Optional[LocomotionData] = None
    hand_motion: Optional[HandMotionData] = None

class HandCommand(BaseModel):
    hand: str   # 'left', 'right', 'both'
    motion: str
    release: Optional[bool] = False


# --- FastAPI 앱 ---
app = FastAPI()

arm_wrapper: Optional[ArmControllerWrapper] = None
loco_wrapper: Optional[LocoClientWrapper] = None
STOP_REQUESTED = False

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


# ==================== 손 제어 함수 ====================

def execute_hand_motion_sync(hand: str, motion: str, release: bool = False):
    """손 모션 실행 (동기) - 단일 동글, selector로 좌우/양손 구분"""
    if not USE_HAND_CONTROL or not hand_controller:
        return
    try:
        selector = hand  # 'left', 'right', 'both' 그대로 전달
        if release:
            hand_controller.send_release(selector=selector)
        else:
            hand_controller.send_motion(motion, selector=selector)
    except Exception as e:
        print(f"[Hand] 에러: {e}")


async def execute_hand_motion(hand: str, motion: str, release: bool = False):
    """손 모션 실행 (비동기)"""
    loop = asyncio.get_running_loop()
    await loop.run_in_executor(None, execute_hand_motion_sync, hand, motion, release)


async def emergency_stop():
    """긴급 정지 - 팔 14개 + 허리 3축 동시 홈으로"""
    print("!!! 긴급 정지 실행 !!!")

    if loco_wrapper:
        loco_wrapper.stop()

    if arm_wrapper:
        loop = asyncio.get_running_loop()
        tasks = [
            loop.run_in_executor(None, arm_wrapper.move_joints_smooth, [0] * 14, 1.0),
            loop.run_in_executor(None, arm_wrapper.move_waist_smooth, 0.0, 0.0, 0.0, 1.0),
        ]
        await asyncio.gather(*tasks)

    if USE_HAND_CONTROL and hand_controller:
        try:
            hand_controller.send_motion('unfold_a', selector='both')
        except Exception as e:
            print(f"[Hand 긴급정지] 에러: {e}")

    print("!!! 긴급 정지 완료 !!!")


@app.on_event("startup")
async def startup_event():
    global hand_controller, arm_wrapper, loco_wrapper
    print("--- FastAPI 서버 시작 ---")
    print("=" * 50)
    print("구조:")
    print("  - 허리 제어: ArmControllerWrapper.move_waist_smooth() (전역 0~2)")
    print("  - 팔 제어: ArmControllerWrapper (전역 15~28)")
    print("    - move_joint_smooth(): 단일 관절")
    print("    - move_joints_smooth(): 전체 관절")
    print("  - 걷기: LocoClientWrapper")
    print("  - 손: HandController (단일 동글)")
    print("=" * 50)

    ChannelFactoryInitialize(0)

    try:
        loco_wrapper = LocoClientWrapper()
        print("✅ LocoClientWrapper 초기화 성공")
    except Exception as e:
        print(f"⚠️ LocoClientWrapper 초기화 실패: {e}")
        loco_wrapper = None

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
        print(f"⚠️ ArmControllerWrapper 초기화 실패: {e}")
        arm_wrapper = None

    # 손 제어 초기화 (단일 동글)
    if USE_HAND_CONTROL:
        try:
            hand_controller = HandController('/dev/ttyACM0')
            print("✅ 손 컨트롤러 연결 성공 (/dev/ttyACM0)")
        except Exception as e:
            print(f"⚠️ 손 컨트롤러 연결 실패: {e}")
            hand_controller = None

    await asyncio.sleep(3)
    await emergency_stop()
    print("[시스템] 준비 완료")


@app.on_event("shutdown")
async def shutdown_event():
    print("--- 서버 종료 ---")
    if arm_wrapper:
        arm_wrapper.go_home()


# ==================== 손 제어 API ====================

@app.get("/hand_motions")
async def get_hand_motions():
    """손 모션 목록 및 연결 상태"""
    connected = hand_controller is not None
    return {
        "enabled": USE_HAND_CONTROL,
        "left_connected": connected,
        "right_connected": connected,
        "single_dongle": True,
        "motions": available_hand_motions
    }


@app.post("/set_hand")
async def set_hand(command: HandCommand):
    """손 모션 실행 (left/right/both)"""
    if not USE_HAND_CONTROL:
        return {"status": "disabled", "message": "Hand control is disabled"}

    if not hand_controller:
        return {"status": "error", "message": "Hand controller not connected"}

    if command.motion not in available_hand_motions:
        return {"status": "error", "message": f"Unknown motion: {command.motion}"}

    await execute_hand_motion(command.hand, command.motion, command.release)
    return {"status": "success"}


# ==================== 모터 제어 API ====================

@app.post("/set_motor")
async def set_motor(command: MotorCommand):
    """단일 모터 제어 (허리 0~2, 팔 15~28)"""
    if not arm_wrapper:
        return {"status": "error", "message": "ArmControllerWrapper not initialized"}

    print(f"[set_motor] index={command.motor_index}, deg={command.target_degree}, dur={command.duration}")

    try:
        loop = asyncio.get_running_loop()
        await loop.run_in_executor(
            None,
            arm_wrapper.move_joint_smooth,
            command.motor_index,
            command.target_degree,
            command.duration
        )
        return {"status": "success"}
    except ValueError as e:
        return {"status": "error", "message": str(e)}
    except Exception as e:
        print(f"[set_motor Error] {e}")
        return {"status": "error", "message": str(e)}


@app.post("/set_waist")
async def set_waist(command: WaistCommand):
    """허리 3축 동시 제어"""
    if not arm_wrapper:
        return {"status": "error", "message": "ArmControllerWrapper not initialized"}

    print(f"[set_waist] yaw={command.yaw}, roll={command.roll}, pitch={command.pitch}, dur={command.duration}")

    try:
        loop = asyncio.get_running_loop()
        await loop.run_in_executor(
            None,
            arm_wrapper.move_waist_smooth,
            command.yaw, command.roll, command.pitch, command.duration
        )
        return {"status": "success"}
    except Exception as e:
        print(f"[set_waist Error] {e}")
        return {"status": "error", "message": str(e)}


@app.post("/set_all_motors")
async def set_all_motors(command: AllMotorsCommand):
    """전체 팔 모터 제어 (14개)"""
    if not arm_wrapper:
        return {"status": "error", "message": "ArmControllerWrapper not initialized"}

    if len(command.target_degrees) != 14:
        return {"status": "error", "message": "target_degrees must have 14 elements"}

    try:
        loop = asyncio.get_running_loop()
        await loop.run_in_executor(
            None,
            arm_wrapper.move_joints_smooth,
            command.target_degrees,
            command.duration
        )
        return {"status": "success"}
    except Exception as e:
        print(f"[set_all_motors Error] {e}")
        return {"status": "error", "message": str(e)}


# ==================== 걷기 API ====================

last_loco_command = {"direction": None, "timestamp": 0}
loco_lock = asyncio.Lock()

@app.post("/set_loco_motion")
async def set_loco_motion(command: LocoCommand):
    """걷기 명령"""
    global last_loco_command

    if not loco_wrapper:
        return {"status": "error", "message": "LocoClientWrapper not initialized"}

    async with loco_lock:
        now = time.time()
        if (command.direction == last_loco_command["direction"] and
            now - last_loco_command["timestamp"] < 0.1):
            return {"status": "skipped", "reason": "duplicate"}
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
        else:
            return {"status": "error", "message": f"Unknown direction: {command.direction}"}
        return {"status": "success"}
    except Exception as e:
        print(f"[Loco Error] {e}")
        return {"status": "error", "message": str(e)}


# ==================== 관절 정보 API ====================

@app.get("/joint_info")
async def get_joint_info():
    return {
        "status": "success",
        "joint_info": [
            {"internal": info[0], "global": info[1], "name": info[2]}
            for info in JOINT_INFO
        ],
        "joint_names": JOINT_NAMES
    }


# ==================== 모션 시퀀스 API ====================

@app.post("/set_motion")
async def set_motion(motion_sequence: List[MotionFrame]):
    """모션 시퀀스 실행"""
    global STOP_REQUESTED
    STOP_REQUESTED = False

    print(f"[모션] 시작: {len(motion_sequence)}개 프레임")
    loop = asyncio.get_running_loop()

    for i, frame in enumerate(motion_sequence):
        if STOP_REQUESTED:
            print(f"[모션] 중단: 프레임 {i+1}")
            break

        print(f"[모션] 프레임 {i+1}/{len(motion_sequence)} ({frame.duration}초)")

        # 손 모션 (비동기 시작)
        hand_task = None
        if frame.hand_motion and USE_HAND_CONTROL and hand_controller:
            hand_task = asyncio.create_task(
                execute_hand_motion(
                    frame.hand_motion.hand,
                    frame.hand_motion.motion
                )
            )

        # 자세(포즈) 명령
        if frame.pose and frame.pose.targets and arm_wrapper:
            with arm_wrapper.arm_ctrl.ctrl_lock:
                current_arm_targets = np.degrees(arm_wrapper.arm_ctrl.q_target.copy())

            try:
                with arm_wrapper.arm_ctrl.ctrl_lock:
                    curr_waist = np.degrees(
                        getattr(arm_wrapper.arm_ctrl, 'waist_q_target', np.zeros(3)).copy()
                    )
            except:
                curr_waist = np.zeros(3)

            waist_targets = curr_waist.copy()
            has_waist = False

            for target in frame.pose.targets:
                if 0 <= target.motor_index <= 2:
                    waist_targets[target.motor_index] = target.target_degree
                    has_waist = True
                elif 15 <= target.motor_index <= 28:
                    internal_idx = GLOBAL_TO_INTERNAL[target.motor_index]
                    current_arm_targets[internal_idx] = target.target_degree
                elif 3 <= target.motor_index <= 16:
                    current_arm_targets[target.motor_index] = target.target_degree

            move_tasks = [
                loop.run_in_executor(
                    None, arm_wrapper.move_joints_smooth,
                    current_arm_targets.tolist(), frame.duration
                )
            ]
            if has_waist:
                move_tasks.append(
                    loop.run_in_executor(
                        None, arm_wrapper.move_waist_smooth,
                        float(waist_targets[0]), float(waist_targets[1]),
                        float(waist_targets[2]), frame.duration
                    )
                )
            await asyncio.gather(*move_tasks)

        # 걷기 명령
        if frame.locomotion and loco_wrapper:
            direction = frame.locomotion.direction
            direction_methods = {
                "forward":    loco_wrapper.forward,
                "backward":   loco_wrapper.backward,
                "left":       loco_wrapper.left,
                "right":      loco_wrapper.right,
                "turn_left":  loco_wrapper.turn_left,
                "turn_right": loco_wrapper.turn_right,
            }
            if direction in direction_methods:
                start_time = time.time()
                while time.time() - start_time < frame.duration:
                    if STOP_REQUESTED:
                        break
                    direction_methods[direction]()
                    await asyncio.sleep(0.02)
                if not STOP_REQUESTED:
                    loco_wrapper.stop()
        elif not frame.pose:
            await asyncio.sleep(frame.duration)

        if hand_task:
            await hand_task

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
    """긴급 정지"""
    global STOP_REQUESTED
    print("[정지] 요청 수신")
    STOP_REQUESTED = True
    await emergency_stop()
    return {"status": "success"}


@app.get("/", response_class=HTMLResponse)
async def read_root():
    html_file_path = os.path.join(os.path.dirname(__file__), "simulator.html")
    if os.path.exists(html_file_path):
        return FileResponse(html_file_path)
    return HTMLResponse("<h1>Error: simulator.html not found</h1>")


if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)
