"""
simulator.py — G1 Motion Editor 통합본 (관절 + IK)
Version: 7.0

simulator.py(관절 편집기 v5.3) + simulator_ik.py(IK 편집기 v6.3) 통합.

구조 변경 (arm_server 분리 반영):
  - 팔/허리 지령은 arm_server(50022) 경유 (ctrl/arm_http.ArmHttpClient).
    → robot_server 와 동시에 떠도 arm_sdk 이중 송신이 없다.
    → 단독 사용 시에도 arm_server 를 먼저 띄워야 한다:
         python arm_server.py   →   python simulator.py
  - 걷기: LocoClientWrapper 직접 (다중 클라이언트 성립 실기 확인됨)
  - 손: HandController (단일 동글) 직접

UI:
  /   통합 simulator.html (좌측 패널 Joint/IK 모드 토글, 타임라인 공용)

엔드포인트 = 두 편집기의 합집합:
  공통: /hand_motions /set_hand /set_loco_motion /set_motion /stop_motion
  관절: /set_motor /set_waist /set_all_motors /joint_info
  IK  : /set_ik /ik_position
  /set_motion 은 프레임에 pose(관절)와 left_xyz/right_xyz(IK)가 섞여 있어도
  프레임별로 자동 판별해 실행한다.

안전 변경:
  - /stop_motion(긴급 정지)은 이동하지 않는다 — loco 정지 + 보간 중단 +
    현재 자세 동결(freeze) + 손 펴기. (구버전은 관절 0도/홈으로 '이동'했는데,
    0도는 앞으로 나란히라 정지 중 팔이 크게 움직였음)
  - 기동 시 자동 홈 이동/허리 리셋 없음 (arm_server 가 자세를 이미 유지 중)
  - 자세 복귀가 필요하면 POST /go_home (IK 홈 자세로 이동)
"""

import os
import time
import asyncio
from typing import List, Optional

import uvicorn
import numpy as np
from fastapi import FastAPI
from fastapi.responses import HTMLResponse, FileResponse
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from contextlib import asynccontextmanager

import sys
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_dir)

USE_HAND_CONTROL = True

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from ctrl.arm_controller_wrapper import (LocoClientWrapper, JOINT_INFO,
                                         JOINT_NAMES, GLOBAL_TO_INTERNAL)
from ctrl.arm_http import ArmHttpClient

hand_controller = None
available_hand_motions = []
if USE_HAND_CONTROL:
    try:
        from ctrl.mandro3 import HandController, motions
        available_hand_motions = list(motions.keys())
        print(f"✅ 손 제어 라이브러리 로드. 모션 {len(available_hand_motions)}개")
    except ImportError as e:
        print(f"⚠️ 손 제어 라이브러리 없음: {e}")
        USE_HAND_CONTROL = False


# ==========================================
# Pydantic (두 편집기 합집합)
# ==========================================
class MotorCommand(BaseModel):
    motor_index: int
    target_degree: float
    duration: float = 1.0

class AllMotorsCommand(BaseModel):
    target_degrees: List[float]     # 14개 (팔만)
    duration: float = 1.0

class WaistCommand(BaseModel):
    yaw: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    duration: float = 1.0

class LocoCommand(BaseModel):
    direction: str

class HandCommand(BaseModel):
    hand: str                        # left | right | both
    motion: str
    release: bool = False

class IKMoveCommand(BaseModel):
    left_xyz:  List[float]
    right_xyz: List[float]
    left_rpy:  Optional[List[float]] = None
    right_rpy: Optional[List[float]] = None
    duration:  float = 1.0

class MotorTarget(BaseModel):
    motor_index: int
    target_degree: float

class PoseData(BaseModel):
    targets: List[MotorTarget]

class LocomotionData(BaseModel):
    direction: str

class HandMotionData(BaseModel):
    hand: str
    motion: str

class MotionFrame(BaseModel):
    """관절(pose)과 IK(left_xyz/right_xyz) 프레임 공용 — 프레임별 자동 판별."""
    duration: float
    pose: Optional[PoseData] = None
    left_xyz:  Optional[List[float]] = None
    right_xyz: Optional[List[float]] = None
    left_rpy:  Optional[List[float]] = None
    right_rpy: Optional[List[float]] = None
    locomotion: Optional[LocomotionData] = None
    hand_motion: Optional[HandMotionData] = None


# ==========================================
# 전역 상태
# ==========================================
arm:  Optional[ArmHttpClient]     = None
loco: Optional[LocoClientWrapper] = None
STOP_REQUESTED = False

current_ik_position = {"left": [0.1, 0.2, 0.2], "right": [0.1, -0.2, 0.2]}
current_rpy         = {"left": [0.0, 0.0, 0.0], "right": [0.0, 0.0, 0.0]}

IK_HOME_LEFT  = [0.1,  0.2, 0.2]
IK_HOME_RIGHT = [0.1, -0.2, 0.2]


# ==========================================
# 손 제어
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


# ==========================================
# 긴급 정지 — 이동 없이 그 자리 동결
# ==========================================
async def emergency_stop():
    print("!!! 긴급 정지 (동결) !!!")
    if loco:
        loco.stop()
    if arm:
        loop = asyncio.get_running_loop()
        try:
            await loop.run_in_executor(None, arm.stop_motion)
            await loop.run_in_executor(None, arm.freeze)
        except Exception as e:
            print(f"[정지] arm 동결 실패: {e}")
    if USE_HAND_CONTROL and hand_controller:
        try:
            hand_controller.send_motion('unfold_a', selector='both')
        except Exception as e:
            print(f"[Hand 정지] 에러: {e}")
    print("!!! 긴급 정지 완료 !!!")


def _move_ik(left_xyz, right_xyz, left_rpy, right_rpy, duration):
    """arm_server /hands 호출 (rpy는 deg). 완료까지 블로킹."""
    import json as _json
    import urllib.request as _u
    body = {"left_xyz": [float(v) for v in left_xyz],
            "right_xyz": [float(v) for v in right_xyz],
            "duration": float(duration), "frequency": 100}
    if left_rpy and any(v != 0 for v in left_rpy):
        body["left_rpy"] = [float(v) for v in left_rpy]
    if right_rpy and any(v != 0 for v in right_rpy):
        body["right_rpy"] = [float(v) for v in right_rpy]
    req = _u.Request("http://localhost:50022/hands",
                     data=_json.dumps(body).encode(),
                     headers={"Content-Type": "application/json"}, method="POST")
    with _u.urlopen(req, timeout=duration + 30.0) as r:
        return _json.loads(r.read())


# ==========================================
# Lifespan
# ==========================================
@asynccontextmanager
async def lifespan(app: FastAPI):
    global hand_controller, arm, loco
    print("--- G1 Motion Editor 통합본 (v7.0) ---")
    print("  팔/허리: arm_server(50022) 경유 · 걷기: LocoClient · 손: 단일 동글")

    ChannelFactoryInitialize(0)

    try:
        loco = LocoClientWrapper()
        print("✅ Loco 초기화")
    except Exception as e:
        print(f"⚠️ Loco 실패: {e}")

    try:
        arm = ArmHttpClient()          # arm_server 가 먼저 떠 있어야 함
    except Exception as e:
        print(f"⚠️ arm_server 연결 실패: {e}")
        print("   → 먼저 실행: python arm_server.py")
        arm = None

    if USE_HAND_CONTROL:
        try:
            hand_controller = HandController('/dev/ttyACM0')
            print("✅ 손 컨트롤러 연결 (/dev/ttyACM0)")
        except Exception as e:
            print(f"⚠️ 손 컨트롤러 실패: {e}")

    print("[시스템] 준비 완료  http://localhost:8000/")
    yield
    print("--- 서버 종료 (자세 유지 — arm_server 관리) ---")


app = FastAPI(title="G1 Motion Editor (통합)", version="7.0", lifespan=lifespan)
app.add_middleware(CORSMiddleware, allow_origins=["*"], allow_credentials=True,
                   allow_methods=["*"], allow_headers=["*"])


# ==========================================
# 손 API
# ==========================================
@app.get("/hand_motions")
async def get_hand_motions():
    connected = hand_controller is not None
    return {"enabled": USE_HAND_CONTROL, "left_connected": connected,
            "right_connected": connected, "single_dongle": True,
            "motions": available_hand_motions}


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
# 관절 API (구 simulator.py)
# ==========================================
@app.get("/joint_info")
async def get_joint_info():
    return {"status": "success",
            "joint_info": [{"internal": i[0], "global": i[1], "name": i[2]}
                           for i in JOINT_INFO],
            "joint_names": JOINT_NAMES}


@app.post("/set_motor")
async def set_motor(command: MotorCommand):
    """단일 모터 (허리 0~2 / 팔 15~28). 현재 타겟 기준 한 축만 변경."""
    if not arm:
        return {"status": "error", "message": "arm_server 미연결"}
    try:
        loop = asyncio.get_running_loop()
        idx, deg, dur = command.motor_index, command.target_degree, command.duration
        if 0 <= idx <= 2:
            waist = np.degrees(arm.arm_ctrl.waist_q_target).tolist()
            waist[idx] = deg
            await loop.run_in_executor(None, lambda: arm.move_waist_smooth(
                yaw=waist[0], roll=waist[1], pitch=waist[2], duration=dur))
        elif 15 <= idx <= 28:
            targets = np.degrees(arm.arm_ctrl.q_target).tolist()
            targets[GLOBAL_TO_INTERNAL[idx]] = deg
            await loop.run_in_executor(None, arm.move_joints_smooth, targets, dur)
        else:
            return {"status": "error", "message": f"invalid index: {idx}"}
        return {"status": "success"}
    except Exception as e:
        print(f"[set_motor Error] {e}")
        return {"status": "error", "message": str(e)}


@app.post("/set_waist")
async def set_waist(command: WaistCommand):
    if not arm:
        return {"status": "error", "message": "arm_server 미연결"}
    try:
        loop = asyncio.get_running_loop()
        await loop.run_in_executor(None, lambda: arm.move_waist_smooth(
            yaw=command.yaw, roll=command.roll,
            pitch=command.pitch, duration=command.duration))
        return {"status": "success"}
    except Exception as e:
        return {"status": "error", "message": str(e)}


@app.post("/set_all_motors")
async def set_all_motors(command: AllMotorsCommand):
    if not arm:
        return {"status": "error", "message": "arm_server 미연결"}
    if len(command.target_degrees) != 14:
        return {"status": "error", "message": "target_degrees must have 14 elements"}
    try:
        loop = asyncio.get_running_loop()
        await loop.run_in_executor(None, arm.move_joints_smooth,
                                   command.target_degrees, command.duration)
        return {"status": "success"}
    except Exception as e:
        return {"status": "error", "message": str(e)}


# ==========================================
# IK API (구 simulator_ik.py)
# ==========================================
@app.get("/ik_position")
async def get_ik_position():
    return {"status": "success",
            "left_xyz": current_ik_position["left"],
            "right_xyz": current_ik_position["right"],
            "left_rpy": current_rpy["left"],
            "right_rpy": current_rpy["right"]}


@app.post("/set_ik")
async def set_ik(command: IKMoveCommand):
    global current_ik_position, current_rpy
    if not arm:
        return {"status": "error", "message": "arm_server 미연결"}
    if len(command.left_xyz) != 3 or len(command.right_xyz) != 3:
        return {"status": "error", "message": "XYZ must have 3 elements"}
    left_rpy = command.left_rpy or [0.0, 0.0, 0.0]
    right_rpy = command.right_rpy or [0.0, 0.0, 0.0]
    try:
        loop = asyncio.get_running_loop()
        await loop.run_in_executor(None, _move_ik, command.left_xyz,
                                   command.right_xyz, left_rpy, right_rpy,
                                   command.duration)
        current_ik_position = {"left": command.left_xyz, "right": command.right_xyz}
        current_rpy = {"left": left_rpy, "right": right_rpy}
        return {"status": "success"}
    except Exception as e:
        print(f"[IK Error] {e}")
        return {"status": "error", "message": str(e)}


@app.post("/go_home")
async def go_home():
    """IK 홈 자세로 이동 (구 긴급정지의 '홈 복귀'를 명시적 동작으로 분리)."""
    global current_ik_position, current_rpy
    if not arm:
        return {"status": "error", "message": "arm_server 미연결"}
    loop = asyncio.get_running_loop()
    await loop.run_in_executor(None, _move_ik, IK_HOME_LEFT, IK_HOME_RIGHT,
                               [0, 0, 0], [0, 0, 0], 1.5)
    current_ik_position = {"left": IK_HOME_LEFT, "right": IK_HOME_RIGHT}
    current_rpy = {"left": [0.0, 0.0, 0.0], "right": [0.0, 0.0, 0.0]}
    return {"status": "success"}


# ==========================================
# 걷기 API
# ==========================================
last_loco_command = {"direction": None, "timestamp": 0}
loco_lock = asyncio.Lock()


@app.post("/set_loco_motion")
async def set_loco_motion(command: LocoCommand):
    global last_loco_command
    if not loco:
        return {"status": "error", "message": "Loco 미초기화"}
    async with loco_lock:
        now = time.time()
        if (command.direction == last_loco_command["direction"]
                and now - last_loco_command["timestamp"] < 0.1):
            return {"status": "skipped"}
        last_loco_command = {"direction": command.direction, "timestamp": now}
    try:
        dmap = {"forward": loco.forward, "backward": loco.backward,
                "left": loco.left, "right": loco.right,
                "turn_left": loco.turn_left, "turn_right": loco.turn_right,
                "stop": loco.stop}
        method = dmap.get(command.direction)
        if not method:
            return {"status": "error", "message": f"Unknown: {command.direction}"}
        await asyncio.get_running_loop().run_in_executor(None, method)
        return {"status": "success"}
    except Exception as e:
        return {"status": "error", "message": str(e)}


# ==========================================
# 모션 시퀀스 — 관절/IK 프레임 자동 판별
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

        hand_future = None
        if frame.hand_motion and USE_HAND_CONTROL and hand_controller:
            hand_future = loop.run_in_executor(
                None, execute_hand_motion_sync,
                frame.hand_motion.hand, frame.hand_motion.motion, False)

        did_arm = False
        # --- IK 프레임 ---
        if frame.left_xyz and frame.right_xyz and arm:
            left_rpy = frame.left_rpy or [0.0, 0.0, 0.0]
            right_rpy = frame.right_rpy or [0.0, 0.0, 0.0]
            await loop.run_in_executor(None, _move_ik, frame.left_xyz,
                                       frame.right_xyz, left_rpy, right_rpy,
                                       frame.duration)
            current_ik_position = {"left": frame.left_xyz, "right": frame.right_xyz}
            current_rpy = {"left": left_rpy, "right": right_rpy}
            did_arm = True

        # --- 관절 프레임 ---
        elif frame.pose and frame.pose.targets and arm:
            arm_targets = np.degrees(arm.arm_ctrl.q_target).tolist()
            waist_targets = np.degrees(arm.arm_ctrl.waist_q_target).tolist()
            has_waist = False
            for t in frame.pose.targets:
                if 0 <= t.motor_index <= 2:
                    waist_targets[t.motor_index] = t.target_degree
                    has_waist = True
                elif 15 <= t.motor_index <= 28:
                    arm_targets[GLOBAL_TO_INTERNAL[t.motor_index]] = t.target_degree
                elif 3 <= t.motor_index <= 16:
                    # 구버전 호환 (내부 인덱스로 저장된 모션 파일)
                    arm_targets[t.motor_index] = t.target_degree
            tasks = [loop.run_in_executor(None, arm.move_joints_smooth,
                                          arm_targets, frame.duration)]
            if has_waist:
                tasks.append(loop.run_in_executor(
                    None, lambda: arm.move_waist_smooth(
                        yaw=waist_targets[0], roll=waist_targets[1],
                        pitch=waist_targets[2], duration=frame.duration)))
            await asyncio.gather(*tasks)
            did_arm = True

        # --- 걷기 ---
        if frame.locomotion and loco:
            dmap = {"forward": loco.forward, "backward": loco.backward,
                    "left": loco.left, "right": loco.right,
                    "turn_left": loco.turn_left, "turn_right": loco.turn_right}
            method = dmap.get(frame.locomotion.direction)
            if method:
                start = time.time()
                while time.time() - start < frame.duration:
                    if STOP_REQUESTED:
                        break
                    method()
                    await asyncio.sleep(0.02)
                if not STOP_REQUESTED:
                    loco.stop()
        elif not did_arm:
            await asyncio.sleep(frame.duration)

        if hand_future:
            await hand_future

    if STOP_REQUESTED:
        await emergency_stop()
        STOP_REQUESTED = False
    else:
        print("[모션] 완료")
        if loco:
            loco.stop()
    return {"status": "success"}


@app.post("/stop_motion")
async def stop_motion():
    global STOP_REQUESTED
    print("[정지] 요청")
    STOP_REQUESTED = True
    await emergency_stop()
    return {"status": "success"}


# ==========================================
# UI — 통합 simulator.html 단일 파일
# ==========================================
@app.get("/", response_class=HTMLResponse)
async def read_root():
    p = os.path.join(current_dir, "simulator.html")
    return FileResponse(p) if os.path.exists(p) else HTMLResponse("simulator.html 없음")


if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)
