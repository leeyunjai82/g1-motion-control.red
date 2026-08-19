"""
arm_server.py — G1 팔/허리 전용 서버 (포트 50022)

rt/arm_sdk 를 이 프로세스가 단독 점유한다. 팔·허리를 움직이는 모든
프로그램(robot_server, 외부 프로젝트, SLAM 개발자)은 반드시 이 서버의
HTTP API 를 통해서만 지령한다. arm_sdk 직접 송신 프로세스는 이것 하나.

기능
  - hold / release        : arm_sdk weight 램프 (제어권 점유/반납)
  - /joints               : 팔 14축 관절각 보간 (deg)
  - /hands                : IK 좌표 이동 (torso 기준 xyz + 쿼터니언/RPY)
  - /waist                : 허리 3축 (deg)
  - /park                 : 기본 자세 복귀 (허리 중립 + DEFAULT_ARM_DEG)
  - /freeze /stop_motion  : 현재 자세 동결 / 보간 중단
  - /pose /status         : 실측·타겟·weight 조회
  - 동작 엔드포인트는 완료까지 블로킹 응답 (시퀀스 작성 용이).
    동시에 두 동작 요청이 오면 뒤엣것은 409.

실행
  python arm_server.py          # ChannelFactoryInitialize(0) — robot_server와 동일
"""

import os
import sys
import json
import time
import asyncio
import threading
import numpy as np
from typing import List, Optional

import uvicorn
import pinocchio as pin
from fastapi import FastAPI, HTTPException
from fastapi.responses import JSONResponse
from pydantic import BaseModel
from contextlib import asynccontextmanager

current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_dir)

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from ctrl.arm_controller_wrapper import ArmControllerWrapper

PORT = 50022

# 기본 자세 (robot_server 와 동일 값 유지)
DEFAULT_ARM_DEG = [16.6,  11.7, -0.1, 56.2,  4.3, -0.9, 1.5,
                   16.3, -12.0,  1.6, 56.4, -7.6,  1.6, 0.7]
PARK_WAIST_DEG = [0.0, 0.0, 0.0]   # yaw, roll, pitch

arm: Optional[ArmControllerWrapper] = None
ARM_MODE = "hold"                  # hold | release (기동 시 weight=1)
_switching = False
# 팔과 허리는 병렬 지령 가능 (robot_server 모션 러너가 joints+waist 동시 실행).
# 같은 그룹 내 동시 동작만 409. hold/release/park 는 둘 다 잡는다.
_arm_lock   = threading.Lock()     # joints / hands
_waist_lock = threading.Lock()     # waist


# ---------------- Pydantic ----------------
class JointsReq(BaseModel):
    deg: List[float]               # 14개 (좌7+우7)
    duration: float = 2.0

class WaistReq(BaseModel):
    yaw: float = 0.0               # deg
    roll: float = 0.0
    pitch: float = 0.0
    duration: float = 1.5

class HandsReq(BaseModel):
    left_xyz: List[float]
    right_xyz: List[float]
    left_quat: Optional[List[float]] = None    # [w,x,y,z]
    right_quat: Optional[List[float]] = None
    left_rpy: Optional[List[float]] = None     # deg (quat 없을 때)
    right_rpy: Optional[List[float]] = None
    duration: float = 2.0
    frequency: int = 100

class RampReq(BaseModel):
    duration: float = 2.0

class ParkReq(BaseModel):
    duration: float = 2.0
    arm_deg: Optional[List[float]] = None      # 미지정 시 DEFAULT_ARM_DEG


def _rpy_to_quat(roll_deg, pitch_deg, yaw_deg):
    r, p, y = np.radians(roll_deg), np.radians(pitch_deg), np.radians(yaw_deg)
    cr, sr = np.cos(r/2), np.sin(r/2)
    cp, sp = np.cos(p/2), np.sin(p/2)
    cy, sy = np.cos(y/2), np.sin(y/2)
    return pin.Quaternion(cr*cp*cy + sr*sp*sy, sr*cp*cy - cr*sp*sy,
                          cr*sp*cy + sr*cp*sy, cr*cp*sy - sr*sp*cy).normalized()


def _quat_from(quat, rpy):
    if quat:
        return pin.Quaternion(quat[0], quat[1], quat[2], quat[3]).normalized()
    if rpy and any(v != 0 for v in rpy):
        return _rpy_to_quat(*rpy)
    return None


def _acquire_or_409(*locks):
    got = []
    for lk in locks:
        if lk.acquire(blocking=False):
            got.append(lk)
        else:
            for g in got:
                g.release()
            raise HTTPException(409, "다른 동작 진행 중")
    return got


async def _run_blocking(locks, fn, *args):
    """동작을 executor 에서 완료까지 실행. locks 는 진입 전에 잡혀 있어야 함."""
    loop = asyncio.get_running_loop()
    try:
        return await loop.run_in_executor(None, fn, *args)
    finally:
        for lk in locks:
            lk.release()


# ---------------- Lifespan ----------------
@asynccontextmanager
async def lifespan(app: FastAPI):
    global arm
    print("[arm_server] 시작")
    ChannelFactoryInitialize(0)
    arm = ArmControllerWrapper(motion_mode=True, simulation_mode=False)
    arm.start()
    print(f"[arm_server] 준비 완료 (weight=1, hold)  http://localhost:{PORT}/")
    yield
    # 종료: 자세 유지, 제어권만 천천히 반납
    print("[arm_server] 종료 — 제어권 반납")
    try:
        if arm and arm.arm_ctrl and arm.arm_ctrl.motion_mode:
            w = arm.arm_ctrl.get_weight()
            if w > 0.0:
                arm.arm_ctrl.ramp_weight(0.0, 1.0)
    except Exception as e:
        print(f"[arm_server] 반납 실패: {e}")
    os._exit(0)


app = FastAPI(title="G1 Arm Server", lifespan=lifespan)


# ---------------- 조회 ----------------
@app.get("/status")
async def status():
    if not arm or not arm.arm_ctrl:
        return {"ready": False}
    return {"ready": True, "mode": ARM_MODE, "switching": _switching,
            "weight": arm.arm_ctrl.get_weight(),
            "busy": _arm_lock.locked() or _waist_lock.locked()}


@app.get("/pose")
async def pose():
    """실측/타겟 관절각. 각도는 deg, *_rad 는 rad."""
    if not arm or not arm.arm_ctrl:
        raise HTTPException(503, "Arm 미초기화")
    ctrl = arm.arm_ctrl
    meas = np.asarray(ctrl.get_current_dual_arm_q(), dtype=float)
    waist = np.asarray(ctrl.get_waist_q(), dtype=float)
    with ctrl.ctrl_lock:
        tgt = np.asarray(ctrl.q_target, dtype=float).copy()
        wt = np.asarray(getattr(ctrl, "waist_q_target", np.zeros(3)),
                        dtype=float).copy()
    return {"arm_deg": np.degrees(meas).round(3).tolist(),
            "arm_target_deg": np.degrees(tgt).round(3).tolist(),
            "waist_deg": np.degrees(waist).round(3).tolist(),
            "waist_target_deg": np.degrees(wt).round(3).tolist(),
            "arm_rad": meas.round(5).tolist(),
            "waist_rad": waist.round(5).tolist()}


# ---------------- 제어권 ----------------
def _do_hold(duration):
    global ARM_MODE, _switching
    try:
        arm.arm_ctrl.sync_targets_to_current()
        arm.arm_ctrl.ramp_weight(1.0, duration)
        ARM_MODE = "hold"
        print("[ARM] hold 완료")
    finally:
        _switching = False


def _do_release(duration, arm_deg):
    global ARM_MODE, _switching
    try:
        # loco 기본자세로 보간 후 반납 — 인계 순간 튐 최소화
        arm.move_waist_smooth(yaw=PARK_WAIST_DEG[0], roll=PARK_WAIST_DEG[1],
                              pitch=PARK_WAIST_DEG[2], duration=duration)
        arm.move_joints_smooth(arm_deg, duration)
        arm.arm_ctrl.ramp_weight(0.0, duration)
        ARM_MODE = "release"
        print("[ARM] release 완료")
    finally:
        _switching = False


@app.post("/hold")
async def hold(req: RampReq = RampReq()):
    global _switching
    if not arm or not arm.arm_ctrl:
        raise HTTPException(503, "Arm 미초기화")
    if ARM_MODE == "hold" and not _switching:
        return {"ok": True, "mode": ARM_MODE}
    locks = _acquire_or_409(_arm_lock, _waist_lock)
    _switching = True
    await _run_blocking(locks, _do_hold, req.duration)
    return {"ok": True, "mode": ARM_MODE}


@app.post("/release")
async def release(req: ParkReq = ParkReq()):
    global _switching
    if not arm or not arm.arm_ctrl:
        raise HTTPException(503, "Arm 미초기화")
    if ARM_MODE == "release" and not _switching:
        return {"ok": True, "mode": ARM_MODE}
    locks = _acquire_or_409(_arm_lock, _waist_lock)
    _switching = True
    await _run_blocking(locks, _do_release, req.duration,
                        req.arm_deg or DEFAULT_ARM_DEG)
    return {"ok": True, "mode": ARM_MODE}


# ---------------- 동작 ----------------
@app.post("/joints")
async def joints(req: JointsReq):
    if not arm:
        raise HTTPException(503, "Arm 미초기화")
    if len(req.deg) != 14:
        raise HTTPException(400, "deg 는 14개")
    locks = _acquire_or_409(_arm_lock)
    await _run_blocking(locks, arm.move_joints_smooth, list(req.deg), req.duration)
    return {"ok": True}


@app.post("/waist")
async def waist(req: WaistReq):
    if not arm:
        raise HTTPException(503, "Arm 미초기화")
    locks = _acquire_or_409(_waist_lock)

    def _do():
        arm.move_waist_smooth(yaw=req.yaw, roll=req.roll,
                              pitch=req.pitch, duration=req.duration)
    await _run_blocking(locks, _do)
    return {"ok": True}


@app.post("/hands")
async def hands(req: HandsReq):
    if not arm:
        raise HTTPException(503, "Arm 미초기화")
    l_rot = _quat_from(req.left_quat, req.left_rpy)
    r_rot = _quat_from(req.right_quat, req.right_rpy)
    locks = _acquire_or_409(_arm_lock)

    def _do():
        arm.move_hands(list(req.left_xyz), list(req.right_xyz),
                       l_rot, r_rot, req.duration, req.frequency)
    await _run_blocking(locks, _do)
    return {"ok": True}


@app.post("/park")
async def park(req: ParkReq = ParkReq()):
    if not arm:
        raise HTTPException(503, "Arm 미초기화")
    locks = _acquire_or_409(_arm_lock, _waist_lock)

    def _do():
        arm.move_waist_smooth(yaw=PARK_WAIST_DEG[0], roll=PARK_WAIST_DEG[1],
                              pitch=PARK_WAIST_DEG[2], duration=req.duration)
        arm.move_joints_smooth(req.arm_deg or DEFAULT_ARM_DEG, req.duration)
    await _run_blocking(locks, _do)
    return {"ok": True}


@app.post("/stop_motion")
async def stop_motion():
    """진행 중 보간 중단 (자세는 그 자리에 멈춤). 언제든 호출 가능."""
    if not arm:
        raise HTTPException(503, "Arm 미초기화")
    try:
        arm.stop_motion()
    except Exception as e:
        return JSONResponse({"ok": False, "error": str(e)})
    return {"ok": True}


@app.post("/freeze")
async def freeze():
    """보간 중단 + 팔·허리 타겟을 현재 실측각으로 동결."""
    if not arm:
        raise HTTPException(503, "Arm 미초기화")
    try:
        arm.stop_motion()
    except Exception:
        pass
    try:
        ctrl = arm.arm_ctrl
        all_q = np.asarray(ctrl.get_current_motor_q(), dtype=float)
        with ctrl.ctrl_lock:
            ctrl.q_target = np.asarray(ctrl.get_current_dual_arm_q(), dtype=float)
            ctrl.tauff_target = np.zeros(14)
            ctrl.waist_q_target = all_q[12:15].copy()
    except Exception as e:
        return JSONResponse({"ok": False, "error": str(e)})
    return {"ok": True}


if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=PORT)

