#!/usr/bin/env python3
# Version: 1.2
# Changes:
#   1.2 - grab_box가 L/R 실제좌표 직접 사용(기울어진 박스 양손 정확)
#   1.1 - handover 허리 yaw 회전 각도비례 감속(90도시 느리게), reset 1.5초
#   1.0 - box 놓기 5초→3초, 미수령 시 약간 내려놓기(타임아웃)
#   0.9 - TTS 멘트 선물 컨셉 제거, 잡기 위주로 변경
#   0.8 - handover 받음 처리 분리 (marker=가림감지 / box=고정5초)
#   0.7 - park 자세 [0.0,±0.28,-0.38] 차렷에 가깝게
#   0.6 - WAIST_BASE_PITCH 상수 (현재 0.0)
#   0.5 - set_mode 시 ready/park 자세, marker_x_axis reshape 방어
#   0.4 - align 후 재감지(redetect) 추가, _run_grab 트레이스백
#   0.3 - 종료 시 팔 자세 유지(제어권만 반납), 포트 50000
#   0.2 - viewer 제거(dashboard 담당), robot_web.html 분리
#   0.1 - run_motion + grab_core 통합 초기본
"""
robot_server.py — G1 통합 로봇 제어 서버 (arm 제어 유일 프로세스)

run_motion.py 기능:
  · 관절/IK 모션 실행 (/run, /run_ik, /motions/run, /send_gift)
  · Loco 방향키 (/loco/move, /loco/stop)
  · 컨트롤 웹 UI (viewer는 dashboard.py)

추가 (잡기):
  · GrabController (grab_core.py) — marker/box 잡기 시퀀스
  · POST /grab_at      — 인식 파일이 좌표 주면 잡기 실행
  · GET  /active_mode  — 현재 모드 (인식 파일이 폴링)
  · POST /set_mode     — 웹에서 marker/box/none 전환
  · GET  /grab_status  — busy 등

인식은 별도 프로세스 (detect_marker.py:50011, detect_box.py:50010)가
담당하고 결과 좌표만 POST /grab_at 로 전달한다.
이 파일만 ArmControllerWrapper(arm)를 점유한다.
"""

import os
import sys
import json
import time
import asyncio
import threading
import numpy as np
from pathlib import Path
from contextlib import asynccontextmanager
from typing import List, Optional

import uvicorn
import pinocchio as pin
from fastapi import FastAPI, UploadFile, File, HTTPException
from fastapi.responses import FileResponse, HTMLResponse, StreamingResponse, Response, JSONResponse
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel

current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_dir)

# ===== 경로 =====
MOTIONS_DIR = Path(current_dir) / "motions"
MOTIONS_DIR.mkdir(exist_ok=True)
ASSETS_DIR  = os.path.join(current_dir, 'assets', 'g1')
URDF_PATH   = os.path.join(ASSETS_DIR, 'g1_29dof_rev_1_0.urdf')
MESH_DIR    = os.path.join(ASSETS_DIR, 'meshes')
VENDOR_DIR  = os.path.join(current_dir, 'assets', 'vendor')

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from ctrl.arm_controller_wrapper import ArmControllerWrapper, LocoClientWrapper, GLOBAL_TO_INTERNAL




# ==========================================
# 카메라 → torso 좌표 변환 (ik_box와 동일 상수)
# ==========================================
CAMERA_X          = 0.0576235
CAMERA_Y          = 0.03003
CAMERA_Z          = 0.42987
CAMERA_PITCH_URDF = 0.8307767239493009  # 47.6도


def camera_to_torso(cx, cy, cz):
    cos_p, sin_p = np.cos(CAMERA_PITCH_URDF), np.sin(CAMERA_PITCH_URDF)
    cx_r =  cx
    cy_r =  cy * cos_p + cz * sin_p
    cz_r = -cy * sin_p + cz * cos_p
    return (float(cz_r + CAMERA_X),
            float(-cx_r + CAMERA_Y),
            float(-cy_r + CAMERA_Z))


def camera_dir_to_torso(dx, dy, dz):
    cos_p, sin_p = np.cos(CAMERA_PITCH_URDF), np.sin(CAMERA_PITCH_URDF)
    dx_r =  dx
    dy_r =  dy * cos_p + dz * sin_p
    dz_r = -dy * sin_p + dz * cos_p
    return float(dz_r), float(-dx_r), float(-dy_r)


def marker_x_axis_in_torso(rvec):
    """마커 X축을 torso XY 평면에 투영한 grip 방향."""
    try:
        rv = np.asarray(rvec, dtype=np.float64).reshape(3, 1)
        R, _ = cv2.Rodrigues(rv)
        x_cam = R[:, 0]
        tx, ty, tz = camera_dir_to_torso(x_cam[0], x_cam[1], x_cam[2])
        v = np.array([tx, ty, tz])
        if np.linalg.norm(v) < 1e-6:
            return np.array([0.0, 1.0, 0.0])
        v[2] = 0.0
        n = np.linalg.norm(v)
        if n < 1e-6:
            return np.array([0.0, 1.0, 0.0])
        return v / n
    except Exception as e:
        print(f"[marker_x_axis] 오류, 기본 방향 사용: {e}")
        return np.array([0.0, 1.0, 0.0])


# ==========================================
# 잡기 파라미터 (ik_box와 동일)
# ==========================================
GRIP_EXTRA     = -0.050
APPROACH_EXTRA = 0.10
GRAB_Z_OFFSET  = 0.08
GRAB_X_OFFSET  = -0.15
HANDOVER_X     = 0.30
LEFT_HAND_Y_OFFSET = 0.0
WAIST_BASE_PITCH = 0.0   # 기본 상체 각도 (0=중립)


# ==========================================
# GrabController
# ==========================================
class GrabController:
    """잡기 시퀀스 실행기.

    robot_server가 arm, speak, wrist_params, handover 설정을 주입.
    """
    def __init__(self, arm=None, speak=None,
                 robot_available=False):
        self.arm  = arm
        self.speak = speak or (lambda t: print(f"[TTS-DUMMY] {t}"))
        self.robot_available = robot_available

        # 손목 RPY
        self.wrist_params = {
            'left':  {'roll': -10.0, 'pitch': -10.0, 'yaw': -15.0},
            'right': {'roll':  10.0, 'pitch': -10.0, 'yaw':  15.0},
        }
        # handover 방향
        self.handover_direction = "center"   # center|left|right
        self.handover_yaw_deg   = 30.0

        # 박스 크기 (marker 모드 고정값, box 모드는 측정값 사용)
        self.box_size = {"width": 0.28, "depth": 0.09, "height": 0.09}

        # TTS 멘트 (선물 컨셉 제거, 잡기 위주)
        self.MSG_PICKED   = "I got it."
        self.MSG_HANDOVER = "Here you go. Please take the box."
        self.MSG_RECEIVED = "Nicely done!"
        self.MSG_TIMEOUT  = "No one? I will put it down."
        self.MSG_HOME      = "Bring me another box."

        self.HOME_LEFT  = [0.15,  0.25, 0.20]
        self.HOME_RIGHT = [0.15, -0.25, 0.20]

        # 허리 정렬 후 재감지 콜백 (robot_server가 주입) — None이면 재감지 안 함
        self.redetect = None
        self._last_kind = "marker"   # 마지막 잡기 종류 (handover 가림 판정용)

    # ---- 로봇 저수준 래퍼 ----
    def _rpy_to_quat(self, roll_deg, pitch_deg, yaw_deg):
        r, p, y = np.radians(roll_deg), np.radians(pitch_deg), np.radians(yaw_deg)
        cr, sr = np.cos(r/2), np.sin(r/2)
        cp, sp = np.cos(p/2), np.sin(p/2)
        cy, sy = np.cos(y/2), np.sin(y/2)
        w = cr*cp*cy + sr*sp*sy
        x = sr*cp*cy - cr*sp*sy
        yq= cr*sp*cy + sr*cp*sy
        z = cr*cp*sy - sr*sp*cy
        return pin.Quaternion(w, x, yq, z).normalized()

    def _move(self, left_xyz, right_xyz, duration, msg="",
              left_rot=None, right_rot=None):
        print(f"[IK] {msg}  L:{[f'{v:.3f}' for v in left_xyz]}  "
              f"R:{[f'{v:.3f}' for v in right_xyz]}")
        if not self.robot_available or self.arm is None:
            time.sleep(duration)
            return True
        try:
            self.arm.move_hands(left_xyz, right_xyz,
                                left_rot, right_rot, duration, 100)
            return True
        except Exception as e:
            print(f"[IK] 오류: {e}")
            return False

    def _reset_waist(self):
        print(f"[WAIST] 리셋 (pitch={WAIST_BASE_PITCH})")
        if self.robot_available and self.arm is not None:
            self.arm.move_waist_smooth(yaw=0.0, roll=0.0, pitch=WAIST_BASE_PITCH, duration=1.5)
        time.sleep(0.5)

    def _align_waist_yaw(self, mx, my):
        yaw_deg = float(np.degrees(np.arctan2(my, mx)))
        print(f"[WAIST] yaw: {yaw_deg:.1f}도")
        if abs(yaw_deg) < 1.5:
            return
        if self.robot_available and self.arm is not None:
            self.arm.move_waist_smooth(yaw=yaw_deg, roll=0.0, pitch=WAIST_BASE_PITCH, duration=1.0)
        else:
            time.sleep(1.0)
        time.sleep(0.5)

    def _wrist_quats(self):
        lp = self.wrist_params['left']
        rp = self.wrist_params['right']
        return (self._rpy_to_quat(lp['roll'], lp['pitch'], lp['yaw']),
                self._rpy_to_quat(rp['roll'], rp['pitch'], rp['yaw']))

    # ---- 공통 후반부: 대칭→들기→handover→복귀 ----
    def _finish_sequence(self, grab_x_base, grp_off_L, grp_off_R,
                         grab_z, lift_z, l_rot, r_rot):
        self.speak(self.MSG_PICKED)

        sym_L = [grab_x_base, +grp_off_L + LEFT_HAND_Y_OFFSET, grab_z]
        sym_R = [grab_x_base, -grp_off_R, grab_z]
        if not self._move(sym_L, sym_R, 1.5, "⑥' 대칭 정렬", l_rot, r_rot):
            return
        time.sleep(0.3)

        ll = [grab_x_base, +grp_off_L + LEFT_HAND_Y_OFFSET, lift_z]
        rl = [grab_x_base, -grp_off_R, lift_z]
        if not self._move(ll, rl, 1.5, "⑦ 들기", l_rot, r_rot):
            return
        time.sleep(0.2)

        if self.handover_direction == "left":
            hy = +self.handover_yaw_deg
        elif self.handover_direction == "right":
            hy = -self.handover_yaw_deg
        else:
            hy = 0.0
        print(f"[GRAB] ⑦' 허리 yaw → {hy:.1f}도")
        if self.robot_available and self.arm is not None:
            # 회전 각도가 클수록 느리게 (기본 1.5초 + 30도당 1초)
            yaw_dur = 1.5 + abs(hy) / 30.0
            self.arm.move_waist_smooth(yaw=hy, roll=0.0, pitch=WAIST_BASE_PITCH, duration=yaw_dur)
            time.sleep(0.5)

        hl = [HANDOVER_X, +grp_off_L + LEFT_HAND_Y_OFFSET, lift_z]
        hr = [HANDOVER_X, -grp_off_R, lift_z]
        if not self._move(hl, hr, 1.5, "⑧ 건네기", l_rot, r_rot):
            return
        time.sleep(0.3)
        self.speak(self.MSG_HANDOVER)

        # 받음 처리 — 종류별로 다름
        received = False
        if self._last_kind == "marker" and self.redetect is not None:
            # 마커: 박스 윗면에 마커가 붙어있어 받으면 가려짐 → 가림 감지 (최대 8초)
            start = time.time()
            while time.time() - start < 8.0:
                d = self.redetect("marker")
                if d is None:
                    received = True
                    print(f"[HANDOVER] 마커 가림 → 받음 ({time.time()-start:.1f}s)")
                    break
                time.sleep(0.2)
            if not received:
                print("[HANDOVER] 타임아웃 → 그냥 놓음")
        else:
            # 박스: 받아도 계속 보이므로 고정 3초 대기 후 놓기
            print("[HANDOVER] 박스 — 3초 대기 후 놓기")
            time.sleep(3.0)
            received = True

        self.speak(self.MSG_RECEIVED if received else self.MSG_TIMEOUT)

        if received:
            # 받음 — 그 높이에서 손 벌려 놓기
            open_L = [HANDOVER_X, +grp_off_L + 0.10 + LEFT_HAND_Y_OFFSET, lift_z]
            open_R = [HANDOVER_X, -grp_off_R - 0.10, lift_z]
            self._move(open_L, open_R, 1.0, "⑩ 손 벌림 (놓기)", l_rot, r_rot)
        else:
            # 못 받음 — 약간 내려서 살포시 놓고 손 벌림
            down_z = lift_z - 0.12
            dl = [HANDOVER_X, +grp_off_L + LEFT_HAND_Y_OFFSET, down_z]
            dr = [HANDOVER_X, -grp_off_R, down_z]
            self._move(dl, dr, 1.2, "⑩ 내려놓기", l_rot, r_rot)
            time.sleep(0.2)
            open_L = [HANDOVER_X, +grp_off_L + 0.10 + LEFT_HAND_Y_OFFSET, down_z]
            open_R = [HANDOVER_X, -grp_off_R - 0.10, down_z]
            self._move(open_L, open_R, 1.0, "⑩' 손 벌림 (놓기)", l_rot, r_rot)
        time.sleep(0.3)

        print("[HANDOVER] ⑪ 복귀")
        self._reset_waist()
        self._move(self.HOME_LEFT, self.HOME_RIGHT, 2.0, "⑪ Home")
        self.speak(self.MSG_HOME)

    # ---- 대기 자세 (모드 선택 시) ----
    def ready(self):
        """잡을 준비 — 팔을 작업 대기 자세(HOME)로 들어 올림."""
        print("[READY] 대기 자세로")
        self._reset_waist()
        l_rot, r_rot = self._wrist_quats()
        self._move(self.HOME_LEFT, self.HOME_RIGHT, 2.0, "READY 대기자세", l_rot, r_rot)

    def park(self):
        """대기 해제 — 팔을 거의 차렷 자세로 내림."""
        print("[PARK] 팔 내림")
        self._reset_waist()
        self._move([0.0, 0.28, -0.38], [0.0, -0.28, -0.38], 2.0, "PARK 팔내림")

    # ---- marker 잡기 ----
    def grab_marker(self, tvec, rvec):
        """마커: 윗면 중심 tvec + 자세 rvec, box_size 고정."""
        print("[GRAB-MARKER] 시작")
        self._last_kind = "marker"
        self._reset_waist()
        mx, my, mz = camera_to_torso(tvec[0], tvec[1], tvec[2])
        self._align_waist_yaw(mx, my)

        # 허리 돌린 후 재감지 (카메라 좌표계 보정) — ik_box와 동일
        if self.redetect is not None:
            time.sleep(0.6)
            d = self.redetect("marker")
            if d and d.get("tvec"):
                tvec = d["tvec"]
                if d.get("rvec"): rvec = d["rvec"]
                mx, my, mz = camera_to_torso(tvec[0], tvec[1], tvec[2])
                print(f"[GRAB-MARKER] 재감지 torso=[{mx:.3f},{my:.3f},{mz:.3f}]")
            else:
                print("[GRAB-MARKER] 재감지 실패 — 원래 좌표 사용")

        box_x_axis = marker_x_axis_in_torso(rvec)
        if box_x_axis[1] < 0:
            box_x_axis = -box_x_axis

        half_w   = self.box_size["width"] / 2
        height_b = self.box_size["height"]

        grab_x_base = mx + GRAB_X_OFFSET
        grab_z  = mz - height_b / 2 + GRAB_Z_OFFSET
        above_z = mz + 0.10
        lift_z  = mz + 0.15
        app_off = half_w + GRIP_EXTRA + APPROACH_EXTRA
        grp_off = half_w + GRIP_EXTRA

        gd = box_x_axis
        def offset_point(bx, by, z, off):
            return ([bx + gd[0]*off, by + gd[1]*off + LEFT_HAND_Y_OFFSET, z],
                    [bx - gd[0]*off, by - gd[1]*off, z])

        l_rot, r_rot = self._wrist_quats()

        L, R = offset_point(mx, my, above_z, app_off)
        if not self._move(L, R, 1.5, "④ 위쪽 접근", l_rot, r_rot): return
        time.sleep(0.2)
        L, R = offset_point(mx, my, grab_z, app_off)
        if not self._move(L, R, 1.0, "⑤ 측면 하강", l_rot, r_rot): return
        time.sleep(0.2)
        L, R = offset_point(grab_x_base, my, grab_z, grp_off)
        if not self._move(L, R, 2.5, "⑥ 잡기", l_rot, r_rot): return
        time.sleep(1.0)

        self._finish_sequence(grab_x_base, grp_off, grp_off,
                              grab_z, lift_z, l_rot, r_rot)

    # ---- box(cardboard) 잡기 ----
    def grab_box(self, L_cam, R_cam, box_h_m=None, top_center_cam=None):
        """박스: L/R(윗면 좌우 변 중심, 안쪽 2cm) 직접 사용.

        L_cam, R_cam: 카메라 좌표 grip 점
        box_h_m: 측정된 박스 높이 (잡는 높이 결정용)
        top_center_cam: 윗면 중심 (waist yaw 정렬용)
        """
        print("[GRAB-BOX] 시작")
        self._last_kind = "box"
        self._reset_waist()

        Lx, Ly, Lz = camera_to_torso(L_cam[0], L_cam[1], L_cam[2])
        Rx, Ry, Rz = camera_to_torso(R_cam[0], R_cam[1], R_cam[2])

        # 중심 (waist 정렬 + grab_x_base)
        if top_center_cam is not None:
            cx, cy, cz = camera_to_torso(top_center_cam[0],
                                          top_center_cam[1],
                                          top_center_cam[2])
        else:
            cx, cy, cz = (Lx+Rx)/2, (Ly+Ry)/2, (Lz+Rz)/2
        self._align_waist_yaw(cx, cy)

        # 허리 돌린 후 재감지 (카메라 좌표계 보정)
        if self.redetect is not None:
            time.sleep(0.6)
            d = self.redetect("box")
            if d and d.get("L") and d.get("R"):
                L_cam, R_cam = d["L"], d["R"]
                if d.get("box_h"): box_h_m = d["box_h"]
                if d.get("top_center"): top_center_cam = d["top_center"]
                Lx, Ly, Lz = camera_to_torso(L_cam[0], L_cam[1], L_cam[2])
                Rx, Ry, Rz = camera_to_torso(R_cam[0], R_cam[1], R_cam[2])
                if top_center_cam is not None:
                    cx, cy, cz = camera_to_torso(*top_center_cam)
                else:
                    cx, cy, cz = (Lx+Rx)/2, (Ly+Ry)/2, (Lz+Rz)/2
                print(f"[GRAB-BOX] 재감지 center=[{cx:.3f},{cy:.3f},{cz:.3f}]")
            else:
                print("[GRAB-BOX] 재감지 실패 — 원래 좌표 사용")

        # 잡는 높이: 윗면(=L/R z)에서 박스 H 절반 내려 옆면 중간
        h = box_h_m if box_h_m else 0.065
        top_z = (Lz + Rz) / 2
        grab_z  = top_z - h / 2 + GRAB_Z_OFFSET
        above_z = top_z + 0.10
        lift_z  = top_z + 0.15

        l_rot, r_rot = self._wrist_quats()

        # === L/R 실제 좌표를 직접 손 목표로 사용 (기울어진 박스 대응) ===
        # 왼손 = L점, 오른손 = R점 (각 변의 실제 위치)
        # 접근: L/R에서 바깥으로 더 벌려 위에서 내려옴
        #   바깥 방향 = 중심(cx,cy)에서 L/R로 향하는 단위벡터
        def outward(px, py):
            dx, dy = px - cx, py - cy
            n = (dx*dx + dy*dy) ** 0.5
            return (dx/n, dy/n) if n > 1e-6 else (0.0, 0.0)
        oLx, oLy = outward(Lx, Ly)
        oRx, oRy = outward(Rx, Ry)

        # 접근점 (L/R에서 바깥 +APPROACH_EXTRA, X는 안 당김)
        appL = [Lx + oLx*APPROACH_EXTRA, Ly + oLy*APPROACH_EXTRA + LEFT_HAND_Y_OFFSET, above_z]
        appR = [Rx + oRx*APPROACH_EXTRA, Ry + oRy*APPROACH_EXTRA, above_z]
        if not self._move(appL, appR, 1.5, "④ 위쪽 접근", l_rot, r_rot): return
        time.sleep(0.2)

        # 하강 (같은 XY, grab_z로)
        appL[2] = grab_z; appR[2] = grab_z
        if not self._move(appL, appR, 1.0, "⑤ 측면 하강", l_rot, r_rot): return
        time.sleep(0.2)

        # 잡기 — 실제 L/R 점 + X는 몸쪽으로 당김(GRAB_X_OFFSET)
        gripL = [Lx + GRAB_X_OFFSET, Ly + LEFT_HAND_Y_OFFSET, grab_z]
        gripR = [Rx + GRAB_X_OFFSET, Ry, grab_z]
        if not self._move(gripL, gripR, 2.5, "⑥ 잡기", l_rot, r_rot): return
        time.sleep(1.0)

        # 대칭 정렬용 파라미터: 잡은 뒤 양손을 평행/대칭으로 정리
        grab_x_base = cx + GRAB_X_OFFSET
        grp_off_L = abs(Ly - cy)
        grp_off_R = abs(Ry - cy)
        self._finish_sequence(grab_x_base, grp_off_L, grp_off_R,
                              grab_z, lift_z, l_rot, r_rot)



try:
    from ctrl.mandro3 import HandController, motions as hand_motions
    HAND_AVAILABLE = True
except ImportError:
    HAND_AVAILABLE = False

try:
    from ctrl.text_to_speech import TextToSpeech
    TTS_AVAILABLE = True
except ImportError:
    TTS_AVAILABLE = False



# ==========================================
# 전역 상태
# ==========================================
arm:  Optional[ArmControllerWrapper] = None
loco: Optional[LocoClientWrapper]    = None
hand: Optional[object]               = None
tts:  Optional[object]               = None
grab: Optional[GrabController]       = None

is_running = False
STOP_FLAG  = False

# 잡기 모드 게이트
ACTIVE_MODE = "none"          # "none" | "marker" | "box"
grab_busy   = False
grab_lock   = threading.Lock()


# ==========================================
# Pydantic
# ==========================================
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
    duration: float
    pose: Optional[PoseData] = None
    locomotion: Optional[LocomotionData] = None
    hand_motion: Optional[HandMotionData] = None

class IKMotionFrame(BaseModel):
    duration: float
    left_xyz: Optional[List[float]] = None
    right_xyz: Optional[List[float]] = None
    left_rpy: Optional[List[float]] = None
    right_rpy: Optional[List[float]] = None
    locomotion: Optional[LocomotionData] = None
    hand_motion: Optional[HandMotionData] = None

class LocoMoveRequest(BaseModel):
    vx: float = 0.0
    vy: float = 0.0
    vyaw: float = 0.0

# 잡기 요청 (인식 파일 → robot_server)
class GrabRequest(BaseModel):
    type: str                              # "marker" | "cardboard"
    tvec: Optional[List[float]] = None     # 카메라 좌표 (marker: 윗면중심)
    rvec: Optional[List[float]] = None     # marker 자세
    L:    Optional[List[float]] = None     # box 왼쪽 grip (카메라)
    R:    Optional[List[float]] = None     # box 오른쪽 grip
    top_center: Optional[List[float]] = None
    box_h: Optional[float] = None          # box 높이 (m)


# ==========================================
# 헬퍼 (run_motion 동일)
# ==========================================
def rpy_to_quaternion(roll_deg, pitch_deg, yaw_deg):
    roll, pitch, yaw = np.radians(roll_deg), np.radians(pitch_deg), np.radians(yaw_deg)
    cr, sr = np.cos(roll/2), np.sin(roll/2)
    cp, sp = np.cos(pitch/2), np.sin(pitch/2)
    cy, sy = np.cos(yaw/2), np.sin(yaw/2)
    w = cr*cp*cy + sr*sp*sy
    x = sr*cp*cy - cr*sp*sy
    y = cr*sp*cy + sr*cp*sy
    z = cr*cp*sy - sr*sp*cy
    return pin.Quaternion(w, x, y, z).normalized()

def move_hands_with_rotation(left_xyz, right_xyz, left_rpy, right_rpy, duration, frequency=100):
    left_rot  = rpy_to_quaternion(*left_rpy)  if left_rpy  and any(v != 0 for v in left_rpy)  else None
    right_rot = rpy_to_quaternion(*right_rpy) if right_rpy and any(v != 0 for v in right_rpy) else None
    arm.move_hands(left_xyz, right_xyz, left_rot, right_rot, duration, frequency)

def execute_hand_motion_sync(h: str, motion: str):
    if hand:
        hand.send_motion(motion, selector=h)



# ==========================================
# Lifespan
# ==========================================
@asynccontextmanager
async def lifespan(app: FastAPI):
    global arm, loco, hand, tts, grab, ACTIVE_MODE

    print("[robot_server] 시작")
    ChannelFactoryInitialize(0)

    try:
        loco = LocoClientWrapper()
        print("✅ Loco 초기화")
    except Exception as e:
        print(f"⚠️ Loco 실패: {e}")

    try:
        arm = ArmControllerWrapper(motion_mode=True, simulation_mode=False)
        arm.start()
        print("✅ Arm 초기화")
    except Exception as e:
        print(f"⚠️ Arm 실패: {e}")
        arm = None

    if HAND_AVAILABLE:
        try:
            hand = HandController('/dev/ttyACM0')
            print("✅ 손 초기화")
        except Exception as e:
            print(f"⚠️ 손 실패: {e}")

    if TTS_AVAILABLE:
        try:
            tts = TextToSpeech(verbose=False)
            print("✅ TTS 초기화")
        except Exception as e:
            print(f"⚠️ TTS 실패: {e}")

    # 잡기 컨트롤러 (arm 주입)
    def _speak(text):
        if tts:
            tts.speak(text)
        else:
            print(f"[TTS-DUMMY] {text}")
    grab = GrabController(arm=arm, speak=_speak,
                          robot_available=(arm is not None))

    # 허리 정렬 후 재감지: 현재 모드의 detect 서버 /pose를 다시 읽음
    def _redetect(kind):
        import urllib.request as _u
        port = 50011 if kind == "marker" else 50010
        try:
            raw = _u.urlopen(f"http://localhost:{port}/pose", timeout=1.0).read()
            d = json.loads(raw)
            return d if d.get("found") else None
        except Exception as e:
            print(f"[REDETECT] 실패: {e}")
            return None
    grab.redetect = _redetect
    print("✅ GrabController 준비")

    print("[robot_server] 준비 완료  http://localhost:50000/")
    yield

    # ==========================================
    # 안전 종료 시퀀스 (팔 자세 유지, 제어권만 반납)
    # ==========================================
    print("[shutdown] 종료 시퀀스 시작")
    t_shutdown = time.time()

    # ACTIVE_MODE 자동 트리거 방지를 위해 none으로 전환
    ACTIVE_MODE = "none"

    # 잡기 진행 중이면 잠깐 대기
    busy_deadline = time.time() + 3.0
    while grab_busy and time.time() < busy_deadline:
        time.sleep(0.1)
    if grab_busy:
        print("[shutdown] grab 진행 중이지만 시간 초과 — 강제 진행")

    if arm:
        try:
            # 팔 자세는 건드리지 않음 (현재 위치 그대로 멈춤)
            # 제어권만 천천히 반납 (weight 1→0, 1초) — 팔이 확 안 떨어지게
            if arm.arm_ctrl and arm.arm_ctrl.motion_mode:
                from ctrl.robot_arm import G1_29_JointIndex
                print("[shutdown] 제어권 반납 (weight 1->0, 1초)")
                for weight in np.linspace(1.0, 0.0, num=50):
                    arm.arm_ctrl.msg.motor_cmd[G1_29_JointIndex.kNotUsedJoint0].q = weight
                    time.sleep(0.02)
                print("[shutdown] 제어권 반납 완료")
        except Exception as e:
            print(f"[shutdown] 종료 시퀀스 실패: {e}")

    time.sleep(0.3)
    print(f"[robot_server] 종료 (총 {time.time()-t_shutdown:.2f}초)")
    os._exit(0)


app = FastAPI(title="G1 Robot Server", lifespan=lifespan)
app.add_middleware(CORSMiddleware, allow_origins=["*"],
                   allow_methods=["*"], allow_headers=["*"])


# ==========================================
# 모션 실행 (run_motion 동일)
# ==========================================
async def _run_loco(direction: str, duration: float):
    dmap = {"forward":loco.forward,"backward":loco.backward,"left":loco.left,
            "right":loco.right,"turn_left":loco.turn_left,"turn_right":loco.turn_right}
    method = dmap.get(direction)
    if method and loco:
        start = time.time()
        while time.time() - start < duration:
            if STOP_FLAG: break
            method()
            await asyncio.sleep(0.02)
        if not STOP_FLAG and loco:
            loco.stop()

async def _execute_frames(frames: List[MotionFrame]):
    global is_running, STOP_FLAG
    is_running = True; STOP_FLAG = False
    loop = asyncio.get_running_loop()
    try:
        for i, frame in enumerate(frames):
            if STOP_FLAG: break
            print(f"[Runner] 프레임 {i+1}/{len(frames)}")
            hand_future = None
            if frame.hand_motion and hand:
                hand_future = loop.run_in_executor(None, execute_hand_motion_sync,
                                                    frame.hand_motion.hand, frame.hand_motion.motion)
            if frame.pose and frame.pose.targets and arm:
                with arm.arm_ctrl.ctrl_lock:
                    arm_targets = np.degrees(arm.arm_ctrl.q_target.copy())
                try:
                    with arm.arm_ctrl.ctrl_lock:
                        waist_targets = np.degrees(getattr(arm.arm_ctrl,'waist_q_target',np.zeros(3)).copy())
                except:
                    waist_targets = np.zeros(3)
                has_waist = False
                for t in frame.pose.targets:
                    if 0 <= t.motor_index <= 2:
                        waist_targets[t.motor_index] = t.target_degree; has_waist = True
                    elif 15 <= t.motor_index <= 28:
                        arm_targets[GLOBAL_TO_INTERNAL[t.motor_index]] = t.target_degree
                tasks = [loop.run_in_executor(None, arm.move_joints_smooth, arm_targets.tolist(), frame.duration)]
                if has_waist:
                    tasks.append(loop.run_in_executor(None, arm.move_waist_smooth,
                        float(waist_targets[0]),float(waist_targets[1]),float(waist_targets[2]),frame.duration))
                await asyncio.gather(*tasks)
            elif frame.locomotion and loco:
                await _run_loco(frame.locomotion.direction, frame.duration)
            else:
                await asyncio.sleep(frame.duration)
            if hand_future: await hand_future
    finally:
        is_running = False
        if loco: loco.stop()

async def _execute_ik_frames(frames: List[IKMotionFrame]):
    global is_running, STOP_FLAG
    is_running = True; STOP_FLAG = False
    loop = asyncio.get_running_loop()
    try:
        for i, frame in enumerate(frames):
            if STOP_FLAG: break
            print(f"[IK Runner] 프레임 {i+1}/{len(frames)}")
            hand_future = None
            if frame.hand_motion and hand:
                hand_future = loop.run_in_executor(None, execute_hand_motion_sync,
                                                    frame.hand_motion.hand, frame.hand_motion.motion)
            if frame.left_xyz and frame.right_xyz and arm:
                lr = frame.left_rpy or [0.0,0.0,0.0]
                rr = frame.right_rpy or [0.0,0.0,0.0]
                await loop.run_in_executor(None, move_hands_with_rotation,
                    frame.left_xyz, frame.right_xyz, lr, rr, frame.duration, 100)
            elif frame.locomotion and loco:
                await _run_loco(frame.locomotion.direction, frame.duration)
            else:
                await asyncio.sleep(frame.duration)
            if hand_future: await hand_future
    finally:
        is_running = False
        if loco: loco.stop()

async def _execute_send_gift():
    loop = asyncio.get_running_loop()
    filepath = MOTIONS_DIR / "right_send.json"
    if not filepath.exists():
        print("[Send] right_send.json 없음"); return
    try:
        with open(filepath, "r", encoding="utf-8") as f:
            data = json.load(f)
    except Exception as e:
        print(f"[Send] 파싱 오류: {e}"); return
    if not data: return
    if tts:
        tts.speak("Here you go.")
        await loop.run_in_executor(None, tts.wait_until_done)
    first = data[0]
    is_ik = "left_xyz" in first or "right_xyz" in first
    if is_ik:
        await _execute_ik_frames([IKMotionFrame(**f) for f in data])
    else:
        await _execute_frames([MotionFrame(**f) for f in data])
    if tts:
        tts.speak("All done. Thank you.")


# ==========================================
# 잡기 — 모드 게이트 + grab_at
# ==========================================
def _run_grab(req: GrabRequest):
    """별도 스레드에서 잡기 시퀀스 실행."""
    global grab_busy
    try:
        if req.type == "marker":
            grab.grab_marker(req.tvec, req.rvec)
        elif req.type == "cardboard":
            grab.grab_box(req.L, req.R, box_h_m=req.box_h,
                          top_center_cam=req.top_center)
        else:
            print(f"[GRAB] 알 수 없는 type: {req.type}")
    except Exception:
        import traceback
        print("[GRAB] 예외 발생:")
        traceback.print_exc()
    finally:
        with grab_lock:
            grab_busy = False
        print("[GRAB] 완료")


@app.post("/grab_at", summary="인식 파일이 좌표 주면 잡기 실행")
async def grab_at(req: GrabRequest):
    global grab_busy
    # 모드 게이트
    if ACTIVE_MODE == "none":
        return JSONResponse({"ok": False, "reason": "mode is none"})
    if req.type == "marker" and ACTIVE_MODE != "marker":
        return JSONResponse({"ok": False, "reason": f"mode={ACTIVE_MODE}"})
    if req.type == "cardboard" and ACTIVE_MODE != "box":
        return JSONResponse({"ok": False, "reason": f"mode={ACTIVE_MODE}"})
    # 중복 방지
    with grab_lock:
        if grab_busy or is_running:
            return JSONResponse({"ok": False, "reason": "busy"})
        grab_busy = True
    threading.Thread(target=_run_grab, args=(req,), daemon=True).start()
    return JSONResponse({"ok": True, "type": req.type})


@app.get("/active_mode", summary="현재 잡기 모드 (인식 파일이 폴링)")
async def get_active_mode():
    return {"mode": ACTIVE_MODE, "busy": grab_busy, "is_running": is_running}


@app.post("/set_mode", summary="잡기 모드 전환 (none/marker/box)")
async def set_mode(mode: str):
    global ACTIVE_MODE
    if mode not in ("none", "marker", "box"):
        return JSONResponse({"ok": False, "error": f"invalid: {mode}"})
    prev = ACTIVE_MODE
    ACTIVE_MODE = mode
    print(f"[MODE] {prev} → {mode}")

    # 모드 전환 시 대기 자세 (잡기 중이 아닐 때만)
    if not grab_busy and not is_running and grab is not None:
        def _pose():
            if mode in ("marker", "box"):
                grab.ready()      # 팔 들어 대기
            else:
                grab.park()       # 팔 내림
        threading.Thread(target=_pose, daemon=True).start()

    return {"ok": True, "mode": ACTIVE_MODE}


@app.get("/grab_status")
async def grab_status():
    return {"mode": ACTIVE_MODE, "busy": grab_busy, "is_running": is_running}


@app.get("/set_wrist")
async def set_wrist(l_roll: float=0, l_pitch: float=0, l_yaw: float=0,
                    r_roll: float=0, r_pitch: float=0, r_yaw: float=0):
    grab.wrist_params = {
        'left':  {'roll': l_roll, 'pitch': l_pitch, 'yaw': l_yaw},
        'right': {'roll': r_roll, 'pitch': r_pitch, 'yaw': r_yaw},
    }
    print(f"[WRIST] {grab.wrist_params}")
    return {"success": True, "wrist_params": grab.wrist_params}


@app.get("/set_box_size")
async def set_box_size(width: float=None, depth: float=None, height: float=None):
    """marker 모드용 박스 크기 (box 모드는 측정값 사용)."""
    if width  is not None: grab.box_size["width"]  = float(width)
    if depth  is not None: grab.box_size["depth"]  = float(depth)
    if height is not None: grab.box_size["height"] = float(height)
    print(f"[BOX_SIZE] {grab.box_size}")
    return {"success": True, "box_size": grab.box_size}


@app.get("/box_size")
async def get_box_size():
    return {"box_size": grab.box_size}


@app.get("/set_handover_direction")
async def set_handover_direction(direction: str="center", yaw_deg: float=None):
    if direction not in ("center", "left", "right"):
        return JSONResponse({"success": False, "error": f"invalid: {direction}"})
    grab.handover_direction = direction
    if yaw_deg is not None:
        grab.handover_yaw_deg = float(yaw_deg)
    print(f"[HANDOVER] {direction}, yaw={grab.handover_yaw_deg}")
    return {"success": True, "direction": direction, "yaw_deg": grab.handover_yaw_deg}


@app.get("/grab_manual")
async def grab_manual():
    """수동 잡기: 현재 모드의 인식 파일 /pose를 GET해서 잡기."""
    import urllib.request
    if ACTIVE_MODE == "none":
        return JSONResponse({"ok": False, "reason": "mode is none"})
    url = ("http://localhost:50011/pose" if ACTIVE_MODE == "marker"
           else "http://localhost:50010/pose")
    try:
        raw = urllib.request.urlopen(url, timeout=1.0).read()
        d = json.loads(raw)
    except Exception as e:
        return JSONResponse({"ok": False, "reason": f"detect fetch 실패: {e}"})
    if not d.get("found"):
        return JSONResponse({"ok": False, "reason": "검출 없음"})
    req = GrabRequest(**{k: d.get(k) for k in
                         ("type","tvec","rvec","L","R","top_center","box_h")
                         if k in d})
    return await grab_at(req)


# ==========================================
# API: 상태 / 모션
# ==========================================
@app.get("/status")
async def status():
    return {"is_running": is_running, "arm_ready": arm is not None,
            "loco_ready": loco is not None, "hand_ready": hand is not None,
            "tts_ready": tts is not None, "active_mode": ACTIVE_MODE,
            "grab_busy": grab_busy}

@app.get("/motions")
async def list_motions():
    files = sorted([f.name for f in MOTIONS_DIR.glob("*.json")])
    return {"motions": files, "directory": str(MOTIONS_DIR)}

@app.post("/motions/run/{filename}")
async def run_motion_by_name(filename: str):
    if is_running or grab_busy:
        raise HTTPException(409, "동작 중")
    filepath = MOTIONS_DIR / filename
    if not filepath.exists():
        raise HTTPException(404, f"파일 없음: {filename}")
    with open(filepath, "r", encoding="utf-8") as f:
        data = json.load(f)
    if not data:
        raise HTTPException(400, "빈 모션")
    first = data[0]
    is_ik = "left_xyz" in first or "right_xyz" in first
    if is_ik:
        asyncio.create_task(_execute_ik_frames([IKMotionFrame(**f) for f in data]))
    else:
        asyncio.create_task(_execute_frames([MotionFrame(**f) for f in data]))
    return {"status": "started", "frames": len(data), "format": "ik" if is_ik else "joint"}

@app.post("/send_gift")
async def send_gift():
    if is_running or grab_busy:
        raise HTTPException(409, "동작 중")
    if not (MOTIONS_DIR / "right_send.json").exists():
        raise HTTPException(404, "right_send.json 없음")
    asyncio.create_task(_execute_send_gift())
    return {"status": "started"}

@app.post("/run")
async def run_motion(frames: List[MotionFrame]):
    if is_running or grab_busy: raise HTTPException(409, "동작 중")
    if not frames: raise HTTPException(400, "빈 모션")
    asyncio.create_task(_execute_frames(frames))
    return {"status": "started", "frames": len(frames)}

@app.post("/run_ik")
async def run_ik_motion(frames: List[IKMotionFrame]):
    if is_running or grab_busy: raise HTTPException(409, "동작 중")
    if not frames: raise HTTPException(400, "빈 모션")
    asyncio.create_task(_execute_ik_frames(frames))
    return {"status": "started", "frames": len(frames)}

@app.post("/stop")
async def stop_motion():
    global STOP_FLAG
    STOP_FLAG = True
    if loco: loco.stop()
    if arm:
        loop = asyncio.get_running_loop()
        await asyncio.gather(
            loop.run_in_executor(None, arm.move_joints_smooth, [0]*14, 1.0),
            loop.run_in_executor(None, arm.move_waist_smooth, 0.0, 0.0, WAIST_BASE_PITCH, 1.0))
    return {"status": "stopped"}

@app.post("/home")
async def go_home():
    global STOP_FLAG
    STOP_FLAG = True
    await asyncio.sleep(0.1)
    if arm:
        loop = asyncio.get_running_loop()
        await asyncio.gather(
            loop.run_in_executor(None, arm.move_joints_smooth, [0]*14, 2.0),
            loop.run_in_executor(None, arm.move_waist_smooth, 0.0, 0.0, WAIST_BASE_PITCH, 2.0))
    return {"status": "home"}

@app.post("/loco/move")
async def loco_move(req: LocoMoveRequest):
    if not loco: raise HTTPException(503, "Loco 미초기화")
    loco.move(req.vx, req.vy, req.vyaw)
    return {"ok": True}

@app.post("/loco/stop")
async def loco_stop_endpoint():
    if loco: loco.stop()
    return {"ok": True}


# ==========================================
# 웹 UI — robot_web.html 읽어 viewer 코드 삽입
# ==========================================
WEB_HTML_PATH = os.path.join(current_dir, "robot_web.html")

@app.get("/", include_in_schema=False)
async def index():
    return HTMLResponse(open(WEB_HTML_PATH, encoding="utf-8").read())


if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=50000, timeout_graceful_shutdown=2)
