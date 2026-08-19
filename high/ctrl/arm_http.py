"""
arm_http.py — arm_server(50022) HTTP 클라이언트

robot_server 가 기존에 쓰던 ArmControllerWrapper 인터페이스를 최대한 그대로
흉내 내는 어댑터. GrabController / 모션 러너 코드 수정을 최소화한다.

제공 (기존 호출부 호환):
  move_hands(l_xyz, r_xyz, l_rot, r_rot, duration, frequency)
  move_waist_smooth(yaw, roll, pitch, duration)   # deg
  move_joints_smooth(deg14, duration)
  stop_motion()
  hold(duration) / release(duration, arm_deg)
  arm_ctrl  — 조회용 심(shim):
      get_current_dual_arm_q() -> rad 14
      get_waist_q()            -> rad 3
      get_weight()             -> float
      q_target / waist_q_target (rad, /pose 타겟 스냅샷)
      ctrl_lock (더미 락 — HTTP 라 원자성은 서버가 보장)
      motion_mode = True

동작 엔드포인트는 arm_server 가 완료까지 블로킹이므로, 이 클라이언트의
호출도 기존과 같이 동작이 끝나야 리턴한다.
"""

import json
import threading
import urllib.request
import numpy as np

ARM_SERVER = "http://localhost:50022"


def _post(path, body=None, timeout=60.0):
    data = json.dumps(body or {}).encode()
    req = urllib.request.Request(ARM_SERVER + path, data=data,
                                 headers={"Content-Type": "application/json"},
                                 method="POST")
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return json.loads(r.read())


def _get(path, timeout=2.0):
    with urllib.request.urlopen(ARM_SERVER + path, timeout=timeout) as r:
        return json.loads(r.read())


class _CtrlShim:
    """arm.arm_ctrl 조회부 호환. 쓰기(직접 대입)는 지원하지 않는다."""

    def __init__(self):
        self.ctrl_lock = threading.Lock()   # 호환용 더미
        self.motion_mode = True

    # ---- 조회 ----
    def get_current_dual_arm_q(self):
        return np.radians(np.asarray(_get("/pose")["arm_deg"], dtype=float))

    def get_waist_q(self):
        return np.radians(np.asarray(_get("/pose")["waist_deg"], dtype=float))

    def get_weight(self):
        return float(_get("/status").get("weight", 0.0))

    @property
    def q_target(self):
        return np.radians(np.asarray(_get("/pose")["arm_target_deg"], dtype=float))

    @property
    def waist_q_target(self):
        return np.radians(np.asarray(_get("/pose")["waist_target_deg"], dtype=float))


class ArmHttpClient:
    """ArmControllerWrapper 대체. 실패 시 예외를 그대로 올린다."""

    def __init__(self):
        self.arm_ctrl = _CtrlShim()
        # 기동 확인
        st = _get("/status")
        if not st.get("ready"):
            raise RuntimeError("arm_server 미준비")
        print(f"[arm_http] 연결됨 (mode={st.get('mode')}, weight={st.get('weight')})")

    # ---- 동작 (완료까지 블로킹) ----
    def move_hands(self, left_xyz, right_xyz, left_rot=None, right_rot=None,
                   duration=2.0, frequency=100):
        body = {"left_xyz": list(map(float, left_xyz)),
                "right_xyz": list(map(float, right_xyz)),
                "duration": float(duration), "frequency": int(frequency)}
        if left_rot is not None:
            body["left_quat"] = [float(left_rot.w), float(left_rot.x),
                                 float(left_rot.y), float(left_rot.z)]
        if right_rot is not None:
            body["right_quat"] = [float(right_rot.w), float(right_rot.x),
                                  float(right_rot.y), float(right_rot.z)]
        _post("/hands", body, timeout=duration + 30.0)

    def move_waist_smooth(self, yaw=0.0, roll=0.0, pitch=0.0, duration=1.5):
        _post("/waist", {"yaw": float(yaw), "roll": float(roll),
                         "pitch": float(pitch), "duration": float(duration)},
              timeout=duration + 15.0)

    def move_joints_smooth(self, deg14, duration=2.0):
        _post("/joints", {"deg": [float(v) for v in deg14],
                          "duration": float(duration)},
              timeout=duration + 15.0)

    def stop_motion(self):
        _post("/stop_motion", timeout=3.0)

    # ---- 제어권 ----
    def hold(self, duration=2.0):
        return _post("/hold", {"duration": float(duration)},
                     timeout=duration + 15.0)

    def release(self, duration=2.0, arm_deg=None):
        body = {"duration": float(duration)}
        if arm_deg:
            body["arm_deg"] = [float(v) for v in arm_deg]
        return _post("/release", body, timeout=duration * 2 + 20.0)

    def freeze(self):
        _post("/freeze", timeout=3.0)

    def park(self, duration=2.0, arm_deg=None):
        body = {"duration": float(duration)}
        if arm_deg:
            body["arm_deg"] = [float(v) for v in arm_deg]
        _post("/park", body, timeout=duration * 2 + 20.0)

    def status(self):
        return _get("/status")

    def start(self):
        pass   # 호환용 (arm_server 가 이미 기동)

