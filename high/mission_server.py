"""
mission_server.py — G1 미션 서버 (포트 50030)
Version: 0.1

시나리오 (한 번의 /grab 으로 전 과정 자동):
  대기(arm release) → 시작 신호 → arm hold 전환 → 팔 대기 자세(READY)
  → 박스 정렬 보행 (파지 가능 구간까지 펄스 이동 — approach_mode: box/none)
  → 박스 인식 → 파지 → 들기
  → 뒤로 한 걸음 → 정지 (hold 유지, 박스 든 채 완료)

시작 방법:
  - 웹: http://<robot>:50030/  의 GRAB 버튼
  - API: POST /grab   (SLAM/marker_nav 등이 접근 완료 후 호출)
    ※ 호출 전 외부 이동 명령 송신을 완전히 중단할 것

사용하는 기존 서버 (미리 떠 있어야 함):
  rs_stream(50001) · detect_box(50010) · arm_server(50022)
  (마커 접근은 marker_nav(50050) 가 담당 — 이 서버는 박스 정렬·파지·놓기만)
  → start_robot.sh 스택 위에 함께 띄우거나, 위 4개만 띄워도 동작

API:
  POST /grab    잡기 (진행 중이면 409)
  POST /place   놓기 — 한걸음 전진 + 내려놓기 + 후진 + release
  POST /stop    중단 — loco 정지 + 팔 동결 (release 하지 않음: 박스 들었을 수 있음)
  POST /reset   대기 상태 복귀 — release + 기본자세 (박스 없는 것 확인 후 호출)
  GET  /status  {state, step, detail, t_run}
  POST /params  파라미터 변경

상태(state): idle → holding → approach → grab → step_back → done
             (실패 시 error, 중단 시 stopped)
"""

import os
import sys
import json
import time
import threading
import urllib.request
import numpy as np

import uvicorn
from fastapi import FastAPI
from fastapi.responses import HTMLResponse, JSONResponse
from fastapi.middleware.cors import CORSMiddleware
from contextlib import asynccontextmanager

current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_dir)

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from ctrl.arm_controller_wrapper import LocoClientWrapper

PORT = 50030
ARM = "http://localhost:50022"
BOX_POSE = "http://localhost:50010/pose"

# ==========================================
# 파라미터
# ==========================================
P = {
    # 접근 방식
    #   "box"    : 박스를 보고 파지 가능 위치까지 미세 정렬 보행 (기본, SLAM 전제)
    #   "marker" : 마커 정면 경유 접근 (원거리 유도용 — 마커 배치가 있을 때)
    #   "none"   : 정렬 없이 현재 자리에서 바로 파지 (SLAM 정밀도가 충분할 때)
    "approach_mode": "box",

    # --- 박스 정렬 보행 ---
    # 파지 가능 "구간" — 점이 아니라 범위로 판정한다.
    #   보행 분해능이 20cm 수준(최소 속도 0.22 × 최소 한 걸음)이라
    #   몇 cm 짜리 목표점을 맞추는 것은 원리적으로 불가능하다.
    #   구간 안에 들어오면 통과시키고, 남은 오차는 팔 IK 가 흡수한다.
    "reach_x_min": 0.33,    # 이보다 가까우면 박스가 화면 아래로 빠짐
    "reach_x_max": 0.45,    # 이보다 멀면 팔이 닿지 않음.
                            #   URDF 계산: 파지 높이 z≈+0.015 에서 왼손 최대 도달
                            #   x≈0.354. 손 목표는 cx-0.15 이므로 cx 0.45 → 손 0.30
                            #   (여유 5cm). cx 0.47 이면 손 0.32 로 여유 3cm — 팔이
                            #   거의 펴져 그립 힘이 안 나온다 (실기 확인)
    "reach_y_abs": 0.10,    # 좌우 — 남은 어긋남은 허리 yaw 가 흡수
    "grab_x": 0.42,         # 구간 밖일 때 조준점 — 창 중앙(0.39)이 아니라 위쪽.
                            #   전진 한 걸음이 11cm 라 중앙을 노리면 검출 하한
                            #   (0.35) 아래로 넘어가기 쉽다. 위쪽을 노려 착지를
                            #   0.36~0.44 (보이는 구간) 에 떨어뜨린다.
    "lost_back_max": 3,     # 정렬 중 박스 소실 시 후진 복구 시도 횟수.
                            #   가까이서 소실 = 대부분 박스가 화면 아래로 빠진 것
                            #   → 뒤로 한 걸음 물러나면 다시 보인다
    "back_vy_ff": 0.06,     # 후진 시 좌우 편향 보정 (m/s, +면 왼쪽).
                            #   이 로봇은 vy=0 명령에도 후진하면 우측 뒤로
                            #   대각선으로 흐른다 — 그만큼 왼쪽을 섞어 상쇄한다.
                            #   반대로 흐르면 부호를 뒤집을 것
    "lost_back_step": 0.16, # 후진 복구 한 번의 이동량 (m). 11cm 로는 지나친 양을
                            #   되돌리기 모자랐다
    "lost_back_below": 0.60,# 직전 관측이 이 거리 안이었으면 소실 시 후진 복구.
                            #   기준은 "걷기 전 마지막 측정값" — 지나침은 보통
                            #   0.47~0.55 에서 측정→전진→소실 순서라, 0.45 로 걸면
                            #   거의 발동하지 않는다 (실기에서 확인)

    # 보행 속도.
    #   Unitree 학습 코드(unitree_rl_gym legged_robot.py)는 선속도 명령의 크기가
    #   0.2 미만이면 0 으로 만든다 — 즉 정책이 "그 이하는 제자리" 로 학습됐다.
    #   그래서 0.2 미만 명령으로는 아무리 오래 줘도 걷지 않는다.
    "align_vmin_cmd": 0.12,  # 명령 하한 (실기: 0.11 까지 걷는 것 확인)
    #   실기에서 0.11 까지 걷는 것 확인 — 0.12 로 운용 (여유 0.01).
    #   한 펄스 이동량 0.12×0.9s ≈ 11cm.
    "align_vx_max": 0.12,
    "align_vx_min": 0.12,
    "align_vy_max": 0.18,   # 게걸음 — 0.12 는 제자리걸음. 0.18×0.9s ≈ 16cm
    "align_vy_min": 0.18,
    "align_vyaw_min": 0.20,
    # 정렬 중 회전 사용 여부.
    #   False = 게걸음만 (기본) — 같은 좌우 오차를 게걸음·회전이 동시에 잡으면
    #           서로 싸워 경로가 휜다. SLAM 이 대략 정면을 맞춰준 전제.
    #   True  = 회전도 함께 사용
    "align_use_yaw": False,
    # 정렬 방식: 펄스(한 걸음 → 정지 → 측정) 반복.
    #   연속 서보는 불가능하다 — 최소 보행 속도(0.12m/s)와 걸음 단위(10~20cm)
    #   때문에 몇 cm 짜리 허용창을 지나쳐 버리고, 실제 정지 위치가 목표와
    #   무관해진다. 짧게 움직이고 멈춰서 다시 재는 방식이라야 수렴한다.
    # 펄스 길이는 최소 한 걸음이 나올 만큼 길어야 한다. G1 보행 주기가
    # 0.6~0.8s 라 그보다 짧으면 체중만 옮기다 멈춰 제자리가 된다.
    "pulse_min": 0.9,       # 최소 펄스 시간 (s) — 한 걸음 보장
    "pulse_max_t": 2.0,     # 최대 펄스 시간 (s)
    "box_min_samples": 15,  # 버퍼를 비운 뒤 이만큼 쌓이면 읽는다 (10Hz → 1.5초).
                            #   파지 정렬은 속도보다 정확도가 중요하다 —
                            #   샘플이 많을수록 중앙값이 안정된다
    "pulse_wait": 3.0,      # 샘플이 box_min_samples 만큼 쌓이기를 기다리는 상한 (s)
    "pulse_max": 20,        # 최대 반복 횟수
    "align_kp_vy": 0.6,
    "align_kp_yaw": 0.6,
    "align_vyaw_max": 0.30,
    "align_ema": 0.35,
    "align_timeout": 40.0,
    "align_lost_stop": 1.5,
    "align_far_max": 1.5,   # 이보다 멀면 SLAM 영역 — 실패 처리 (m)
    # 팔을 언제 들어올릴 것인가 (READY)
    #   True  = 접근 전에 미리 올린다 (기본, robot_server 와 같은 순서)
    #   False = 정렬이 끝나고 정지한 뒤에 올린다
    #           → 걷는 동안 손이 카메라 시야를 가리는 경우에만 사용
    "ready_before_approach": True,
    "settle_time": 2.0,     # 정지 후 안정화 대기 (s)
                            #   · detect_box 는 1초 윈도우 median 이라 정지 직후에는
                            #     걷는 동안의 좌표가 섞여 있다 → 윈도우가 비워질 시간
                            #   · loco.stop() 후 다리가 실제로 멎는 시간도 포함

    "yaw_sign": 1,          # 게걸음 부호 (do_align_box)

    # 파지 (robot_server GrabController 와 동일 상수)
    "home_x": 0.15,         # 대기(READY) 자세 — 몸 가까이 들어올린 위치
    "home_y": 0.25,
    "home_z": 0.20,
    "waist_base_pitch": -3.0,
    "grab_x_offset": -0.15,
    "lift_up": 0.20,        # 파지 후 들어올리는 높이 (박스 윗면 기준, m)
                            #   0.15 → 0.20: 든 박스가 정면 카메라의 마커 시야
                            #   위쪽을 살짝 가려서 5cm 더 올림
    "grab_z_offset": 0.08,
    "approach_extra": 0.10,
    "box_detect_timeout": 8.0,
    # 박스 타당성 검사 — median 은 "일관되게 틀린" 오검출도 안정화하므로
    # 좌표가 물리적으로 말이 되는지 별도로 본다 (torso 기준, m)
    "box_w_min": 0.10,      # L-R 폭 최소 (이보다 좁으면 오검출)
    "box_w_max": 0.60,      # L-R 폭 최대 (팔 벌림 한계)
    "box_x_min": 0.15,      # 전방 거리 최소 (몸에 너무 붙음)
    "box_x_max": 0.55,      # 전방 거리 최대 (IK 도달 한계)
    "box_z_min": -0.35,     # 높이 최소
    "box_z_max": 0.45,      # 높이 최대
    "box_h_max": 0.40,      # 박스 높이 최대
    "redetect_tol": 0.08,   # 허리 정렬 전후 중심 좌표 불일치 허용 (초과 시 중단)
    # 후진 한 걸음
    "back_vx": 0.15,        # 후진 속도 (m/s)
    "back_time": 1.5,       # 후진 시간 (s) → 약 0.2m
    # 내려놓기 (place)
    "place_x": 0.32,        # 놓는 위치 전방 거리 (torso x, m)
    "tuck_x": 0.15,         # 놓은 뒤 팔을 당겨오는 x (몸 가까이) — 높이는 유지.
                            #   앞으로 뻗은 채 후진하면 균형을 잃는다
    "place_fwd_t": 0.9,     # 놓기 직전 전진 시간 (s) — 한 걸음(≈11cm) 들어가서 놓는다.
                            #   접근(marker_nav)은 잡기와 같은 자리에 서므로, 놓을 때
                            #   박스가 테이블 위에 확실히 얹히려면 이만큼 더 붙어야
                            #   한다. 0 이면 끔. 마커가 필요 없는 맹목 전진이라
                            #   박스가 카메라를 가려도 무관.
    "place_drop": 0.10,     # 들던 높이에서 내리는 양 (m).
                            #   손을 놓는 순간 박스가 조금 떨어지지만, 깊게 내리다
                            #   테이블을 찍는 것보다 낫다
    "open_extra": 0.10,     # 놓을 때 양손 벌림 (m)
}

# ==========================================
# 카메라 → torso (robot_server 와 동일 상수)
# ==========================================
CAMERA_X, CAMERA_Y, CAMERA_Z = 0.0576235, 0.03003, 0.42987
CAMERA_PITCH = 0.8307767239493009   # 47.6도


def camera_to_torso(cx, cy, cz):
    cp, sp = np.cos(CAMERA_PITCH), np.sin(CAMERA_PITCH)
    cy_r = cy * cp + cz * sp
    cz_r = -cy * sp + cz * cp
    return float(cz_r + CAMERA_X), float(-cx + CAMERA_Y), float(-cy_r + CAMERA_Z)


def camera_dir_to_torso(dx, dy, dz):
    cp, sp = np.cos(CAMERA_PITCH), np.sin(CAMERA_PITCH)
    dy_r = dy * cp + dz * sp
    dz_r = -dy * sp + dz * cp
    return float(dz_r), float(-dx), float(-dy_r)


# ==========================================
# 외부 서버 헬퍼
# ==========================================
def _get(url, timeout=1.0):
    with urllib.request.urlopen(url, timeout=timeout) as r:
        return json.loads(r.read())


def _post(url, body=None, timeout=60.0):
    req = urllib.request.Request(url, data=json.dumps(body or {}).encode(),
                                 headers={"Content-Type": "application/json"},
                                 method="POST")
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return json.loads(r.read())


def arm_hold():
    return _post(ARM + "/hold", {"duration": 2.0}, timeout=20)


def arm_release():
    return _post(ARM + "/release", {"duration": 2.0}, timeout=30)


def arm_freeze():
    return _post(ARM + "/freeze", timeout=5)


def arm_waist(yaw=0.0, roll=0.0, pitch=None, duration=1.5):
    if pitch is None:
        pitch = P["waist_base_pitch"]
    return _post(ARM + "/waist", {"yaw": yaw, "roll": roll, "pitch": pitch,
                                  "duration": duration}, timeout=duration + 15)


def arm_ready(duration=2.0):
    """대기(READY) 자세 — 팔을 몸 가까이 들어올린다.

    move_hands 는 직교좌표 직선 보간이라, 팔이 몸 옆 아래에 있는 상태에서
    곧바로 박스 위 지점으로 보내면 그 직선이 테이블 모서리·밑면을 통과한다.
    파지 전에 반드시 이 자세를 거칠 것 (robot_server GrabController.ready 와 동일).
    """
    hx, hy, hz = P["home_x"], P["home_y"], P["home_z"]
    return arm_hands([hx, +hy, hz], [hx, -hy, hz], duration)


def arm_hands(left_xyz, right_xyz, duration=2.0):
    return _post(ARM + "/hands", {"left_xyz": left_xyz, "right_xyz": right_xyz,
                                  "duration": duration, "frequency": 100},
                 timeout=duration + 30)


_BOX_RESET_OK = None      # None=미확인, True=지원, False=구버전


def _reset_box_window():
    """detect_box 안정화 버퍼를 비운다 → 지원 여부(bool).

    구버전 detect_box 에는 이 엔드포인트가 없다. 한 번 확인해 두고
    없으면 다시 시도하지 않는다(매 측정마다 404 왕복 낭비 방지).
    """
    global _BOX_RESET_OK
    if _BOX_RESET_OK is False:
        return False
    try:
        req = urllib.request.Request(BOX_POSE.replace("/pose", "/reset_window"),
                                     data=b"", method="POST")
        with urllib.request.urlopen(req, timeout=1.0):
            pass
        _BOX_RESET_OK = True
        return True
    except Exception:
        if _BOX_RESET_OK is None:
            print("[mission] detect_box /reset_window 없음 — 예전 방식으로 측정")
        _BOX_RESET_OK = False
        return False


def box_pose():
    try:
        d = _get(BOX_POSE, timeout=1.0)
        return d if d.get("found") else None
    except Exception:
        return None


def box_geometry(d):
    """/pose 응답 → torso 좌표 (Lx..Rz, cx,cy,cz, h). 없으면 None."""
    if not (d and d.get("L") and d.get("R")):
        return None
    Lx, Ly, Lz = camera_to_torso(*d["L"])
    Rx, Ry, Rz = camera_to_torso(*d["R"])
    if d.get("top_center"):
        cx, cy, cz = camera_to_torso(*d["top_center"])
    else:
        cx, cy, cz = (Lx + Rx) / 2, (Ly + Ry) / 2, (Lz + Rz) / 2
    h = d.get("box_h") or 0.065
    return dict(L=(Lx, Ly, Lz), R=(Rx, Ry, Rz), c=(cx, cy, cz), h=float(h))


def box_valid(g):
    """박스 좌표 타당성 검사 → (ok, 사유).

    detect_box 는 1초 median 으로 이미 시간 안정화를 하지만, 엉뚱한 물체를
    박스로 오검출하면 그 틀린 값도 똑같이 안정적으로 나온다. 팔을 보내기 전에
    좌표가 물리적으로 말이 되는지 확인한다.
    """
    if g is None:
        return False, "좌표 없음"
    (Lx, Ly, Lz), (Rx, Ry, Rz), (cx, cy, cz) = g["L"], g["R"], g["c"]
    w = ((Lx - Rx) ** 2 + (Ly - Ry) ** 2) ** 0.5
    if not (P["box_w_min"] <= w <= P["box_w_max"]):
        return False, f"폭 이상 {w:.2f}m"
    if not (P["box_x_min"] <= cx <= P["box_x_max"]):
        return False, f"거리 이상 {cx:.2f}m"
    if not (P["box_z_min"] <= cz <= P["box_z_max"]):
        return False, f"높이 이상 {cz:.2f}m"
    if not (0.01 <= g["h"] <= P["box_h_max"]):
        return False, f"박스높이 이상 {g['h']:.2f}m"
    if Ly < Ry:      # 왼쪽 그립점이 오른쪽보다 오른쪽에 있으면 L/R 뒤바뀜
        return False, "L/R 좌우 반전"
    return True, "ok"


# ==========================================
# 미션 상태
# ==========================================
S = {"state": "idle", "step": "-", "detail": "", "t_run": 0.0}
_run = threading.Event()
loco = None
LAST_GRAB = None        # 마지막 파지 형상 {"oL","oR","lift_z","grab_z"} — place 에서 사용


def _set(state=None, step=None, detail=None):
    if state is not None:
        S["state"] = state
    if step is not None:
        S["step"] = step
    if detail is not None:
        S["detail"] = detail
    print(f"[mission] {S['state']} / {S['step']} {S['detail']}")


def _aborted():
    return not _run.is_set()


# ==========================================
# 1단계-A: 박스 정렬 보행 (기본)
#   SLAM 이 1m 안쪽까지 데려다 놓은 상태에서, 박스를 보며
#   파지 가능한 위치(전방 grab_x, 좌우 중앙)까지 미세하게 걸어 맞춘다.
# ==========================================
def do_align_box():
    """박스 정렬 — 펄스 방식 (짧게 움직이고 멈춰서 다시 잰다).

    연속 서보를 쓰지 않는 이유: 보행은 걸음 단위(10~20cm)로 일어나고 최소
    속도 아래로는 아예 걷지 않는다. 그래서 수 cm 짜리 허용창을 향해 연속
    제어하면 창을 지나쳐 버리고, 실제 정지 위치가 목표와 무관해진다.
    """
    _set(state="approach", step="align", detail="박스 정렬")
    t0 = time.time()

    def measure(settle):
        """멈춘 상태에서 좌표를 잰다 → (g, why).

        detect_box 는 SMOOTH_WINDOW_SEC(2초) 슬라이딩 윈도우 중앙값을 내보낸다.
        그냥 기다렸다 읽으면 걷는 동안의 좌표가 섞여, 눈으로는 제자리인데
        "아직 멀다"고 읽히는 일이 생긴다. 그래서 정지 직후 버퍼를 비우고
        (POST /reset_window), 새 샘플이 충분히 쌓인 뒤에 읽는다.
        reset_window 가 없는 구버전 detect_box 면 조용히 예전 방식으로 돈다.
        """
        loco.stop()
        time.sleep(0.8)                     # 다리가 완전히 멎고 몸 흔들림이 잦아들 시간
        fresh = _reset_box_window()
        t_s = time.time()
        if fresh:
            # 새 샘플이 box_min_samples 개 쌓일 때까지 (최대 settle 초)
            while time.time() - t_s < settle:
                if _aborted():
                    return None, "중단됨"
                d = box_pose()
                if d and d.get("n", 0) >= P["box_min_samples"]:
                    break
                time.sleep(0.05)
        else:
            while time.time() - t_s < settle:
                if _aborted():
                    return None, "중단됨"
                time.sleep(0.05)
        g = box_geometry(box_pose())
        if g is None:
            return None, "검출 없음"
        Lx, Ly, _lz = g["L"]
        Rx, Ry, _rz = g["R"]
        w = ((Lx - Rx) ** 2 + (Ly - Ry) ** 2) ** 0.5
        if not (P["box_w_min"] <= w <= P["box_w_max"]):
            return None, f"폭 이상 {w:.2f}m"
        if not (0.01 <= g["h"] <= P["box_h_max"]):
            return None, f"박스높이 이상 {g['h']:.2f}m"
        if Ly < Ry:
            return None, "L/R 좌우 반전"
        return g, "ok"

    def pulse(vx, vy, vyaw, dist):
        """dist 만큼 가는 데 필요한 시간 동안 움직이고 멈춘다.

        단, 최소 pulse_min 은 보장한다 — 그보다 짧으면 걸음이 아예 시작되지
        않아 제자리가 된다. 그래서 한 걸음보다 작은 오차는 줄일 수 없고,
        허용 오차가 걸음 크기보다 커야 하는 이유가 된다.
        """
        spd = max(0.01, abs(vx) + abs(vy))
        dur = min(P["pulse_max_t"], max(P["pulse_min"], dist / spd))
        t_p = time.time()
        while time.time() - t_p < dur:
            if _aborted():
                loco.stop()
                return False
            loco.move(vx, vy, vyaw)
            time.sleep(0.05)
        loco.stop()
        return True

    miss = 0
    back_n = 0
    last_cx = None
    last_cy = None
    for n in range(P["pulse_max"]):
        if _aborted():
            loco.stop()
            return False, "중단됨"
        if time.time() - t0 > P["align_timeout"]:
            loco.stop()
            return False, "정렬 시간 초과"

        g, why = measure(P["pulse_wait"])
        if g is None:
            if why == "중단됨":
                return False, "중단됨"
            miss += 1
            S["detail"] = f"[{n+1}] 박스 확인 중 — {why}"
            # 측정 실패는 대부분 박스가 화면 아래로 빠진 것 — 완전 소실뿐
            # 아니라 반쯤 걸쳐 형상 검증(폭/높이)에 떨어지는 경우도,
            # 정렬 시작부터 이미 반쯤 나가 있어 한 번도 못 잰 경우(last_cx
            # None)도 같다. 실패 사유 불문 뒤로 한 걸음 물러나 다시 본다.
            if (back_n < P["lost_back_max"]
                    and (last_cx is None
                         or last_cx < P["lost_back_below"])):
                back_n += 1
                miss = 0
                # 후진은 우측 뒤로 흐르므로 vy 를 섞어 곧게 물러난다.
                # 소실 직전 박스가 한쪽에 있었으면 그쪽으로 더 실어 화면에 되돌린다.
                vy = P["back_vy_ff"]
                if last_cy is not None and abs(last_cy) > 0.05:
                    vy += 0.5 * last_cy
                vy = max(-0.12, min(0.12, vy))
                S["detail"] += (f" → 후진 복구 {back_n}/{P['lost_back_max']}"
                                f" (vy {vy:+.2f})")
                if not pulse(-abs(P["align_vx_max"]), vy, 0.0,
                             P["lost_back_step"]):
                    return False, "중단됨"
                continue
            if miss >= 3:
                loco.stop()
                return False, f"박스 소실 ({why})"
            continue
        miss = 0
        back_n = 0

        cx, cy, _cz = g["c"]
        last_cx, last_cy = cx, cy
        if cx > P["align_far_max"]:
            loco.stop()
            return False, f"박스가 너무 멂 {cx:.2f}m — SLAM 재접근 필요"

        # 구간 판정 — 잡을 수 있으면 그대로 통과
        ok_x = P["reach_x_min"] <= cx <= P["reach_x_max"]
        ok_y = abs(cy) <= P["reach_y_abs"]
        err_x = cx - P["grab_x"]      # 구간 밖일 때 중앙을 조준
        err_y = cy
        S["detail"] = f"[{n+1}] x={cx:.3f}(목표{P['grab_x']:.2f}) y={cy:+.3f}"

        if ok_x and ok_y:
            loco.stop()
            return True, f"정렬 완료 x={cx:.3f} y={cy:+.3f}"

        # 큰 오차부터 하나씩 잡는다 (동시에 주면 서로 간섭한다)
        vx = vy = vyaw = 0.0
        if not ok_y and abs(err_y) >= abs(err_x):
            mag = min(P["align_vy_max"], max(P["align_vy_min"], abs(err_y)))
            vy = P["yaw_sign"] * (mag if err_y > 0 else -mag)
            S["detail"] += f" → 좌우 {vy:+.2f}"
        elif not ok_x:
            mag = min(P["align_vx_max"], max(P["align_vx_min"], abs(err_x)))
            vx = mag if err_x > 0 else -mag
            S["detail"] += f" → 전후 {vx:+.2f}"
        elif not ok_y:
            mag = min(P["align_vy_max"], max(P["align_vy_min"], abs(err_y)))
            vy = P["yaw_sign"] * (mag if err_y > 0 else -mag)
            S["detail"] += f" → 좌우 {vy:+.2f}"

        # 학습 문턱: 선속도 명령의 크기가 0.2 미만이면 정책이 제자리로 해석한다
        spd = (vx * vx + vy * vy) ** 0.5
        if 0 < spd < P["align_vmin_cmd"]:
            k = P["align_vmin_cmd"] / spd
            vx, vy = vx * k, vy * k

        need = abs(err_y) if (vy != 0.0) else abs(err_x)
        if not pulse(vx, vy, vyaw, need):
            return False, "중단됨"

    loco.stop()
    return False, f"정렬 미수렴 ({P['pulse_max']}회 시도)"


# ==========================================
# 2단계: 박스 파지 (GrabController.grab_box 축약 — 들기까지만)
# ==========================================
def do_grab():
    _set(state="grab", step="detect", detail="박스 인식 대기")
    t0 = time.time()
    g = None
    why = "인식 없음"
    while time.time() - t0 < P["box_detect_timeout"]:
        if _aborted():
            return False, "중단됨"
        cand = box_geometry(box_pose())
        ok, why = box_valid(cand)
        if ok:
            g = cand
            break
        # 타당성 미달이면 바로 실패시키지 않고 계속 본다
        # (median 이 올바른 대상으로 수렴할 시간을 준다)
        S["detail"] = f"박스 확인 중 — {why}"
        time.sleep(0.3)
    if g is None:
        return False, f"박스 인식 실패 ({why})"

    (Lx, Ly, Lz), (Rx, Ry, Rz), (cx, cy, cz) = g["L"], g["R"], g["c"]

    # 허리 yaw 정렬 (필요 시) 후 재감지
    yaw_deg = float(np.degrees(np.arctan2(cy, cx)))
    if abs(yaw_deg) >= 1.5:
        _set(step="waist", detail=f"허리 yaw {yaw_deg:.1f}도")
        arm_waist(yaw=yaw_deg, duration=1.0 + abs(yaw_deg) / 30.0)
        time.sleep(0.6)
        g2 = box_geometry(box_pose())
        ok2, why2 = box_valid(g2)
        if ok2:
            # 허리만 돌았으므로 박스의 실제 위치는 그대로여야 한다.
            # 두 측정이 크게 어긋나면 둘 중 하나가 오검출 → 팔을 보내지 않는다.
            dx = g2["c"][0] - cx
            dy = g2["c"][1] - cy
            dz = g2["c"][2] - cz
            drift = (dx * dx + dy * dy + dz * dz) ** 0.5
            if drift > P["redetect_tol"]:
                return False, f"재감지 불일치 {drift:.2f}m — 오검출 의심"
            g = g2
            (Lx, Ly, Lz), (Rx, Ry, Rz), (cx, cy, cz) = g["L"], g["R"], g["c"]
        else:
            print(f"[mission] 재감지 실패({why2}) — 원래 좌표 사용")

    if _aborted():
        return False, "중단됨"

    h = g["h"]
    top_z = (Lz + Rz) / 2
    grab_z = top_z - h / 2 + P["grab_z_offset"]
    above_z = top_z + 0.10
    lift_z = top_z + P["lift_up"]

    def outward(px, py):
        dx, dy = px - cx, py - cy
        n = (dx * dx + dy * dy) ** 0.5
        return (dx / n, dy / n) if n > 1e-6 else (0.0, 0.0)
    oLx, oLy = outward(Lx, Ly)
    oRx, oRy = outward(Rx, Ry)
    ae = P["approach_extra"]
    gx = P["grab_x_offset"]

    _set(step="approach", detail="위쪽 접근")
    arm_hands([Lx + oLx * ae, Ly + oLy * ae, above_z],
              [Rx + oRx * ae, Ry + oRy * ae, above_z], 1.5)
    if _aborted(): return False, "중단됨"

    _set(step="descend", detail="측면 하강")
    arm_hands([Lx + oLx * ae, Ly + oLy * ae, grab_z],
              [Rx + oRx * ae, Ry + oRy * ae, grab_z], 1.0)
    if _aborted(): return False, "중단됨"

    _set(step="grip", detail="잡기")
    arm_hands([Lx + gx, Ly, grab_z], [Rx + gx, Ry, grab_z], 2.5)
    time.sleep(1.0)
    if _aborted(): return False, "중단됨"

    # 대칭 정렬 + 들기
    gxb = cx + gx
    oL, oR = abs(Ly - cy), abs(Ry - cy)
    _set(step="lift", detail="들기")
    arm_hands([gxb, +oL, grab_z], [gxb, -oR, grab_z], 1.5)
    time.sleep(0.3)
    arm_hands([gxb, +oL, lift_z], [gxb, -oR, lift_z], 1.5)
    time.sleep(0.3)
    global LAST_GRAB
    LAST_GRAB = {"oL": oL, "oR": oR, "lift_z": lift_z, "grab_z": grab_z}

    # 허리 yaw 0 복귀 (후진 보행 전 필수 — 비틀린 채 걸으면 균형 무너짐)
    arm_waist(yaw=0.0, duration=1.5 + abs(yaw_deg) / 30.0)
    time.sleep(0.3)
    return True, "파지 완료"


# ==========================================
# 3단계: 뒤로 한 걸음
# ==========================================
def do_step_back():
    _set(state="step_back", step="back", detail=f"{P['back_time']}초 후진")
    t0 = time.time()
    while time.time() - t0 < P["back_time"]:
        if _aborted():
            loco.stop()
            return False, "중단됨"
        loco.move(-abs(P["back_vx"]), 0, 0)
        time.sleep(0.05)
    loco.stop()
    time.sleep(0.3)
    loco.stop()
    return True, "후진 완료"


# ==========================================
# 내려놓기 (place) — 박스를 정면에 놓고 손을 뗀다. hold 유지.
# ==========================================
def do_place():
    g = LAST_GRAB or {"oL": 0.12, "oR": 0.12, "lift_z": 0.15, "grab_z": 0.05}
    px = P["place_x"]
    down_z = g["lift_z"] - P["place_drop"]
    oe = P["open_extra"]

    # (박스를 들고 있는 상태이므로 팔은 이미 lift_z 높이에 있다 — READY 불필요)
    _set(state="placing", step="extend", detail="앞으로 내밀기")
    arm_hands([px, +g["oL"], g["lift_z"]], [px, -g["oR"], g["lift_z"]], 1.5)
    if _aborted(): return False, "중단됨"

    _set(step="lower", detail="내려놓기")
    arm_hands([px, +g["oL"], down_z], [px, -g["oR"], down_z], 1.5)
    time.sleep(0.3)
    if _aborted(): return False, "중단됨"

    _set(step="open", detail="손 벌림")
    arm_hands([px, +g["oL"] + oe, down_z], [px, -g["oR"] - oe, down_z], 1.0)
    time.sleep(0.3)

    # 손을 벌린 채 위로 뺀 다음, 높이는 유지한 채 몸쪽으로 당긴다.
    # 팔을 앞으로 뻗은 채 후진하면 무게중심이 앞에 걸려 휘청인다(실기).
    # 손이 박스 옆면 바깥에 있으므로 수평으로 당기는 경로는 박스를 지나지 않는다.
    _set(step="retract", detail="손 위로 빼기")
    arm_hands([px, +g["oL"] + oe, g["lift_z"]], [px, -g["oR"] - oe, g["lift_z"]], 1.0)
    if _aborted(): return False, "중단됨"

    _set(step="tuck", detail="팔 몸쪽으로")
    arm_hands([P["tuck_x"], +0.25, g["lift_z"]],
              [P["tuck_x"], -0.25, g["lift_z"]], 1.2)
    time.sleep(0.2)
    return True, "내려놓기 완료"


def place_thread():
    t0 = time.time()
    try:
        try:
            st = _get(ARM + "/status", timeout=2)
            if st.get("mode") != "release":
                pass
            else:
                arm_hold()      # release 상태였으면 hold 전환 후 진행
        except Exception:
            pass
        # 놓기 직전 한 걸음 전진 — 잡기 위치 그대로 서 있으면 박스가
        # 테이블 가장자리에 걸치므로 들어가서 놓는다.
        if P["place_fwd_t"] > 0:
            _set(state="placing", step="fwd", detail="한 걸음 전진")
            t_f = time.time()
            while time.time() - t_f < P["place_fwd_t"]:
                if _aborted():
                    loco.stop()
                    _set(state="stopped", detail="중단됨")
                    return
                loco.move(abs(P["align_vx_max"]), 0.0, 0.0)
                time.sleep(0.05)
            loco.stop()
            time.sleep(0.5)

        ok, msg = do_place()
        if ok:
            global LAST_GRAB
            LAST_GRAB = None
            # 순서 중요: 놓기 → 후진 → 팔 정리.
            # 박스 위에서 팔을 접으면 접는 경로가 박스를 치므로,
            # 물러난 다음에 팔을 몸쪽으로 거둔다.
            ok2, msg2 = do_step_back()
            if not ok2:
                _set(state="stopped", detail=msg2)
                return
            _set(state="placing", step="release", detail="팔 release")
            try:
                arm_release()      # 후진까지 끝났으니 팔을 놓는다 (걷기 모드 복귀)
            except Exception as e:
                _set(state="error", detail=f"release 실패: {e}")
                return
            _set(state="placed", step="-",
                 detail="박스 내려놓음 → 후진 → release 완료")
        else:
            _set(state="stopped", detail=msg)
    finally:
        S["t_run"] = round(time.time() - t0, 1)
        _run.clear()


# ==========================================
# 미션 스레드
# ==========================================
def mission_thread():
    t0 = time.time()
    try:
        # 0) hold 전환
        _set(state="holding", step="hold", detail="arm hold 전환")
        try:
            arm_hold()
        except Exception as e:
            _set(state="error", detail=f"hold 실패: {e}")
            return
        arm_waist(yaw=0.0, duration=1.5)   # 허리 중립(pitch −3) 확보
        if P["ready_before_approach"]:
            _set(step="ready", detail="팔 대기 자세로 들어올림")
            arm_ready(2.0)
        if _aborted():
            _set(state="stopped"); return

        # 1) 접근 — 모드에 따라
        mode = P["approach_mode"]
        if mode == "none":
            ok, msg = True, "정렬 생략 (approach_mode=none)"
            _set(state="approach", step="skip", detail=msg)
        else:
            ok, msg = do_align_box()
        if not ok:
            loco.stop()
            _set(state="stopped" if msg == "중단됨" else "error", detail=msg)
            return

        # 1-1) 정지 안정화 — 걷는 동안의 좌표가 median 에서 빠질 때까지 기다린다
        if mode != "none":
            _set(step="settle", detail=f"정지 안정화 {P['settle_time']:.1f}s")
            loco.stop()
            t_s = time.time()
            while time.time() - t_s < P["settle_time"]:
                if _aborted():
                    _set(state="stopped"); return
                time.sleep(0.1)

        # 1-2) 팔 대기 자세 — 정렬이 끝난 뒤에 올린다.
        #      걷는 동안 손이 카메라 시야에 들어오면 박스 검출을 방해한다.
        #      move_hands 는 직교 직선 보간이므로, 몸 가까이(home_x) 들어올리는
        #      이 단계를 거쳐야 이후 박스 위로 갈 때 테이블을 훑지 않는다.
        if not P["ready_before_approach"]:
            _set(step="ready", detail="팔 대기 자세로 들어올림")
            arm_ready(2.0)
            if _aborted():
                _set(state="stopped"); return
            time.sleep(0.5)

        # 2) 파지
        ok, msg = do_grab()
        if not ok:
            loco.stop()
            _set(state="stopped" if msg == "중단됨" else "error", detail=msg)
            return

        # 3) 후진 한 걸음
        ok, msg = do_step_back()
        if not ok:
            _set(state="stopped", detail=msg)
            return

        _set(state="done", step="-", detail="완료 — hold 유지(박스 들고 있음)")
    finally:
        S["t_run"] = round(time.time() - t0, 1)
        _run.clear()
        try:
            loco.stop()
        except Exception:
            pass


# ==========================================
# FastAPI
# ==========================================
@asynccontextmanager
async def lifespan(app):
    global loco
    print("[mission_server] 시작")
    ChannelFactoryInitialize(0)
    loco = LocoClientWrapper()
    # 대기 상태 = release (팔 스윙 보행 가능 상태로 대기)
    try:
        st = _get(ARM + "/status", timeout=2)
        if st.get("mode") != "release":
            print("[mission_server] 대기 상태 확보 — arm release")
            arm_release()
    except Exception as e:
        print(f"[mission_server] ⚠ arm_server 확인 실패: {e}")
    print(f"[mission_server] 준비 완료 (idle/release)  http://localhost:{PORT}/")
    yield
    print("[mission_server] 종료")


app = FastAPI(title="G1 Mission Server", lifespan=lifespan)
app.add_middleware(CORSMiddleware, allow_origins=["*"],
                   allow_methods=["*"], allow_headers=["*"])


@app.get("/status")
async def status():
    return {**S, "running": _run.is_set(), "params": P}


@app.post("/grab")
async def grab():
    """잡기 (정렬→파지→들기→후진). SLAM 등 외부 소프트웨어는 이동 명령을
    완전히 중단한 뒤 호출."""
    if _run.is_set():
        return JSONResponse({"ok": False, "reason": "이미 진행 중"}, status_code=409)
    mode = P["approach_mode"]
    info = {}
    if mode != "none":
        g = box_geometry(box_pose())
        if g is None:
            return JSONResponse({"ok": False,
                                 "reason": "박스 미검출 — 위치/50010 확인"})
        cx, cy, _cz = g["c"]
        last_cx, last_cy = cx, cy
        if cx > P["align_far_max"]:
            return JSONResponse({"ok": False,
                                 "reason": f"박스가 {cx:.2f}m — 더 가까이 이동 후 시작"})
        info = {"box": {"cx": round(cx, 2), "cy": round(cy, 2)}}
    _run.set()
    S.update(state="starting", step="-", detail="", t_run=0.0)
    threading.Thread(target=mission_thread, daemon=True).start()
    return {"ok": True, "mode": mode, **info}


@app.post("/stop")
async def stop():
    """중단 — loco 정지 + 팔 동결. release 하지 않음 (박스 들었을 수 있음)."""
    _run.clear()
    try:
        loco.stop()
    except Exception:
        pass
    try:
        arm_freeze()
    except Exception:
        pass
    if S["state"] not in ("idle", "done"):
        S["state"] = "stopped"
    return {"ok": True}


@app.post("/place")
async def place():
    """박스를 정면(place_x)에 내려놓기. 파지 완료(done) 후 호출 권장.
    파지 기록이 없으면 기본 폭으로 동작하므로 빈손일 때 누르면 팔만 움직인다."""
    if _run.is_set():
        return JSONResponse({"ok": False, "reason": "진행 중"}, status_code=409)
    _run.set()
    threading.Thread(target=place_thread, daemon=True).start()
    return {"ok": True, "using_last_grab": LAST_GRAB is not None}


@app.post("/reset")
async def reset():
    """대기 상태 복귀 — release + 기본자세. 박스를 들고 있지 않을 때만 호출."""
    if _run.is_set():
        return JSONResponse({"ok": False, "reason": "진행 중 — /stop 먼저"},
                            status_code=409)
    try:
        arm_release()
    except Exception as e:
        return JSONResponse({"ok": False, "reason": f"release 실패: {e}"})
    S.update(state="idle", step="-", detail="", t_run=0.0)
    return {"ok": True}


@app.get("/active_mode")
async def active_mode():
    """detect_box 호환 엔드포인트.

    detect_box 의 검출 루프는 ROBOT_SERVER/active_mode 가 "box" 일 때만 돈다
    (아니면 CPU 절약을 위해 잠든다). 미션 스택에는 robot_server 가 없으므로
    mission_server 가 이 역할을 대신한다.
    → start_mission.sh 에서 ROBOT_SERVER=http://localhost:50030 으로 띄울 것.
    """
    return {"mode": "box", "busy": _run.is_set(), "is_running": _run.is_set()}


@app.post("/test_pulse")
async def test_pulse(body: dict):
    """이동량 측정 도구 — 한 번 펄스를 주고 박스 좌표가 얼마나 변했는지 잰다.

    보행은 걸음 단위로 일어나고 최소 명령 속도(0.2) 아래로는 아예 걷지 않는다.
    실제 한 펄스의 이동량이 얼마인지는 기기마다 다르므로 직접 재는 편이 빠르다.

    body: {"vx":0.25, "vy":0.0, "duration":0.9}
    """
    if _run.is_set():
        return JSONResponse({"ok": False, "reason": "미션 진행 중"}, status_code=409)
    vx = float(body.get("vx", 0.0))
    vy = float(body.get("vy", 0.0))
    dur = float(body.get("duration", 0.9))

    def snap():
        loco.stop()
        time.sleep(P["pulse_wait"])
        g = box_geometry(box_pose())
        return None if g is None else g["c"]

    before = snap()
    if before is None:
        return {"ok": False, "reason": "박스 미검출 — 측정 불가"}

    t0 = time.time()
    while time.time() - t0 < dur:
        loco.move(vx, vy, 0.0)
        time.sleep(0.05)
    loco.stop()

    after = snap()
    if after is None:
        return {"ok": False, "reason": "이동 후 박스 미검출",
                "before": [round(v, 3) for v in before]}
    return {"ok": True,
            "cmd": {"vx": vx, "vy": vy, "duration": dur},
            "before": [round(v, 3) for v in before],
            "after": [round(v, 3) for v in after],
            "moved_x_cm": round((before[0] - after[0]) * 100, 1),
            "moved_y_cm": round((before[1] - after[1]) * 100, 1)}


VALID_MODES = ("box", "none")


@app.post("/params")
async def params(body: dict):
    if "approach_mode" in body and body["approach_mode"] not in VALID_MODES:
        return JSONResponse({"ok": False,
                             "reason": f"approach_mode 는 {VALID_MODES} 중 하나"},
                            status_code=400)
    for k, v in body.items():
        if k in P:
            P[k] = type(P[k])(v)
    return {"ok": True, "params": P}


# ==========================================
# 웹 UI
# ==========================================
PAGE = """<!DOCTYPE html><html lang="ko"><head><meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>G1 Mission</title>
<style>
body{margin:0;background:#0e1116;color:#c9d4e0;font-family:ui-monospace,Menlo,monospace;
  min-height:100vh;display:flex;flex-direction:column;align-items:center;gap:18px;padding:28px}
h1{font-size:18px;color:#3ddc97;margin:0}
.state{font-size:15px;padding:10px 22px;border:1px solid #2a3340;border-radius:10px;background:#161b22}
.state b{color:#4aa8ff}
.state select{background:#0e1116;color:#c9d4e0;border:1px solid #2a3340;border-radius:6px;
  padding:4px 8px;font-family:inherit;font-size:13px}
button{font:inherit;border:none;border-radius:12px;cursor:pointer;padding:22px 0;width:280px;font-size:20px;font-weight:700}
#btn-start{background:#3ddc97;color:#05221a}
#btn-stop{background:#ff6b6b;color:#2a0505}
#btn-place{background:#4aa8ff;color:#04121f}
#btn-reset{background:#21262d;color:#c9d4e0;border:1px solid #2a3340;font-size:14px;padding:12px 0}
.feeds{display:flex;gap:12px;flex-wrap:wrap;justify-content:center}
.feeds img{width:320px;border:1px solid #2a3340;border-radius:8px;background:#05080c}
.hint{color:#6b7785;font-size:12px}
</style></head><body>
<h1>G1 Mission · 마커 접근 → 파지 → 후진</h1>
<div class="state">state: <b id="st">-</b> · <span id="step">-</span> · <span id="dt"></span></div>
<div class="state">접근 방식:
  <select id="mode" onchange="setMode()">
    <option value="box">박스 정렬 (기본)</option>
    <option value="none">정렬 생략</option>
  </select>
</div>
<button id="btn-start" onclick="post('/grab')">✊ GRAB (잡기)</button>
<button id="btn-stop" onclick="post('/stop')">■ STOP (동결)</button>
<button id="btn-place" onclick="post('/place')">🖐 PLACE (놓기)</button>
<button id="btn-reset" onclick="post('/reset')">↺ RESET (release·대기)</button>
<div class="feeds">
  <img id="f-box" alt="box">
  <img id="f-front" alt="front cam">
</div>
<div class="hint">box :50010 · arm :50022 — API: POST /grab /place /stop /reset /params · GET /status</div>
<script>
const host=location.hostname;
document.getElementById('f-box').src=`http://${host}:50010/video_feed`;
document.getElementById('f-front').src=`http://${host}:50050/video_feed`;
async function setMode(){const v=document.getElementById('mode').value;
  try{const r=await fetch('/params',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({approach_mode:v})});const d=await r.json();
    if(d.reason)alert(d.reason);}catch(e){alert(e.message);}}
async function post(p){try{const r=await fetch(p,{method:'POST'});const d=await r.json();
  if(d.reason)alert(d.reason);}catch(e){alert(e.message);}}
setInterval(async()=>{try{const d=await(await fetch('/status')).json();
  document.getElementById('st').textContent=d.state+(d.running?' ●':'');
  document.getElementById('step').textContent=d.step;
  document.getElementById('dt').textContent=d.detail||'';
  const ms=document.getElementById('mode');
  if(d.params&&!ms.matches(':focus'))ms.value=d.params.approach_mode;
}catch(e){}},400);
</script></body></html>"""


@app.get("/", include_in_schema=False)
async def index():
    return HTMLResponse(PAGE)


if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=PORT)
