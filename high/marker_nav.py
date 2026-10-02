"""
marker_nav.py — 정면 카메라 마커 항법 (포트 50050)
Version: 0.1

역할: 접근 전용.
  torso 정면에 붙인 USB 카메라(640x480)로 ArUco 마커를 보고
  테이블 앞 목표 거리까지 걸어가 정지한다. 그게 전부다.
  잡기(:50030/grab)와 놓기(:50030/place)는 외부에서 따로 호출한다 —
  접근은 잡기 전에도 놓기 전에도 같은 동작이므로 여기에 묶지 않는다.

  대기 → [/start] → 마커 탐색 → 펄스 접근(face회전/y게걸음/거리전진) → 정지(arrived)

  외부 조립 예:
    :50050/approach → (status.state=="arrived") → :50030/grab   # 잡기
    운반 → :50050/approach → (arrived) → :50030/place            # 놓기

이동 방식(펄스):
  멈춰서 측정 → 한 걸음 이동 → 정지 → 재측정 반복.
  이유 2가지.
    1) Unitree 보행 정책은 선속도 명령 크기가 0.2 미만이면 제자리로 학습돼 있다
       (unitree_rl_gym legged_robot.py). 즉 느린 연속 서보가 불가능하다.
    2) 저가 롤링셔터 카메라라 걸으면서 찍으면 마커가 뭉개진다.

규약:
  · loco 명령을 내는 주체는 언제나 하나. 미션 진행 중에는 이동하지 않는다.
  · 팔은 건드리지 않는다 (arm_server 는 mission_server 만 호출).

API:
  POST /approach       접근 — 마커 앞까지 걸어가 정지 (state="arrived")
  POST /stop           중단 (loco 정지)
  POST /mission/grab   잡기 — :50030/grab 프록시 (자기 loco 정지 후)
  POST /mission/place  놓기 — :50030/place 프록시
  POST /loco/move    수동 방향 이동 {vx,vy,vyaw} — 시험용, 50ms 간격 반복 호출
  POST /loco/stop    수동 정지
  POST /params       파라미터 변경
  GET  /status       상태
  GET  /pose         현재 마커 좌표 (torso 기준)
  GET  /video_feed     검출 오버레이 영상 (웹용)
  GET  /raw_feed       원본 MJPEG — 다른 프로그램이 카메라 영상 받아갈 때
  GET  /snapshot       원본 한 장 (JPEG)
"""

import os
import sys
import time
import threading
import urllib.request
import urllib.error

import numpy as np
import cv2
import uvicorn
from fastapi import FastAPI
from fastapi.responses import HTMLResponse, JSONResponse, StreamingResponse
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from contextlib import asynccontextmanager

current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_dir)

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from ctrl.arm_controller_wrapper import LocoClientWrapper

PORT = 50050
MISSION = os.environ.get("MISSION_SERVER", "http://localhost:50030")

# ==========================================
# 카메라 — 640x480 실촬영 캘리브레이션 (RMS 0.168px)
#   이 값은 이 카메라·이 해상도에서만 유효하다.
# ==========================================
CAM_DEV = os.environ.get("MARKER_CAM", "/dev/video6")
CAM_W, CAM_H = 640, 480

# 체스보드 캘리브레이션 (640x480, 9x6 내부코너 25mm, 9장, RMS 0.410px)
#   화각 H 64.8도 / V 51.0도
#   코너 분포 x 35~629 / y 75~426 — 네 사분면 모두 커버.
CAMERA_MATRIX = np.array([
    [504.0362,   0.0000, 314.2731],
    [  0.0000, 503.4106, 238.5117],
    [  0.0000,   0.0000,   1.0000],
], dtype=np.float64)
DIST_COEFFS = np.array([0.151700, -0.427100, -0.002000, 0.004300, 0.460900],
                       dtype=np.float64)

ARUCO_DICT_TYPE = cv2.aruco.DICT_4X4_50

# 카메라 장착 위치 (torso 원점 기준, m). 정면 수평 장착이라 회전 없음.
CAM_X = 0.05      # 앞으로
CAM_Y = 0.0       # 좌우 (중앙)
CAM_Z = 0.0       # 위아래

P = {
    "marker_size": 0.090,   # 마커 검은 테두리 바깥 한 변 (m)
                            #   광각 교체 후 45mm 는 원거리 인식 불안정 → 90mm
    "marker_id": -1,        # -1 = 아무 마커나, 0 이상이면 그 id 만
    "flip_h": 0,            # 1 이면 영상 좌우 반전을 되돌린다.
                            #   미러 출력 카메라를 그대로 두면 y 부호·face 각도·
                            #   회전 판단이 전부 반대가 된다.
                            #   확인: 로봇 왼쪽에 손 흔들어 화면 오른쪽에 보이면 1.

    # ── 목표 (도착 판정 세 가지) ─────────────────────────────
    # 1) 거리: 구간 안        2) 좌우: |y| ≤ lat_tol
    # 3) 정면성: |face| ≤ face_tol (마커 법선과 몸통이 이루는 각)
    #   창 폭은 한 펄스(0.12×0.9s ≈ 11cm)보다 넉넉해야 진동하지 않는다.
    "dist_min": 0.30,       # 이보다 가까우면 물러난다
    "dist_max": 0.45,       # 이보다 멀면 다가간다 (중심 0.375, 폭 15cm)
    "lat_tol": 0.10,        # 좌우 허용 오차 (m) — 게걸음 펄스 11cm 와 균형
    "face_tol": 12.0,       # 정면성 허용 오차 (deg). 180 이면 판정 끔.
                            #   하한은 회전 분해능이 정한다: 한 펄스 실효 9~17°라
                            #   이보다 좁히면 회전할 때마다 지나쳐 진동한다.
                            #   (실기: -17° 표시가 실제 기울기와 일치 확인 — 측정 유효)

    # ── 동작 배정 (한 오차에 한 동작, 서로 간섭 없음) ─────────
    # face → 회전     : face 는 몸통 방향의 함수라 회전으로만 바뀐다.
    #                   게걸음을 아무리 해도 face 는 1도도 안 변한다.
    # y    → 게걸음   : 게걸음은 face 를 건드리지 않는다.
    # 거리 → 전진     : y 가 밀리면 vy 를 살짝 섞어 사선으로 걷는다.
    # 순서는 face → y → 거리. 회전이 y 를 흔들면 다음에 게걸음이 잡고,
    # 게걸음은 face 를 안 흔드니 한 방향으로 수렴한다.

    # 속도 — 전진과 옆걸음의 문턱이 다르다 (실기 로그 확인):
    #   전진: 0.12 에서 펄스당 9~11cm 정상 보행 (0.11 까지 확인)
    #   옆걸음: 0.12 는 제자리걸음 — 12펄스 연속 y 변화 0. 문턱이 더 높다.
    "vmin_cmd": 0.12,
    "vx_max": 0.12,     # 근거리 전진/후진 (분해능 우선)
    "vx_far": 0.25,     # 원거리 전진 — 멀 때는 크게 성큼 (11cm 짤짤이 방지)
    "far_switch": 0.25, # 전후 오차가 이보다 크면 원거리 속도/긴 펄스 사용
    "pulse_far_t": 1.5, # 원거리 펄스 상한 (0.25×1.5 ≈ 37cm)
    "vy_max": 0.20,     # 게걸음 — 0.12 는 제자리걸음이라 확실히 걷는 0.20 (펄스당 ≈18cm).
                        #   미션 정렬은 0.16 으로 잘게 — 접근은 크게 잡아도 미션이 다듬는다
    # 전진 드리프트 보상. 실기: 전진 펄스마다 y 가 +0.07~0.08 밀린다(우측 편향).
    # 비례항만으로는 y 가 작을 때 보상이 모자라 상시 피드포워드를 더한다.
    "vy_ff": 0.08,          # 전진 중 상시 왼쪽(+) 보정. 반대로 밀리면 부호 반전
    "vy_blend_max": 0.15,   # 피드포워드+비례 합계 상한

    # 회전. 0.9초 미만 명령은 걸음이 시작되기 전에 끝나 거의 돌지 않는다
    # (실기 로그: 9도=0.52s → 방위 변화 0). 시간을 고정하고 매번 재측정.
    "vyaw": 0.30,
    "rot_pulse": 0.9,       # 회전 펄스 시간 (s) — 한 걸음 보장
    "rot_sign": 1,          # 회전 부호가 반대로 움직이면 -1
    "rot_boost_pos": 1.0,   # face<0 쪽(좌회전) 속도 배율 — 이 로봇은 좌회전이
                            #   약하다 (우회전 w=0.17에 11° vs 좌회전 w=0.27에 4°).
                            #   좌회전이 계속 모자라면 1.5~2.0 으로
    "face_fail_max": 4,     # 회전해도 |face| 가 안 줄면 이만큼 후 포기(도착 처리)

    # 전진 펄스
    "pulse_min": 0.9,       # 최소 펄스 (s) — 이보다 짧으면 걸음이 안 나온다
    "pulse_max_t": 1.0,     # 최대 펄스 (s, ≈22cm) — 길게 가면 드리프트 누적으로
                            #   근거리(0.3m 화면폭 22cm)에서 마커가 옆으로 빠진다
    "pulse_wait": 0.8,      # 정지 후 안정화 대기 (s)
    "measure_time": 0.8,    # 프레임 수집 시간 (s) — 중앙값
    "measure_min": 3,       # 최소 검출 수
    "pulse_max": 30,        # 최대 반복

    # 마커 탐색
    # 후진 탐색은 화각이 좁아(40°) 물러나도 각도 커버가 안 늘어 효과가 적었음
    # → 회전 탐색이 기본. 정면 기준 좌우 번갈아 ±search_span 까지 훑는다.
    "search_mode": "rotate",
    "search_enable": 1,
    "search_back_step": 0.9,
    "search_back_max": 4,
    "search_pulse": 0.9,    # 한 스텝 회전 시간 — 0.9s 미만은 걸음이 안 나온다
    "search_span": 90.0,

    "far_max": 2.0,         # 이 거리 초과 측정이면 실패 처리.
                            #   1.5 로 걸면 회전·후퇴 중 잠깐 넘는 측정에도
                            #   항법이 중단돼 여유를 둔다 (검출은 그 전에 끊김)
    "nav_timeout": 120.0,
}

# ==========================================
# 상태
# ==========================================
S = {"state": "idle", "step": "-", "detail": "", "t_run": 0.0}
LOG = []          # 최근 진행 기록 — 실패 후 원인 추적용
_run = threading.Event()
loco = None
cap = None
detector = None
_frame_lock = threading.Lock()
latest_frame = None     # 오버레이 포함 (웹 표시용)
latest_raw = None       # 원본 — 다른 프로그램이 받아 쓰는 용도 (rs_stream 방식)
latest_pose = None      # {"x","y","yaw_deg","id","t"}
last_seen = None        # 마지막으로 마커를 본 관측 — 소실 시 탐색 방향 결정용


def _log(msg):
    """진행 기록. 실패한 뒤 무슨 일이 있었는지 보려면 GET /log."""
    LOG.append(f"{time.strftime('%H:%M:%S')} {msg}")
    del LOG[:-80]
    print(f"[marker_nav] {msg}")


def _set(state=None, step=None, detail=None):
    if state is not None:
        S["state"] = state
    if step is not None:
        S["step"] = step
    if detail is not None:
        S["detail"] = detail
    print(f"[marker_nav] {S['state']} / {S['step']} {S['detail']}")


def _aborted():
    return not _run.is_set()


# ==========================================
# 카메라 → torso 변환
#   카메라: x 오른쪽, y 아래, z 앞    torso: x 앞, y 왼쪽, z 위
#   정면 수평 장착이라 회전 없이 축만 바꾼다.
# ==========================================
def cam_to_torso(cx, cy, cz):
    return (cz + CAM_X, -cx + CAM_Y, -cy + CAM_Z)


# ==========================================
# 카메라 루프
# ==========================================
def camera_loop():
    global latest_frame, latest_raw, latest_pose
    while True:
        ok, frame = cap.read()
        if not ok:
            time.sleep(0.05)
            continue
        if P["flip_h"]:
            frame = cv2.flip(frame, 1)      # 미러 카메라 되돌리기
        raw = frame.copy()          # 오버레이 그리기 전 원본
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = detector.detectMarkers(gray)

        pose = None
        if ids is not None and len(ids) > 0:
            pick = None
            for i, mid in enumerate(ids.flatten()):
                if P["marker_id"] >= 0 and int(mid) != P["marker_id"]:
                    continue
                pick = (i, int(mid))
                break
            if pick is not None:
                i, mid = pick
                # 마커 크기가 런타임에 바뀔 수 있으므로 매 프레임 구성
                sz = P["marker_size"] / 2
                o = np.array([[-sz, sz, 0], [sz, sz, 0],
                              [sz, -sz, 0], [-sz, -sz, 0]], dtype=np.float32)
                ok2, rvec, tvec = cv2.solvePnP(
                    o, corners[i].reshape(4, 2).astype(np.float32),
                    CAMERA_MATRIX, DIST_COEFFS, flags=cv2.SOLVEPNP_IPPE_SQUARE)
                if ok2:
                    tx, ty, tz = tvec.reshape(3)
                    mx, my, mz = cam_to_torso(tx, ty, tz)
                    # 마커 법선과 로봇 정면이 이루는 각 (정면성 판단용)
                    R, _ = cv2.Rodrigues(rvec)
                    nx, ny, _nz = cam_to_torso(R[0, 2], R[1, 2], R[2, 2])
                    nx -= CAM_X; ny -= CAM_Y      # 방향벡터라 오프셋 제거
                    yaw = float(np.degrees(np.arctan2(ny, nx)))
                    # 법선이 로봇 쪽을 향하므로 정면으로 마주보면 yaw=180.
                    # 0 이 정면이 되도록 바꾼다 (+면 마커가 왼쪽으로 틀어짐).
                    face = (180.0 - yaw + 180.0) % 360.0 - 180.0
                    pose = {"x": float(mx), "y": float(my), "z": float(mz),
                            "yaw_deg": yaw, "face_deg": face,
                            "id": mid, "t": time.time()}
                    cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                    cv2.drawFrameAxes(frame, CAMERA_MATRIX, DIST_COEFFS,
                                      rvec, tvec, P["marker_size"] * 0.5)
                    cv2.putText(frame,
                                f"id{mid} x={mx:.2f} y={my:+.2f} f={face:+.0f}",
                                (10, 30), 0, 0.65, (0, 255, 0), 2)

        if pose is None:
            cv2.putText(frame, "no marker", (10, 30), 0, 0.7, (0, 0, 255), 2)
        cv2.putText(frame, f"{S['state']} {S['step']}", (10, 460), 0, 0.6,
                    (255, 255, 0), 2)

        with _frame_lock:
            latest_frame = frame
            latest_raw = raw
            latest_pose = pose
            if pose is not None and abs(pose["y"]) < 0.6 and pose["x"] < 2.5:
                # 쓰레기 관측(모션블러 오검출 등)이 탐색 방향을 오염시키지 않게
                globals()["last_seen"] = pose
        time.sleep(0.01)


def get_pose(max_age=0.5):
    with _frame_lock:
        p = latest_pose
    if p is None or time.time() - p["t"] > max_age:
        return None
    return p


# ==========================================
# 이동
# ==========================================
def pulse(vx, vy, vyaw, dist=None):
    """dist(m) 만큼 갈 시간 동안 움직이고 멈춘다. 최소 pulse_min 보장."""
    if dist is None:
        dur = P["pulse_min"]
    else:
        spd = max(0.01, abs(vx) + abs(vy))
        dur = min(P["pulse_max_t"], max(P["pulse_min"], dist / spd))
    t0 = time.time()
    while time.time() - t0 < dur:
        if _aborted():
            loco.stop()
            return False
        loco.move(vx, vy, vyaw)
        time.sleep(0.05)
    loco.stop()
    return True


def settle_measure():
    """멈춘 뒤 안정화하고, 여러 프레임을 모아 중앙값으로 측정한다.

    단발 측정은 그 한 프레임이 모션 블러거나 검출 실패면 그대로 결과가 된다.
    안정화 대기(pulse_wait) 후 measure_time 동안 모아 중앙값을 쓴다.
    (detect_marker / detect_box 가 쓰는 방식과 같은 취지)
    """
    loco.stop()
    t0 = time.time()
    while time.time() - t0 < P["pulse_wait"]:
        if _aborted():
            return None
        time.sleep(0.05)

    xs, ys, zs, yaws, faces, ids = [], [], [], [], [], []
    seen = 0
    t1 = time.time()
    last_t = 0.0
    while time.time() - t1 < P["measure_time"]:
        if _aborted():
            return None
        p = get_pose(max_age=0.4)
        if p is not None and p["t"] != last_t:     # 같은 프레임 중복 방지
            last_t = p["t"]
            xs.append(p["x"]); ys.append(p["y"]); zs.append(p["z"])
            yaws.append(p["yaw_deg"]); faces.append(p.get("face_deg", 0.0))
            ids.append(p["id"])
            seen += 1
        time.sleep(0.03)

    if seen < P["measure_min"]:
        return None
    return {"x": float(np.median(xs)), "y": float(np.median(ys)),
            "z": float(np.median(zs)), "yaw_deg": float(np.median(yaws)),
            "face_deg": float(np.median(faces)),
            "id": int(np.median(ids)), "n": seen, "t": time.time()}


def search_back():
    """뒤로 물러나며 마커를 찾는다.

    화각이 좁아서 가까울수록 보이는 범위가 좁다(0.3m 에서 가로 22cm).
    물러나면 시야가 넓어져 좌우로 벗어났던 마커가 다시 들어온다.
    회전과 달리 각도 오차가 누적되지 않는 것도 장점.

    ※ 뒤는 카메라가 없다 — 뒤쪽 공간이 확보돼 있어야 한다.
    """
    _set(step="search_back", detail="뒤로 물러나며 탐색")
    for n in range(P["search_back_max"]):
        if _aborted():
            return None
        S["detail"] = f"뒤로 {n+1}/{P['search_back_max']}"
        _log(f"후진 탐색 {n+1}/{P['search_back_max']}")
        t0 = time.time()
        while time.time() - t0 < P["search_back_step"]:
            if _aborted():
                loco.stop()
                return None
            loco.move(-abs(P["vx_max"]), 0.0, 0.0)
            time.sleep(0.05)
        loco.stop()
        p = settle_measure()
        if p is not None:
            _set(detail=f"마커 발견 id{p['id']} (뒤로 {n+1}회)")
            return p
    return None


def search_rotate():
    """제자리 회전하며 마커를 찾는다.

    마지막으로 마커를 본 좌표를 기억해 뒀다가 그쪽 방향부터 훑는다 —
    마커는 대개 빠져나간 쪽에 있으므로, 반대쪽부터 돌면 화각(40°)이
    좁아 더 멀어져 영영 못 찾을 수 있다. 마지막 관측이 없으면 좌우 번갈아.
    """
    _set(step="search", detail="마커 탐색 중")

    step_deg = abs(P["vyaw"]) * P["search_pulse"] * 57.2958   # 1회 회전량
    span = P["search_span"]
    cur = 0.0          # 현재 정면 기준 누적 각도

    # 마지막으로 본 방향 (+1 왼쪽 / -1 오른쪽 / 0 모름)
    side = 0
    if last_seen is not None:
        side = 1 if last_seen["y"] > 0 else -1
        _log(f"탐색: 마지막 관측 y={last_seen['y']:+.2f} → "
             f"{'왼쪽' if side > 0 else '오른쪽'}부터")

    # 목표 각도열. 마지막 본 쪽을 먼저, 반대쪽은 나중에.
    targets = [0.0]
    k = 1
    while step_deg * k <= span:
        a, b = step_deg * k, -step_deg * k
        targets += ([b, a] if side < 0 else [a, b])
        k += 1
    if side != 0:
        # 본 쪽 각도들을 몰아서 먼저 훑는다: 0, 본쪽 s,2s,3s..., 반대쪽 s,2s...
        same = [t for t in targets if t * side > 0]
        oppo = [t for t in targets if t * side < 0]
        targets = [0.0] + same + oppo

    for i, tgt in enumerate(targets):
        if _aborted():
            return None
        # 현재 각도에서 목표 각도까지 회전
        delta = tgt - cur
        if abs(delta) > 1e-3:
            dur = abs(delta) / 57.2958 / abs(P["vyaw"])
            vy = P["vyaw"] if delta > 0 else -P["vyaw"]
            vy *= (1 if P["rot_sign"] >= 0 else -1)
            t0 = time.time()
            while time.time() - t0 < dur:
                if _aborted():
                    loco.stop()
                    return None
                loco.move(0.0, 0.0, vy)
                time.sleep(0.05)
            loco.stop()
            cur = tgt
        S["detail"] = f"탐색 {tgt:+.0f}도 ({i+1}/{len(targets)})"
        p = settle_measure()
        if p is not None:
            _set(detail=f"마커 발견 id{p['id']} ({tgt:+.0f}도)")
            return p

    # 못 찾았으면 정면으로 되돌리고 끝낸다
    if abs(cur) > 1e-3:
        dur = abs(cur) / 57.2958 / abs(P["vyaw"])
        vy = -P["vyaw"] if cur > 0 else P["vyaw"]
        vy *= (1 if P["rot_sign"] >= 0 else -1)
        t0 = time.time()
        while time.time() - t0 < dur:
            loco.move(0.0, 0.0, vy)
            time.sleep(0.05)
        loco.stop()
    return None


# ==========================================
# 항법
# ==========================================
def search_marker():
    """탐색 진입점 — 모드에 따라."""
    if not P["search_enable"]:
        return get_pose()
    if P["search_mode"] == "rotate":
        return search_rotate()
    return search_back()


def navigate():
    t0 = time.time()
    p = settle_measure()
    if p is None:
        p = search_marker()
    if p is None:
        return False, "마커를 찾지 못함"

    face_prev = None
    face_fail = 0
    for n in range(P["pulse_max"]):
        if _aborted():
            return False, "중단됨"
        if time.time() - t0 > P["nav_timeout"]:
            return False, "시간 초과"

        if p is None:
            p = settle_measure()
            if p is None:
                _log("측정 실패 — 탐색 시작")
                p = search_marker()
                if p is None:
                    _log("탐색 실패 — 마커 소실로 종료")
                    return False, "마커 소실"
                _log(f"탐색으로 재발견 x={p['x']:.2f} y={p['y']:+.2f}")

        mx, my = p["x"], p["y"]
        face = p.get("face_deg", 0.0)
        if mx > P["far_max"]:
            return False, f"마커가 {mx:.2f}m — 검출 한계(far_max) 밖"

        lo, hi = P["dist_min"], P["dist_max"]
        ok_x = lo <= mx <= hi
        ok_y = abs(my) <= P["lat_tol"]
        ok_f = abs(face) <= P["face_tol"]
        S["detail"] = (f"[{n+1}] x={mx:.2f}"
                       f"(구간{lo:.2f}~{hi:.2f}) "
                       f"y={my:+.2f} 정면{face:+.0f}° (n={p.get('n','?')})")

        if ok_x and ok_y and ok_f:
            loco.stop()
            _log(f"도착 x={mx:.2f} y={my:+.2f} 정면{face:+.0f}°")
            return True, f"도착 x={mx:.2f} y={my:+.2f} 정면{face:+.0f}도"

        # ── 1) 정면성 — 회전으로만 잡힌다 ──────────────────────
        #    face 는 몸통 방향의 함수. 좌회전(+vyaw)하면 face 가 커지므로
        #    face>0 은 우회전으로 줄인다. (이 로봇은 우회전이 강한 쪽이라
        #    로그상 face +30° 상황과 잘 맞는다)
        if not ok_f:
            if face_fail >= P["face_fail_max"]:
                loco.stop()
                _log(f"정면화 포기(회전해도 안 줄음) — 도착 처리. "
                     f"rot_sign/rot_boost_pos 확인")
                return True, (f"도착 x={mx:.2f} y={my:+.2f} "
                              f"정면{face:+.0f}도 (정면화 미완)")
            if face_prev is not None and abs(face) >= abs(face_prev) - 2.0:
                face_fail += 1
            else:
                face_fail = 0
            face_prev = face

            w = abs(P["vyaw"])
            if face < 0:
                w *= P["rot_boost_pos"]       # 좌회전(약한 쪽) 보상
            vyaw = -w if face > 0 else w      # face>0 → 우회전
            vyaw *= (1 if P["rot_sign"] >= 0 else -1)
            _log(f"[{n+1}] x={mx:.2f} y={my:+.2f} 정면{face:+.0f}° "
                 f"n={p.get('n','?')} → 회전 (w={vyaw:+.2f}, "
                 f"{P['rot_pulse']:.1f}s, 실패{face_fail})")
            S["detail"] += f" → 회전 w={vyaw:+.2f}"
            t1 = time.time()
            while time.time() - t1 < P["rot_pulse"]:
                if _aborted():
                    loco.stop()
                    return False, "중단됨"
                loco.move(0.0, 0.0, vyaw)
                time.sleep(0.05)
            loco.stop()
            p = None
            continue

        # ── 2) 좌우 — 게걸음 (face 를 건드리지 않는다) ─────────
        if not ok_y:
            vy = P["vy_max"] if my > 0 else -P["vy_max"]
            if abs(vy) < P["vmin_cmd"]:
                vy = P["vmin_cmd"] if vy > 0 else -P["vmin_cmd"]
            _log(f"[{n+1}] x={mx:.2f} y={my:+.2f} 정면{face:+.0f}° "
                 f"n={p.get('n','?')} → 좌우 {vy:+.2f}")
            S["detail"] += f" → 좌우 {vy:+.2f}"
            if not pulse(0.0, vy, 0.0, abs(my)):
                return False, "중단됨"
            p = None
            continue

        # ── 3) 거리 — 전진/후진 ──────────────────────────────
        #    멀면 크게(vx_far, 긴 펄스), 가까우면 잘게(vx_max) 간다.
        #    전진 중 우측 편향으로 y 가 밀리므로 피드포워드+비례로 상쇄.
        mid = (lo + hi) / 2.0
        err_x = mx - mid
        far = err_x > P["far_switch"]
        spd = P["vx_far"] if far else P["vx_max"]
        vx = spd if err_x > 0 else -P["vx_max"]
        if abs(vx) < P["vmin_cmd"]:
            vx = P["vmin_cmd"] if vx > 0 else -P["vmin_cmd"]
        vy = 0.0
        if err_x > 0:      # 전진할 때만 드리프트 보정
            vy = max(-P["vy_blend_max"],
                     min(P["vy_blend_max"], P["vy_ff"] + 0.6 * my))
        _log(f"[{n+1}] x={mx:.2f} y={my:+.2f} 정면{face:+.0f}° "
             f"n={p.get('n','?')} → 전후 {vx:+.2f}"
             + (f" (vy {vy:+.2f})" if vy else "")
             + (" [원거리]" if far else ""))
        S["detail"] += f" → 전후 {vx:+.2f}"
        cap = P["pulse_far_t"] if far else P["pulse_max_t"]
        dur = min(cap, max(P["pulse_min"], abs(err_x) / max(0.01, abs(vx))))
        t1 = time.time()
        while time.time() - t1 < dur:
            if _aborted():
                loco.stop()
                return False, "중단됨"
            loco.move(vx, vy, 0.0)
            time.sleep(0.05)
        loco.stop()
        p = None

    return False, f"미수렴 ({P['pulse_max']}회)"


def nav_thread():
    t0 = time.time()
    del LOG[:]
    _log(f"시작 (구간 {P['dist_min']:.2f}~{P['dist_max']:.2f}m, "
         f"좌우±{P['lat_tol']:.2f}m, 정면±{P['face_tol']:.0f}°, "
         f"rot_sign={P['rot_sign']} boost={P['rot_boost_pos']})")
    try:
        _set(state="nav", step="approach", detail="")
        ok, msg = navigate()
        loco.stop()
        _log(f"종료: {msg}")
        if not ok:
            _set(state="stopped" if msg == "중단됨" else "error", detail=msg)
            return
        _set(state="arrived", step="-", detail=msg)
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
    global loco, cap, detector
    print("[marker_nav] 시작")
    ChannelFactoryInitialize(0)
    loco = LocoClientWrapper()

    adict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT_TYPE)
    try:
        params = cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(adict, params)
    except AttributeError:
        params = cv2.aruco.DetectorParameters_create()

        class _Legacy:
            def detectMarkers(self, g):
                return cv2.aruco.detectMarkers(g, adict, parameters=params)
        detector = _Legacy()

    cap = cv2.VideoCapture(CAM_DEV)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAM_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_H)
    if not cap.isOpened():
        print(f"[marker_nav] ⚠ 카메라 열기 실패: {CAM_DEV}")
    else:
        aw = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        ah = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        print(f"[marker_nav] 카메라 {CAM_DEV} {aw}x{ah}")
        if (aw, ah) != (CAM_W, CAM_H):
            print(f"[marker_nav] ⚠ 해상도가 {CAM_W}x{CAM_H} 가 아니다 — "
                  f"캘리브레이션 값이 맞지 않는다")
        threading.Thread(target=camera_loop, daemon=True).start()

    print(f"[marker_nav] 준비 완료  http://localhost:{PORT}/")
    yield
    try:
        loco.stop()
    except Exception:
        pass
    if cap is not None:
        cap.release()
    print("[marker_nav] 종료")


app = FastAPI(title="G1 Marker Nav", lifespan=lifespan)
app.add_middleware(CORSMiddleware, allow_origins=["*"],
                   allow_methods=["*"], allow_headers=["*"])


@app.get("/status")
async def status():
    p = get_pose()
    return {**S, "running": _run.is_set(), "marker": p, "params": P}


@app.get("/log")
async def log():
    """직전 항법의 진행 기록. 실패 원인 추적용."""
    return {"lines": LOG}


@app.get("/pose")
async def pose():
    p = get_pose()
    return p if p else {"found": False}


@app.post("/approach")
async def approach():
    """접근. 마커 앞까지 걸어가 정지 — 도착하면 state="arrived".
    잡기(:50030/grab)/놓기(:50030/place)는 외부에서 따로 호출한다."""
    if _run.is_set():
        return JSONResponse({"ok": False, "reason": "이미 진행 중"}, status_code=409)
    _run.set()
    S.update(state="starting", step="-", detail="", t_run=0.0)
    threading.Thread(target=nav_thread, daemon=True).start()
    return {"ok": True}


@app.post("/stop")
async def stop():
    _run.clear()
    try:
        loco.stop()
    except Exception:
        pass
    if S["state"] not in ("idle", "done"):
        S["state"] = "stopped"
    return {"ok": True}


class MoveReq(BaseModel):
    vx: float = 0.0
    vy: float = 0.0
    vyaw: float = 0.0


@app.post("/loco/move")
async def loco_move(req: MoveReq):
    """수동 이동 — 항법이 돌고 있으면 거부한다 (이동 명령 소스는 하나)."""
    if _run.is_set():
        return JSONResponse({"ok": False, "reason": "항법 진행 중 — 이동 금지"},
                            status_code=409)
    loco.move(req.vx, req.vy, req.vyaw)
    return {"ok": True}


@app.post("/loco/stop")
async def loco_stop():
    loco.stop()
    return {"ok": True}


def _mission_call(path):
    """미션 서버 프록시. 항법 중이면 거부, 호출 전 자기 loco 정지."""
    if _run.is_set():
        return JSONResponse({"ok": False, "reason": "접근 진행 중 — 먼저 정지"},
                            status_code=409)
    try:
        loco.stop()
    except Exception:
        pass
    time.sleep(0.3)
    try:
        req = urllib.request.Request(MISSION + path, data=b"{}",
                                     headers={"Content-Type": "application/json"},
                                     method="POST")
        with urllib.request.urlopen(req, timeout=10) as r:
            import json as _json
            return _json.loads(r.read())
    except urllib.error.HTTPError as e:
        return JSONResponse({"ok": False,
                             "reason": f"미션 거부: {e.read()[:120]}"},
                            status_code=409)
    except Exception as e:
        return JSONResponse({"ok": False, "reason": f"미션 호출 실패: {e}"},
                            status_code=502)


@app.post("/mission/grab")
async def mission_grab():
    """잡기 — mission_server /grab 을 대신 호출한다."""
    return _mission_call("/grab")


@app.post("/mission/place")
async def mission_place():
    """놓기 — mission_server /place 를 대신 호출한다."""
    return _mission_call("/place")


@app.post("/params")
async def params(body: dict):
    for k, v in body.items():
        if k in P:
            P[k] = type(P[k])(v)
    return {"ok": True, "params": P}


def gen_frames(raw=False):
    while True:
        with _frame_lock:
            src_f = latest_raw if raw else latest_frame
            f = None if src_f is None else src_f.copy()
        if f is not None:
            ok, buf = cv2.imencode('.jpg', f)
            if ok:
                yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n'
                       + buf.tobytes() + b'\r\n')
        time.sleep(0.05)


@app.get("/video_feed")
async def video_feed():
    """검출 오버레이 포함 MJPEG (웹 표시용)."""
    return StreamingResponse(gen_frames(False),
                             media_type='multipart/x-mixed-replace; boundary=frame')


@app.get("/raw_feed")
async def raw_feed():
    """원본 MJPEG — 다른 프로그램이 이 카메라 영상을 받아 쓸 때.
    카메라 장치는 한 프로세스만 열 수 있으므로, 직접 열지 말고 이걸 쓸 것.
    (rs_stream 의 /video_feed 와 같은 용법)"""
    return StreamingResponse(gen_frames(True),
                             media_type='multipart/x-mixed-replace; boundary=frame')


@app.get("/snapshot")
async def snapshot():
    """원본 한 장 (JPEG). 폴링 방식 소비자용."""
    from fastapi.responses import Response
    with _frame_lock:
        f = None if latest_raw is None else latest_raw.copy()
    if f is None:
        return JSONResponse({"ok": False, "reason": "프레임 없음"}, status_code=503)
    ok, buf = cv2.imencode('.jpg', f)
    if not ok:
        return JSONResponse({"ok": False, "reason": "인코딩 실패"}, status_code=500)
    return Response(content=buf.tobytes(), media_type="image/jpeg")


PAGE = """<!DOCTYPE html><html lang="ko"><head><meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>G1 Marker Nav</title>
<style>
body{margin:0;background:#0e1116;color:#c9d4e0;font-family:ui-monospace,Menlo,monospace;
  min-height:100vh;display:flex;flex-direction:column;align-items:center;gap:16px;padding:24px}
h1{font-size:17px;color:#3ddc97;margin:0}
.state{font-size:14px;padding:9px 18px;border:1px solid #2a3340;border-radius:10px;background:#161b22}
.state b{color:#4aa8ff}
button{font:inherit;border:none;border-radius:12px;cursor:pointer;padding:18px 0;width:300px;
  font-size:17px;font-weight:700}
#b-start{background:#3ddc97;color:#05221a}
#b-stop{background:#ff6b6b;color:#2a0505}
.row{display:flex;gap:10px;width:300px}
.row button{flex:1;padding:15px 0;font-size:15px}
#b-grab{background:#f7b731;color:#2a1d00}
#b-place{background:#a55eea;color:#12042a}
img{width:640px;max-width:96vw;border:1px solid #2a3340;border-radius:8px;background:#05080c}
.pad{display:grid;grid-template-columns:repeat(5,58px);grid-template-rows:repeat(3,48px);gap:5px}
.pad button{background:#1c232d;border:1px solid #2a3340;color:#c9d4e0;border-radius:9px;
  font-size:18px;cursor:pointer;font-family:inherit;padding:0;width:auto}
.pad button.active{background:#3ddc97;color:#05221a}
#p-f{grid-area:1/3}#p-b{grid-area:3/3}#p-l{grid-area:2/2}#p-r{grid-area:2/4}
#p-s{grid-area:2/3;color:#ff6b6b}#p-tl{grid-area:2/1}#p-tr{grid-area:2/5}
.spd{display:flex;gap:10px;align-items:center;font-size:12px;color:#6b7785;width:330px}
.spd input{flex:1}
.hint{color:#6b7785;font-size:11px;text-align:center;line-height:1.7}
#log{width:640px;max-width:96vw;max-height:220px;overflow-y:auto;margin:0;
  background:#0a0d12;border:1px solid #2a3340;border-radius:8px;padding:10px;
  font-size:11px;line-height:1.6;color:#8b949e;white-space:pre-wrap}
</style></head><body>
<h1>G1 Marker Nav · 정면 카메라 마커 항법</h1>
<div class="state">state: <b id="st">-</b> · <span id="dt"></span></div>
<button id="b-start" onclick="post('/approach')">▶ 접근</button>
<button id="b-stop" onclick="post('/stop')">■ 정지</button>
<div class="row">
  <button id="b-grab" onclick="post('/mission/grab')">✊ 잡기</button>
  <button id="b-place" onclick="post('/mission/place')">🖐 놓기</button>
</div>

<div class="pad">
  <button id="p-f"  data-cmd="forward">▲</button>
  <button id="p-tl" data-cmd="turn_left">↺</button>
  <button id="p-l"  data-cmd="left">◀</button>
  <button id="p-s"  onclick="stopLoco()">■</button>
  <button id="p-r"  data-cmd="right">▶</button>
  <button id="p-tr" data-cmd="turn_right">↻</button>
  <button id="p-b"  data-cmd="backward">▼</button>
</div>
<div class="spd"><span>속도</span>
  <input type="range" id="speed" min="0.05" max="0.5" step="0.01" value="0.15">
  <span id="spdv" style="color:#3ddc97">0.15</span></div>

<img id="feed" alt="marker">
<pre id="log"></pre>
<div class="hint">마커 90mm · 정지 구간 0.3~0.45m · 좌우 ±0.10 · 정면 ±12° · 검출 한계 약 1.5m<br>
마커를 놓치면 제자리에서 좌우로 회전하며 다시 찾습니다 (±90°)<br>
방향키: ↑↓←→ 이동 · Q,E 회전 · Esc 정지 (항법 진행 중엔 잠김)<br>
잡기/놓기 버튼은 미션 서버(:50030)를 호출합니다 — 진행 상황은 미션 웹에서</div>
<script>
document.getElementById('feed').src='/video_feed';
async function post(p){stopLoco();try{const r=await fetch(p,{method:'POST'});const d=await r.json();
  if(d.reason)alert(d.reason);}catch(e){alert(e.message);}}

const speed=document.getElementById('speed'),spdv=document.getElementById('spdv');
speed.oninput=()=>spdv.textContent=(+speed.value).toFixed(2);
const cmap={forward:()=>({vx:+speed.value,vy:0,vyaw:0}),
  backward:()=>({vx:-speed.value,vy:0,vyaw:0}),
  left:()=>({vx:0,vy:+speed.value,vyaw:0}),
  right:()=>({vx:0,vy:-speed.value,vyaw:0}),
  turn_left:()=>({vx:0,vy:0,vyaw:+speed.value}),
  turn_right:()=>({vx:0,vy:0,vyaw:-speed.value})};
let lt=null,ab=null,warned=false;
function startLoco(c,b){if(lt)return;ab=b;if(b)b.classList.add('active');
  const s=()=>fetch('/loco/move',{method:'POST',
    headers:{'Content-Type':'application/json'},body:JSON.stringify(cmap[c]())})
    .then(r=>{if(r.status===409&&!warned){warned=true;alert('항법 진행 중 — 이동 잠금');
      stopLoco();}}).catch(()=>{});
  s();lt=setInterval(s,50);}
function stopLoco(){warned=false;if(lt){clearInterval(lt);lt=null;}
  if(ab){ab.classList.remove('active');ab=null;}
  fetch('/loco/stop',{method:'POST'}).catch(()=>{});}
document.querySelectorAll('.pad button[data-cmd]').forEach(b=>{const c=b.dataset.cmd;
  b.addEventListener('mousedown',e=>{e.preventDefault();startLoco(c,b);});
  b.addEventListener('mouseup',e=>{e.preventDefault();stopLoco();});
  b.addEventListener('mouseleave',()=>stopLoco());
  b.addEventListener('touchstart',e=>{e.preventDefault();startLoco(c,b);},{passive:false});
  b.addEventListener('touchend',e=>{e.preventDefault();stopLoco();});});
const km={'ArrowUp':'forward','ArrowDown':'backward','ArrowLeft':'left',
  'ArrowRight':'right','q':'turn_left','e':'turn_right'};
document.addEventListener('keydown',e=>{if(e.repeat)return;
  if(['INPUT','SELECT'].includes(e.target.tagName))return;
  const c=km[e.key];if(c){e.preventDefault();
    startLoco(c,document.querySelector(`[data-cmd="${c}"]`));}
  if(e.key==='Escape'){e.preventDefault();stopLoco();}});
document.addEventListener('keyup',e=>{if(km[e.key]){e.preventDefault();stopLoco();}});
window.addEventListener('beforeunload',stopLoco);
setInterval(async()=>{try{const d=await(await fetch('/status')).json();
  document.getElementById('st').textContent=d.state+(d.running?' ●':'');
  document.getElementById('dt').textContent=(d.step&&d.step!=='-'?d.step+' · ':'')+(d.detail||'');
}catch(e){}},400);
setInterval(async()=>{try{const d=await(await fetch('/log')).json();
  const el=document.getElementById('log');
  const bottom=el.scrollHeight-el.scrollTop-el.clientHeight<30;
  el.textContent=(d.lines||[]).join(String.fromCharCode(10));
  if(bottom)el.scrollTop=el.scrollHeight;
}catch(e){}},1000);
</script></body></html>"""


@app.get("/", include_in_schema=False)
async def index():
    return HTMLResponse(PAGE)


if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=PORT)
