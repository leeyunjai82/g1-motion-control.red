#!/usr/bin/env python3
# Version: 1.64
# Changes from 1.63:
#   - 박스 크기 (W/D/H) 웹에서 실시간 변경 가능
#     · 전역 box_size dict
#     · BOX_CORNERS_3D → get_box_corners() 동적 계산
#     · grab_sequence가 매번 box_size 스냅샷 사용
#     · /set_box_size?width=0.28&depth=0.09&height=0.09 엔드포인트
#     · 웹 UI에 W/D/H 입력 카드 (cm 단위)
# Changes from 1.62:
#   - 건네기 방향 옵션 (center / left / right)
#     · HANDOVER_DIRECTION, HANDOVER_YAW_DEG 상단 상수
#     · 웹 UI에 ◀ 왼쪽 / ■ 중앙 / 오른쪽 ▶ 버튼 + yaw 각도 입력
#     · /set_handover_direction?direction=left|center|right&yaw_deg=30 엔드포인트
#     · 잡기 → 들기 후 waist yaw를 해당 각도로 회전한 뒤 건네기
# Changes from 1.61:
#   - USE_ARM_CONTROLLER / USE_TTS 상단 토글 추가
#     · False면 해당 모듈을 import 자체를 건너뜀 (의존성 없는 환경에서 실행 가능)
#     · True면 기존 동작 그대로 (import 실패 시 자동 fallback)
# Changes from 1.60:
#   - detect를 전용 스레드로 분리 (30fps 유지)
#     · generate_frames는 송신만 (5fps로 떨어뜨려도 검출은 계속 30fps)
#     · latest_annotated 변수에 그려진 이미지 저장
#     · 검출 주기와 웹 송신 주기 완전 독립
# Changes from 1.59:
#   - 웹 스트림 성능 최적화
#     · 출력 해상도 640x480 → 320x240 (브라우저에서 2배 확대 표시)
#     · 출력 FPS 상한 5fps (송신만 — 검출은 30fps 유지)
#     · JPEG 품질 80 → 70
# Changes from 1.58:
#   - 마커 pose 1초 윈도우 중앙값 안정화 (튀는 값 outlier 제거)
#     · pose_history deque에 매 프레임 raw pose 저장
#     · 최근 1초 + 같은 ID 샘플로 tvec/rvec 컴포넌트별 중앙값 계산
#     · 3샘플 미만일 때는 raw 그대로 사용 (첫 검출 끊김 방지)
#     · 시각 오버레이(draw_box_3d)는 raw 위치 그대로
# Changes from 1.57:
#   - 손목 yaw 값 변경: ±10 → ±15 (더 안쪽으로 회전)
#   - lift_z: mz + 0.22 → mz + 0.15 (들기/건네기 높이 7cm 낮춤)
"""
Unitree G1 + 원격/로컬 MJPEG 스트림
ArUco 마커 3D 박스 오버레이 + Start / Release / Home + TTS
"""

import os
import sys
import threading
import time
import random
import urllib.request
from collections import deque
import numpy as np
import cv2
import uvicorn
from contextlib import asynccontextmanager
from fastapi import FastAPI
from fastapi.responses import HTMLResponse, StreamingResponse, JSONResponse
from fastapi.middleware.cors import CORSMiddleware

# ==========================================
# 모듈 활성화 토글 (False면 import 자체를 건너뜀)
# ==========================================
USE_ARM_CONTROLLER = True    # False = arm_controller import 안 함 (순수 시뮬)
USE_TTS            = False    # False = TTS import 안 함 (조용히 더미)

# ==========================================
# 로봇 라이브러리 로드
# ==========================================
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir  = os.path.dirname(current_dir)
sys.path.append(parent_dir)

if USE_ARM_CONTROLLER:
    try:
        from ctrl.arm_controller_wrapper import ArmControllerWrapper
        ROBOT_AVAILABLE = True
        print("[시스템] 로봇 라이브러리 로드 성공")
    except ImportError as e:
        ROBOT_AVAILABLE = False
        print(f"[경고] 시뮬레이션 모드: {e}")
else:
    ROBOT_AVAILABLE = False
    print("[시스템] arm_controller 비활성화 (USE_ARM_CONTROLLER=False)")

# TTS 로드 (실패해도 동작 계속 — speak가 더미 함수가 됨)
if USE_TTS:
    try:
        from ctrl.text_to_speech import TextToSpeech
        tts = TextToSpeech()
        def speak(text):
            tts.speak(text)
        print("[시스템] TTS 로드 성공")
    except Exception as e:
        print(f"[경고] TTS 로드 실패 (조용히 진행): {e}")
        def speak(text):
            print(f"[TTS-DUMMY] {text}")
else:
    print("[시스템] TTS 비활성화 (USE_TTS=False)")
    def speak(text):
        print(f"[TTS-DUMMY] {text}")

# ==========================================
# 설정
# ==========================================
RS_STREAM_URL = os.environ.get("RS_STREAM_URL", "http://localhost:50001/video_feed")

# 카메라 캘리브레이션 (camera_calib.py 실행해서 출력된 값을 여기에 붙여넣기)
CAM_WIDTH  = 640
CAM_HEIGHT = 480
CAM_FX     = 606.756104
CAM_FY     = 606.583374
CAM_PPX    = 316.739441
CAM_PPY    = 258.982391
CAM_DIST   = [0.0, 0.0, 0.0, 0.0, 0.0]

ARUCO_DICT_TYPE = cv2.aruco.DICT_4X4_50
MARKER_SIZE     = 0.045

CAMERA_X          = 0.0576235
CAMERA_Y          = 0.03003#0.05003 #0.01753 + 0.0175 + 0.015
CAMERA_Z          = 0.42987
CAMERA_PITCH_URDF = 0.8307767239493009  # 47.6도

# init / 일반 동작용 HOME (작업 대기 자세 — 살짝 앞으로 뻗음)
HOME_LEFT  = [0.15,  0.25, 0.20]
HOME_RIGHT = [0.15, -0.25, 0.20]

# 종료용 PARK 자세 (차렷에 가까운 자연스러운 park 자세)
SHUTDOWN_HOME_LEFT  = [0.2,  0.2, 0.0]
SHUTDOWN_HOME_RIGHT = [0.2, -0.2, 0.0]

GRIP_EXTRA = -0.050
APPROACH_EXTRA = 0.10
GRAB_Z_OFFSET = 0.08
GRAB_X_OFFSET = -0.15

# 박스 크기 (런타임 변경 가능 — 웹 UI에서 조정)
box_size = {
    "width":  0.28,   # 양손이 잡는 면 사이 폭 (m)
    "depth":  0.09,
    "height": 0.09,
}

# 왼손 Y 추가 오프셋 (양수 = 왼손이 박스에서 더 멀리 = 박스가 좌측으로 치우치는 거 보정)
# 0.0이면 비활성. 카메라 좌표 보정 우선 시도 중이라 일단 0.
LEFT_HAND_Y_OFFSET = 0.0

HANDOVER_X = 0.30   # 건네기 X 거리 (작을수록 가까이, 이전 0.40)

# ==========================================
# Handover 방향 (center / left / right)
# ==========================================
# "center" = 정면 (waist 0)
# "left"   = 받는 사람이 로봇의 왼쪽 → waist yaw 양수
# "right"  = 받는 사람이 로봇의 오른쪽 → waist yaw 음수
HANDOVER_DIRECTION = "center"   # "center" | "left" | "right"
HANDOVER_YAW_DEG   = 30.0       # left/right일 때 회전 각도

# ==========================================
# 마커 pose 안정화 (튀는 값 outlier 제거)
# ==========================================
AVERAGING_WINDOW_SEC  = 1.0   # 최근 N초 데이터로 중앙값 계산
AVERAGING_MIN_SAMPLES = 3     # 최소 N개 모이면 평균 적용 (미달 시 raw 사용)

# ==========================================
# 웹 스트림 출력 (성능 최적화 — detect는 그대로, 출력만 줄임)
# ==========================================
WEB_STREAM_WIDTH   = 320   # 출력 너비 (브라우저에서 2배 확대해서 표시)
WEB_STREAM_HEIGHT  = 240
WEB_STREAM_FPS_MAX = 5     # 출력 프레임율 상한 (시연용 — 검출에도 충분)
WEB_STREAM_QUALITY = 70    # JPEG 품질 (기존 80 → 70)

# ==========================================
# TTS 멘트 (영어 - 쉬운 단어)
# ==========================================
MSG_INIT     = "Hi, Red Hat Summit! I have gifts for you."
MSG_TRIGGER  = "A box! Let me pick it up."
MSG_PICKED   = "I got it."
MSG_HANDOVER = "This is for you! Please grab the top of the box."
MSG_RECEIVED = "Enjoy your gift!"
MSG_TIMEOUT  = "Nobody? I will put it back."
MSG_HOME     = "Who is next? Bring me another box."

# 대기 잡소리 (Red Hat 6 + Intel 2 + Circulus 2)
IDLE_CHATTER = [
    "Welcome to Red Hat Summit 2026!",
    "Free Owala tumbler! Come and get one.",
    "Bring me a box, and I will give you a gift.",
    "I run on RHEL.",
    "I run on open source.",
    "I am faster than OpenShift.",
    "I run on Intel Panther Lake.",
    "I have Intel inside.",
    "My software is from Circulus, Korea.",
    "Circulus from Korea made my software.",
]
IDLE_INTERVAL_MIN = 30.0
IDLE_INTERVAL_MAX = 60.0

MARKER_OBJ_PTS = np.array([
    [-MARKER_SIZE/2,  MARKER_SIZE/2, 0],
    [ MARKER_SIZE/2,  MARKER_SIZE/2, 0],
    [ MARKER_SIZE/2, -MARKER_SIZE/2, 0],
    [-MARKER_SIZE/2, -MARKER_SIZE/2, 0],
], dtype=np.float32)

def get_box_corners():
    """현재 box_size 기준 8개 꼭짓점 (마커가 윗면 중심)"""
    hw = box_size["width"]  / 2
    hd = box_size["depth"]  / 2
    h  = box_size["height"]
    return np.array([
        [-hw, -hd,  0],
        [ hw, -hd,  0],
        [ hw,  hd,  0],
        [-hw,  hd,  0],
        [-hw, -hd, -h],
        [ hw, -hd, -h],
        [ hw,  hd, -h],
        [-hw,  hd, -h],
    ], dtype=np.float32)

BOX_EDGES = [
    (0,1),(1,2),(2,3),(3,0),
    (4,5),(5,6),(6,7),(7,4),
    (0,4),(1,5),(2,6),(3,7),
]

# ==========================================
# 전역 변수
# ==========================================
camera_matrix = None
dist_coeffs   = None
image_width   = 640
image_height  = 480

latest_image       = None
latest_annotated   = None      # detect/오버레이가 그려진 이미지 (송신용)
latest_markers     = []
latest_marker_pose = None
marker_last_seen_time = 0.0

# 마커 pose 히스토리 (1초 윈도우 중앙값 계산용)
pose_history = deque(maxlen=120)   # 30fps × 4초 여유

lock           = threading.Lock()
pose_lock      = threading.Lock()
image_lock     = threading.Lock()
annotated_lock = threading.Lock()

stream_started = False
arm            = None
aruco_dict     = None
aruco_params   = None
aruco_detector = None

grab_state = {'active': False, 'lifted_left': None, 'lifted_right': None, 'busy': False}

# 자동 모드 설정
AUTO_DEFAULT = {
    "enabled": True,
    "x_min": 0.30, "x_max": 0.40,
    "y_min": -0.15, "y_max": 0.15,
    "z_min": -0.10, "z_max": 0.20,
    "dwell_sec": 1.0,
}
auto_mode = dict(AUTO_DEFAULT)
auto_state = {"in_zone_since": None}

wrist_params = {
    'left':  {'roll': -10.0, 'pitch': -10.0, 'yaw': -15.0},
    'right': {'roll':  10.0, 'pitch': -10.0, 'yaw':  15.0},
}


# ==========================================
# 카메라 매트릭스 구성 (상단 상수로부터)
# ==========================================
def setup_camera_matrix():
    global camera_matrix, dist_coeffs, image_width, image_height
    image_width  = CAM_WIDTH
    image_height = CAM_HEIGHT
    camera_matrix = np.array([
        [CAM_FX, 0,      CAM_PPX],
        [0,      CAM_FY, CAM_PPY],
        [0,      0,      1      ]
    ], dtype=np.float32)
    if len(CAM_DIST) == 5:
        dist_coeffs = np.array(CAM_DIST, dtype=np.float32)
    else:
        dist_coeffs = np.zeros((5,), dtype=np.float32)
    print(f"[CALIB] {image_width}x{image_height}, fx={CAM_FX:.1f}, fy={CAM_FY:.1f}, "
          f"ppx={CAM_PPX:.1f}, ppy={CAM_PPY:.1f}")


# ==========================================
# ArUco 초기화
# ==========================================
def init_aruco():
    global aruco_dict, aruco_params, aruco_detector
    aruco_dict   = cv2.aruco.getPredefinedDictionary(ARUCO_DICT_TYPE)
    aruco_params = cv2.aruco.DetectorParameters()
    try:
        aruco_detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
    except AttributeError:
        aruco_detector = None


# ==========================================
# MJPEG 스트림 수신 (별도 스레드)
# ==========================================
def stream_reader_loop():
    global latest_image, stream_started

    while True:
        print(f"[STREAM] 연결 시도: {RS_STREAM_URL}")
        try:
            req = urllib.request.urlopen(RS_STREAM_URL, timeout=5)
            stream_started = True
            print("[STREAM] 연결 성공")

            buf = b""
            while True:
                chunk = req.read(4096)
                if not chunk:
                    break
                buf += chunk

                while True:
                    soi = buf.find(b'\xff\xd8')
                    eoi = buf.find(b'\xff\xd9', soi + 2) if soi >= 0 else -1
                    if soi < 0 or eoi < 0:
                        break
                    jpg = buf[soi:eoi+2]
                    buf = buf[eoi+2:]

                    img = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
                    if img is not None:
                        with image_lock:
                            latest_image = img

        except Exception as e:
            print(f"[STREAM] 오류: {e}")
            stream_started = False
            time.sleep(2.0)


# ==========================================
# 로봇 초기화
# ==========================================
def init_robot():
    global arm
    if not ROBOT_AVAILABLE:
        return
    try:
        arm = ArmControllerWrapper(motion_mode=True, simulation_mode=False)
        arm.start()
        print("[로봇] 초기화 완료")
        print("[로봇] waist 0으로 리셋")
        arm.move_waist_smooth(yaw=0.0, roll=0.0, pitch=0.0, duration=2.0)
        time.sleep(2.0)

        print("[로봇] HOME 자세로 이동")
        try:
            arm.move_hands(HOME_LEFT, HOME_RIGHT, None, None, 2.0, 100)
            time.sleep(0.5)
        except Exception as e:
            print(f"[로봇] HOME 이동 실패: {e}")
    except Exception as e:
        print(f"[로봇] 초기화 실패: {e}")


def auto_monitor_loop():
    """마커 torso 좌표가 영역 안에 dwell_sec 머물면 자동 잡기 트리거."""
    while True:
        time.sleep(0.1)

        if not auto_mode["enabled"]:
            auto_state["in_zone_since"] = None
            continue
        if grab_state.get('busy'):
            auto_state["in_zone_since"] = None
            continue

        with pose_lock:
            pose = latest_marker_pose
        if pose is None or not is_marker_visible(threshold_sec=0.3):
            auto_state["in_zone_since"] = None
            continue

        tvec = pose['tvec']
        mx, my, mz = camera_to_torso(tvec[0], tvec[1], tvec[2])

        in_zone = (
            auto_mode["x_min"] <= mx <= auto_mode["x_max"] and
            auto_mode["y_min"] <= my <= auto_mode["y_max"] and
            auto_mode["z_min"] <= mz <= auto_mode["z_max"]
        )

        if not in_zone:
            auto_state["in_zone_since"] = None
            continue

        if auto_state["in_zone_since"] is None:
            auto_state["in_zone_since"] = time.time()
            print(f"[AUTO] 영역 진입: torso=[{mx:.3f},{my:.3f},{mz:.3f}]")
            continue

        elapsed = time.time() - auto_state["in_zone_since"]
        if elapsed >= auto_mode["dwell_sec"]:
            print(f"[AUTO] {auto_mode['dwell_sec']}초 머무름 → 자동 잡기 트리거")
            speak(MSG_TRIGGER)
            launch_grab(tvec)


def idle_chatter_loop():
    """대기 상태에서 잡소리 로테이션. 박스 보이거나 동작 중이면 침묵."""
    # 셔플 큐
    queue = list(IDLE_CHATTER)
    random.shuffle(queue)
    idx = 0

    # 시작 시 첫 잡소리까지 잠깐 대기 (init 멘트 끝나고 나오게)
    time.sleep(15.0)

    while True:
        # 잡소리 조건: 마커 안 보임 + busy 아님 + grab_active 아님
        can_speak = (
            not is_marker_visible(threshold_sec=0.5) and
            not grab_state.get('busy') and
            not grab_state.get('active')
        )

        if can_speak:
            text = queue[idx]
            idx += 1
            if idx >= len(queue):
                random.shuffle(queue)
                idx = 0
            speak(text)

        # 30~60초 랜덤 대기
        wait = random.uniform(IDLE_INTERVAL_MIN, IDLE_INTERVAL_MAX)
        time.sleep(wait)


@asynccontextmanager
async def lifespan(app: FastAPI):
    setup_camera_matrix()
    init_aruco()

    t = threading.Thread(target=stream_reader_loop, daemon=True)
    t.start()

    print("[STREAM] 첫 프레임 대기...")
    for _ in range(100):
        with image_lock:
            if latest_image is not None:
                break
        time.sleep(0.1)
    if latest_image is None:
        print("[STREAM] 프레임 미도착 — 계속 진행")
    else:
        print("[STREAM] 프레임 수신 시작")

    init_robot()

    threading.Thread(target=detect_loop, daemon=True).start()
    print("[DETECT] 검출 스레드 시작 (30fps)")

    threading.Thread(target=auto_monitor_loop, daemon=True).start()
    print("[AUTO] 자동 모니터 스레드 시작")

    threading.Thread(target=idle_chatter_loop, daemon=True).start()
    print("[CHATTER] 잡소리 스레드 시작")

    # 시작 인사
    speak(MSG_INIT)

    yield

    # ==========================================
    # 안전 종료 시퀀스
    # ==========================================
    print("[shutdown] 종료 시퀀스 시작")
    t_shutdown = time.time()

    auto_mode["enabled"] = False

    busy_deadline = time.time() + 3.0
    while grab_state.get('busy') and time.time() < busy_deadline:
        time.sleep(0.1)
    if grab_state.get('busy'):
        print("[shutdown] grab 진행 중이지만 시간 초과 — 강제 진행")

    if arm:
        try:
            print(f"[shutdown] PARK 자세로 이동 (L:{SHUTDOWN_HOME_LEFT}, R:{SHUTDOWN_HOME_RIGHT})")
            arm.move_hands(SHUTDOWN_HOME_LEFT, SHUTDOWN_HOME_RIGHT, None, None, 2.0, 100)
            time.sleep(0.3)

            print("[shutdown] waist 0으로 리셋")
            arm.move_waist_smooth(yaw=0.0, roll=0.0, pitch=0.0, duration=1.5)
            time.sleep(0.3)

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

    print(f"[shutdown] 프로세스 종료 (총 {time.time()-t_shutdown:.2f}초 소요)")
    os._exit(0)


app = FastAPI(title="G1 Box Grab", lifespan=lifespan)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)


# ==========================================
# 좌표 변환
# ==========================================
def camera_to_torso(cx, cy, cz):
    cos_p, sin_p = np.cos(CAMERA_PITCH_URDF), np.sin(CAMERA_PITCH_URDF)
    cx_r  =  cx
    cy_r  =  cy * cos_p + cz * sin_p
    cz_r  = -cy * sin_p + cz * cos_p
    return float(cz_r + CAMERA_X), float(-cx_r + CAMERA_Y), float(-cy_r + CAMERA_Z)


def camera_dir_to_torso(dx, dy, dz):
    cos_p, sin_p = np.cos(CAMERA_PITCH_URDF), np.sin(CAMERA_PITCH_URDF)
    dx_r  =  dx
    dy_r  =  dy * cos_p + dz * sin_p
    dz_r  = -dy * sin_p + dz * cos_p
    return float(dz_r), float(-dx_r), float(-dy_r)


def get_marker_x_axis_in_torso(rvec):
    R, _ = cv2.Rodrigues(rvec)
    x_cam = R[:, 0]
    tx, ty, tz = camera_dir_to_torso(x_cam[0], x_cam[1], x_cam[2])
    v = np.array([tx, ty, tz])
    if np.linalg.norm(v) < 1e-6:
        return np.array([0.0, 1.0, 0.0])
    v[2] = 0.0
    norm_xy = np.linalg.norm(v)
    if norm_xy < 1e-6:
        return np.array([0.0, 1.0, 0.0])
    return v / norm_xy


# ==========================================
# 손목 회전 변환
# ==========================================
def rpy_to_quat(roll_deg, pitch_deg, yaw_deg):
    import pinocchio as pin
    r, p, y = np.radians(roll_deg), np.radians(pitch_deg), np.radians(yaw_deg)
    cr, sr = np.cos(r/2), np.sin(r/2)
    cp, sp = np.cos(p/2), np.sin(p/2)
    cy, sy = np.cos(y/2), np.sin(y/2)
    w = cr*cp*cy + sr*sp*sy
    x = sr*cp*cy - cr*sp*sy
    yq= cr*sp*cy + sr*cp*sy
    z = cr*cp*sy - sr*sp*cy
    return pin.Quaternion(w, x, yq, z).normalized()


# ==========================================
# 3D 박스 그리기
# ==========================================
def draw_box_3d(frame, rvec, tvec):
    box_corners = get_box_corners()
    pts, _ = cv2.projectPoints(box_corners, rvec, tvec, camera_matrix, dist_coeffs)
    pts = pts.reshape(-1, 2).astype(int)

    overlay = frame.copy()
    cv2.fillPoly(overlay, [pts[:4].reshape((-1,1,2))], (0, 255, 0))
    cv2.addWeighted(overlay, 0.25, frame, 0.75, 0, frame)

    TOP_EDGES = {(0,1),(1,2),(2,3),(3,0)}
    for i, j in BOX_EDGES:
        if (i,j) in TOP_EDGES or (j,i) in TOP_EDGES:
            color = (0, 255, 0)
            thickness = 3
        else:
            color = (0, 255, 255)
            thickness = 2
        cv2.line(frame, tuple(pts[i]), tuple(pts[j]), color, thickness)

    for pt in pts[:4]:
        cv2.circle(frame, tuple(pt), 5, (0, 255, 0), -1)


# ==========================================
# ArUco 감지
# ==========================================
def detect_and_draw_aruco(image):
    global latest_markers, latest_marker_pose, marker_last_seen_time

    if camera_matrix is None or image is None:
        return image

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    if aruco_detector:
        corners, ids, _ = aruco_detector.detectMarkers(gray)
    else:
        corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)

    current_markers = []
    best_pose       = None

    if ids is not None:
        for i, marker_id in enumerate(ids.flatten()):
            c  = corners[i][0]
            cx = int(np.mean(c[:, 0]))
            cy = int(np.mean(c[:, 1]))

            ok, rvec, tvec = cv2.solvePnP(
                MARKER_OBJ_PTS, c, camera_matrix, dist_coeffs,
                flags=cv2.SOLVEPNP_IPPE_SQUARE
            )
            if not ok:
                continue

            rvec = rvec.flatten()
            tvec = tvec.flatten()

            # 시각 오버레이는 raw pose 기준 (반응성 유지)
            draw_box_3d(image, rvec, tvec)

            current_markers.append({'id': int(marker_id), 'corners': c, 'cx': cx, 'cy': cy})
            if best_pose is None:
                best_pose = {'id': int(marker_id), 'rvec': rvec, 'tvec': tvec}

    # === 마커 pose 1초 윈도우 중앙값으로 안정화 ===
    if best_pose is not None:
        now = time.time()
        pose_history.append({
            'time': now,
            'id':   best_pose['id'],
            'rvec': best_pose['rvec'].copy(),
            'tvec': best_pose['tvec'].copy(),
        })
        # 1초 이내 + 같은 ID 만 모아서 컴포넌트별 중앙값
        cutoff = now - AVERAGING_WINDOW_SEC
        recent = [p for p in pose_history
                  if p['time'] >= cutoff and p['id'] == best_pose['id']]
        if len(recent) >= AVERAGING_MIN_SAMPLES:
            tvecs = np.array([p['tvec'] for p in recent])
            rvecs = np.array([p['rvec'] for p in recent])
            best_pose = {
                'id':   best_pose['id'],
                'tvec': np.median(tvecs, axis=0),
                'rvec': np.median(rvecs, axis=0),
            }

    with lock:
        latest_markers = current_markers
    with pose_lock:
        if best_pose is not None:
            latest_marker_pose = best_pose
            marker_last_seen_time = time.time()

    return image


def is_marker_visible(threshold_sec=0.5):
    return (time.time() - marker_last_seen_time) < threshold_sec


# ==========================================
# 검출 전용 스레드 (30fps로 detect, latest_marker_pose 갱신)
# ==========================================
DETECT_LOOP_INTERVAL = 1.0 / 30.0   # 약 30fps

def detect_loop():
    """latest_image → detect_and_draw_aruco → latest_annotated 저장.
    웹 송신 FPS와 무관하게 검출 주기 유지."""
    global latest_annotated
    while True:
        loop_start = time.time()

        with image_lock:
            img = None if latest_image is None else latest_image.copy()

        if img is None:
            time.sleep(0.05)
            continue

        # detect + 오버레이 (img가 in-place로 그려짐)
        detect_and_draw_aruco(img)

        with annotated_lock:
            latest_annotated = img

        # 30fps 페이싱
        elapsed = time.time() - loop_start
        if elapsed < DETECT_LOOP_INTERVAL:
            time.sleep(DETECT_LOOP_INTERVAL - elapsed)


# ==========================================
# 비디오 출력 (송신만 — detect는 별도 스레드)
# ==========================================
def generate_frames():
    frame_interval = 1.0 / WEB_STREAM_FPS_MAX
    next_send = 0.0
    while True:
        # 출력 주기 제한 (검출 주기와 독립)
        now = time.time()
        if now < next_send:
            time.sleep(max(0.0, next_send - now))
        next_send = time.time() + frame_interval

        with annotated_lock:
            img = None if latest_annotated is None else latest_annotated.copy()

        if img is None:
            time.sleep(0.05)
            continue

        # 다운스케일 후 인코딩
        small = cv2.resize(img, (WEB_STREAM_WIDTH, WEB_STREAM_HEIGHT),
                           interpolation=cv2.INTER_AREA)

        _, buffer = cv2.imencode('.jpg', small,
                                 [cv2.IMWRITE_JPEG_QUALITY, WEB_STREAM_QUALITY])
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')


# ==========================================
# 로봇 동작
# ==========================================
def robot_move(left_xyz, right_xyz, duration, msg="", left_rot=None, right_rot=None):
    print(f"[IK] {msg}  L:{[f'{v:.3f}' for v in left_xyz]}  R:{[f'{v:.3f}' for v in right_xyz]}")
    if not ROBOT_AVAILABLE or arm is None:
        time.sleep(duration)
        return True
    try:
        arm.move_hands(left_xyz, right_xyz, left_rot, right_rot, duration, 100)
        return True
    except Exception as e:
        print(f"[IK] 오류: {e}")
        return False


def reset_waist():
    print("[WAIST] 0으로 리셋")
    if ROBOT_AVAILABLE and arm is not None:
        arm.move_waist_smooth(yaw=0.0, roll=0.0, pitch=0.0, duration=1.0)
    time.sleep(0.5)


def align_waist_yaw(tvec):
    # 카메라 오프셋(CAMERA_Y) 적용된 torso 좌표 기준으로 yaw 계산
    mx, my, mz = camera_to_torso(tvec[0], tvec[1], tvec[2])
    yaw_deg = float(np.degrees(np.arctan2(my, mx)))
    print(f"[WAIST] yaw: {yaw_deg:.1f}도 (torso my={my:.3f}, mx={mx:.3f})")
    if abs(yaw_deg) < 1.5:
        return
    if ROBOT_AVAILABLE and arm is not None:
        arm.move_waist_smooth(yaw=yaw_deg, roll=0.0, pitch=0.0, duration=1.0)
    else:
        time.sleep(1.0)
    time.sleep(0.5)


def grab_sequence(tvec_orig):
    print("[GRAB] ① waist 리셋")
    reset_waist()

    align_waist_yaw(tvec_orig)

    with pose_lock:
        pose = latest_marker_pose
    if pose is None or not is_marker_visible(threshold_sec=0.5):
        print("[GRAB] 재감지 실패 (마커 안 보임)")
        return False, None, None

    tvec = pose['tvec']
    rvec = pose['rvec']
    mx, my, mz = camera_to_torso(tvec[0], tvec[1], tvec[2])

    box_x_axis = get_marker_x_axis_in_torso(rvec)
    if box_x_axis[1] < 0:
        box_x_axis = -box_x_axis
    print(f"[GRAB] torso: [{mx:.3f}, {my:.3f}, {mz:.3f}], box_x_axis (torso XY): "
          f"[{box_x_axis[0]:+.3f}, {box_x_axis[1]:+.3f}]")

    # 현재 박스 크기 스냅샷
    half_w   = box_size["width"]  / 2
    height_b = box_size["height"]

    grab_x_base = mx + GRAB_X_OFFSET
    grab_z = mz - height_b / 2 + GRAB_Z_OFFSET
    above_z = mz + 0.10
    lift_z = mz + 0.15
    app_off = half_w + GRIP_EXTRA + APPROACH_EXTRA
    grp_off = half_w + GRIP_EXTRA

    grip_dir = box_x_axis

    def offset_point(base_x, base_y, z, offset):
        lx = base_x + grip_dir[0] * offset
        ly = base_y + grip_dir[1] * offset + LEFT_HAND_Y_OFFSET
        rx = base_x - grip_dir[0] * offset
        ry = base_y - grip_dir[1] * offset
        return [lx, ly, z], [rx, ry, z]

    lp = wrist_params['left']
    rp = wrist_params['right']
    l_rot = rpy_to_quat(lp['roll'], lp['pitch'], lp['yaw'])
    r_rot = rpy_to_quat(rp['roll'], rp['pitch'], rp['yaw'])

    L, R = offset_point(mx, my, above_z, app_off)
    if not robot_move(L, R, 1.5, "④ 위쪽 접근", l_rot, r_rot): return False, None, None
    time.sleep(0.2)

    L, R = offset_point(mx, my, grab_z, app_off)
    if not robot_move(L, R, 1.0, "⑤ 측면 하강", l_rot, r_rot): return False, None, None
    time.sleep(0.2)

    L, R = offset_point(grab_x_base, my, grab_z, grp_off)
    if not robot_move(L, R, 2.5, "⑥ 잡기", l_rot, r_rot): return False, None, None
    time.sleep(1.0)

    # ⑥ 잡기 완료 멘트
    speak(MSG_PICKED)

    sym_L = [grab_x_base, +grp_off + LEFT_HAND_Y_OFFSET, grab_z]
    sym_R = [grab_x_base, -grp_off, grab_z]
    if not robot_move(sym_L, sym_R, 1.5, "⑥' 좌우 대칭 정렬", l_rot, r_rot): return False, None, None
    time.sleep(0.3)

    ll = [grab_x_base, +grp_off + LEFT_HAND_Y_OFFSET, lift_z]
    rl = [grab_x_base, -grp_off, lift_z]
    if not robot_move(ll, rl, 1.5, "⑦ 들기", l_rot, r_rot): return False, None, None
    time.sleep(0.2)

    # ⑦' 허리 회전 — handover 방향에 따라
    if HANDOVER_DIRECTION == "left":
        handover_yaw = +HANDOVER_YAW_DEG
    elif HANDOVER_DIRECTION == "right":
        handover_yaw = -HANDOVER_YAW_DEG
    else:
        handover_yaw = 0.0
    print(f"[GRAB] ⑦' 허리 yaw → {handover_yaw:.1f}도 (handover: {HANDOVER_DIRECTION})")
    if ROBOT_AVAILABLE and arm is not None:
        arm.move_waist_smooth(yaw=handover_yaw, roll=0.0, pitch=0.0, duration=1.5)
        time.sleep(0.5)

    hl = [HANDOVER_X, +grp_off + LEFT_HAND_Y_OFFSET, lift_z]
    hr = [HANDOVER_X, -grp_off, lift_z]
    if not robot_move(hl, hr, 1.5, "⑧ 건네기", l_rot, r_rot): return False, None, None
    time.sleep(0.3)

    # ⑧ 건네기 완료 → 사용자 행동 유도 멘트
    speak(MSG_HANDOVER)

    print("[HANDOVER] 10초 대기 (마커 가려지면 받음, 안 가려지면 내려놓음)")
    start = time.time()
    received = False
    while time.time() - start < 10.0:
        if not is_marker_visible(threshold_sec=0.5):
            received = True
            print(f"[HANDOVER] 마커 가려짐 → 받음 감지 ({time.time()-start:.1f}초)")
            break
        time.sleep(0.1)

    if received:
        speak(MSG_RECEIVED)
        robot_move([HANDOVER_X, +grp_off+0.10, lift_z],
                   [HANDOVER_X, -grp_off-0.10, lift_z],
                   1.0, "⑩ 손 벌림 (수령)", l_rot, r_rot)
    else:
        print("[HANDOVER] 타임아웃 → 박스 내려놓기")
        speak(MSG_TIMEOUT)
        robot_move([HANDOVER_X, +grp_off, grab_z],
                   [HANDOVER_X, -grp_off, grab_z],
                   1.5, "⑩a 내려놓기", l_rot, r_rot)
        robot_move([HANDOVER_X, +grp_off+0.10, grab_z],
                   [HANDOVER_X, -grp_off-0.10, grab_z],
                   1.0, "⑩b 손 벌림", l_rot, r_rot)
        robot_move([HANDOVER_X, +grp_off+0.10, lift_z],
                   [HANDOVER_X, -grp_off-0.10, lift_z],
                   1.0, "⑩c 위로 후퇴", l_rot, r_rot)

    # ==========================================
    # 마무리: 중앙 복귀 → 홈
    # ==========================================
    print("[HANDOVER] ⑪ 허리 중앙 복귀")
    reset_waist()
    print("[HANDOVER] ⑪ 홈 자세 복귀")
    robot_move(HOME_LEFT, HOME_RIGHT, 2.0, "⑪ Home")

    # ⑪ 홈 복귀 완료 멘트
    speak(MSG_HOME)

    return False, None, None


# ==========================================
# HTML
# ==========================================
HTML_PAGE = """<!DOCTYPE html>
<html>
<head>
    <title>G1 Box Grab</title>
    <style>
        * { box-sizing: border-box; margin: 0; padding: 0; }
        body { font-family: monospace; background: #1a1a1a; color: #fff; padding: 24px; }
        #wrap { max-width: 1000px; margin: 0 auto; }
        h1   { color: #4CAF50; margin-bottom: 4px; font-size: 22px; }
        .sub { color: #666; font-size: 13px; margin-bottom: 24px; }
        #layout { display: flex; gap: 24px; align-items: flex-start; justify-content: center; }
        #stream  { border: 2px solid #4CAF50; display: block; }

        #panel {
            background: #242424; border-radius: 10px; padding: 20px;
            width: 300px; display: flex; flex-direction: column; gap: 16px;
        }

        .card { background: #1e1e1e; border-radius: 8px; padding: 14px; }
        .card-title { color: #4CAF50; font-size: 12px; text-transform: uppercase;
                      letter-spacing: 1px; margin-bottom: 10px; }

        #marker-status { font-size: 14px; color: #555; }
        #marker-status.found { color: #4CAF50; }

        .info-row { display: flex; justify-content: space-between; font-size: 13px; margin: 4px 0; }
        .info-key { color: #555; }
        .info-val { color: #ccc; }

        .btn-group { display: flex; gap: 8px; }
        .btn {
            flex: 1; padding: 13px 0; border: none; border-radius: 6px;
            font-size: 14px; font-weight: bold; cursor: pointer; transition: opacity .15s;
        }
        .btn:disabled { opacity: 0.35; cursor: not-allowed; }
        .btn-start   { background: #4CAF50; color: #000; }
        .btn-release { background: #2196F3; color: #fff; }
        .btn-home    { background: #FF9800; color: #000; }

        #status-bar {
            border-radius: 6px; padding: 10px 14px; font-size: 13px;
            background: #2a2a2a; color: #666;
        }
        .s-running { background: #1a2a1a !important; color: #4CAF50 !important; }
        .s-holding { background: #1a1a2a !important; color: #64B5F6 !important; }
        .s-error   { background: #2a1a1a !important; color: #ef5350 !important; }
        .rpy-row   { display:flex; align-items:center; margin:3px 0; }
        .rpy-label { color:#555; font-size:12px; width:16px; }
        .rpy-input { background:#333; border:1px solid #444; color:#fff; padding:4px 6px;
                     border-radius:4px; width:70px; font-size:13px; }
        .btn-apply { background:#555; color:#fff; border:none; border-radius:4px; cursor:pointer; font-size:13px; }
        .btn-apply:hover { background:#666; }
    </style>
</head>
<body>
    <div id="wrap">
    <h1>G1 Box Grab</h1>
    <p class="sub">원격 MJPEG → 마커 감지 → 가로면 잡기 → 정면 대칭 → 건네기</p>

    <div id="layout">
        <img id="stream" src="/video_feed" width="640" height="480" style="image-rendering:auto;">

        <div id="panel">
            <div class="card">
                <div class="card-title">마커 감지</div>
                <div id="marker-status">대기 중...</div>
            </div>

            <div class="card">
                <div class="card-title">박스 위치 (torso)</div>
                <div class="info-row"><span class="info-key">X 전방</span><span class="info-val" id="tx">-</span></div>
                <div class="info-row"><span class="info-key">Y 좌우</span><span class="info-val" id="ty">-</span></div>
                <div class="info-row"><span class="info-key">Z 높이</span><span class="info-val" id="tz">-</span></div>
                <div class="info-row"><span class="info-key">Yaw 목표</span><span class="info-val" id="yaw">-</span></div>
            </div>

            <div class="card">
                <div class="card-title">손목 RPY (도)</div>
                <div style="display:grid; grid-template-columns:1fr 1fr; gap:8px;">
                    <div>
                        <div style="color:#4CAF50; font-size:11px; margin-bottom:4px;">왼손 L</div>
                        <div class="rpy-row"><span class="rpy-label">R</span><input class="rpy-input" id="l-roll"  type="number" value="-10" step="5"></div>
                        <div class="rpy-row"><span class="rpy-label">P</span><input class="rpy-input" id="l-pitch" type="number" value="-10"   step="5"></div>
                        <div class="rpy-row"><span class="rpy-label">Y</span><input class="rpy-input" id="l-yaw"   type="number" value="-15" step="5"></div>
                    </div>
                    <div>
                        <div style="color:#FF9800; font-size:11px; margin-bottom:4px;">오른손 R</div>
                        <div class="rpy-row"><span class="rpy-label">R</span><input class="rpy-input" id="r-roll"  type="number" value="10" step="5"></div>
                        <div class="rpy-row"><span class="rpy-label">P</span><input class="rpy-input" id="r-pitch" type="number" value="-10"  step="5"></div>
                        <div class="rpy-row"><span class="rpy-label">Y</span><input class="rpy-input" id="r-yaw"   type="number" value="15" step="5"></div>
                    </div>
                </div>
                <button class="btn btn-apply" onclick="applyWrist()" style="margin-top:10px; width:100%; padding:8px;">적용</button>
                <div id="wrist-msg" style="font-size:11px; color:#666; margin-top:6px;"></div>
            </div>

            <div class="card">
                <div class="card-title">자동 모드</div>
                <label style="display:flex; align-items:center; gap:8px; margin-bottom:10px;">
                    <input type="checkbox" id="auto-enabled" onchange="toggleAuto()" style="width:18px; height:18px;">
                    <span id="auto-label" style="color:#888;">OFF</span>
                </label>
                <div style="display:grid; grid-template-columns:1fr 1fr; gap:6px; font-size:11px; color:#666;">
                    <div>X min<input class="rpy-input" id="ax-min" type="number" value="0.30" step="0.05" style="width:100%"></div>
                    <div>X max<input class="rpy-input" id="ax-max" type="number" value="0.40" step="0.05" style="width:100%"></div>
                    <div>Y min<input class="rpy-input" id="ay-min" type="number" value="-0.15" step="0.05" style="width:100%"></div>
                    <div>Y max<input class="rpy-input" id="ay-max" type="number" value="0.15" step="0.05" style="width:100%"></div>
                    <div>Z min<input class="rpy-input" id="az-min" type="number" value="-0.10" step="0.05" style="width:100%"></div>
                    <div>Z max<input class="rpy-input" id="az-max" type="number" value="0.20" step="0.05" style="width:100%"></div>
                </div>
                <button class="btn btn-apply" onclick="applyAutoZone()" style="margin-top:8px; width:100%; padding:6px; font-size:11px;">영역 적용</button>
                <div id="auto-progress" style="margin-top:8px; height:6px; background:#333; border-radius:3px; overflow:hidden;">
                    <div id="auto-bar" style="height:100%; width:0%; background:#4CAF50; transition: width .15s;"></div>
                </div>
                <div id="auto-msg" style="font-size:11px; color:#666; margin-top:6px;">대기</div>
            </div>

            <div class="card">
                <div class="card-title">박스 크기 (cm)</div>
                <div style="display:grid; grid-template-columns:1fr 1fr 1fr; gap:6px; font-size:11px; color:#666;">
                    <div>W<input class="rpy-input" id="bx-w" type="number" value="28" step="1" style="width:100%"></div>
                    <div>D<input class="rpy-input" id="bx-d" type="number" value="9"  step="1" style="width:100%"></div>
                    <div>H<input class="rpy-input" id="bx-h" type="number" value="9"  step="1" style="width:100%"></div>
                </div>
                <button class="btn btn-apply" onclick="applyBoxSize()" style="margin-top:8px; width:100%; padding:6px; font-size:11px;">크기 적용</button>
                <div id="bx-msg" style="font-size:11px; color:#666; margin-top:6px;">현재: 28 × 9 × 9</div>
            </div>

            <div class="card">
                <div class="card-title">건네기 방향</div>
                <div style="display:grid; grid-template-columns:1fr 1fr 1fr; gap:6px;">
                    <button class="btn btn-apply" id="ho-left"   onclick="setHandover('left')"   style="padding:8px; font-size:12px;">◀ 왼쪽</button>
                    <button class="btn btn-apply" id="ho-center" onclick="setHandover('center')" style="padding:8px; font-size:12px;">■ 중앙</button>
                    <button class="btn btn-apply" id="ho-right"  onclick="setHandover('right')"  style="padding:8px; font-size:12px;">오른쪽 ▶</button>
                </div>
                <div style="display:flex; align-items:center; gap:8px; margin-top:8px; font-size:11px; color:#666;">
                    <span>Yaw</span>
                    <input class="rpy-input" id="ho-yaw" type="number" value="30" step="5" style="width:60px;">
                    <span>도</span>
                </div>
                <div id="ho-msg" style="font-size:11px; color:#666; margin-top:6px;">현재: 중앙</div>
            </div>

            <div id="status-bar">대기 중</div>

            <div class="btn-group">
                <button class="btn btn-start"   id="btn-start"   onclick="startGrab()">▶ Start</button>
                <button class="btn btn-release" id="btn-release" onclick="doRelease()" disabled>↔ Release</button>
                <button class="btn btn-home"    id="btn-home"    onclick="goHome()">⌂ Home</button>
            </div>
        </div>
    </div>

    <script>
        function poll() {
            fetch('/status').then(r => r.json()).then(d => {
                const ms = document.getElementById('marker-status');
                ms.className   = d.marker_found ? 'found' : '';
                ms.textContent = d.marker_found
                    ? `ID: ${d.marker_id}  감지됨 ✓`
                    : '대기 중...';

                if (d.torso) {
                    document.getElementById('tx').textContent  = d.torso.x.toFixed(3) + ' m';
                    document.getElementById('ty').textContent  = d.torso.y.toFixed(3) + ' m';
                    document.getElementById('tz').textContent  = d.torso.z.toFixed(3) + ' m';
                    document.getElementById('yaw').textContent = d.yaw_deg.toFixed(1) + '°';
                }

                document.getElementById('btn-start').disabled   = d.busy;
                document.getElementById('btn-home').disabled    = d.busy;
                document.getElementById('btn-release').disabled = d.busy || !d.grab_active;

                if (d.busy) setSt('동작 중...', 'running');
                else if (d.grab_active) setSt('박스 들고 있음', 'holding');
                else setSt('대기 중', '');

                const cb = document.getElementById('auto-enabled');
                if (cb.checked !== d.auto_enabled) cb.checked = d.auto_enabled;
                document.getElementById('auto-label').textContent = d.auto_enabled ? 'ON' : 'OFF';
                document.getElementById('auto-label').style.color = d.auto_enabled ? '#4CAF50' : '#888';

                const pct = d.auto_dwell > 0
                    ? Math.min(100, (d.auto_elapsed / d.auto_dwell) * 100)
                    : 0;
                document.getElementById('auto-bar').style.width = pct + '%';

                if (!d.auto_enabled) {
                    document.getElementById('auto-msg').textContent = '대기 (OFF)';
                } else if (d.busy) {
                    document.getElementById('auto-msg').textContent = '잡기 동작 중';
                } else if (d.auto_in_zone) {
                    document.getElementById('auto-msg').textContent =
                        `영역 안 ${d.auto_elapsed.toFixed(1)}/${d.auto_dwell.toFixed(1)}초`;
                } else {
                    document.getElementById('auto-msg').textContent = '마커 영역 밖';
                }
            }).catch(() => {});
        }
        setInterval(poll, 500);

        function toggleAuto() {
            const enabled = document.getElementById('auto-enabled').checked;
            fetch('/set_auto_mode?enabled=' + enabled).then(r => r.json());
        }
        function applyAutoZone() {
            const q = [
                'x_min=' + document.getElementById('ax-min').value,
                'x_max=' + document.getElementById('ax-max').value,
                'y_min=' + document.getElementById('ay-min').value,
                'y_max=' + document.getElementById('ay-max').value,
                'z_min=' + document.getElementById('az-min').value,
                'z_max=' + document.getElementById('az-max').value,
            ].join('&');
            fetch('/set_auto_mode?' + q).then(r => r.json()).then(d => {
                document.getElementById('auto-msg').textContent = '영역 적용됨';
            });
        }

        function startGrab() {
            setBtns(true);
            fetch('/start_grab').then(r => r.json()).then(d => {
                if (!d.success) setSt('실패: ' + d.error, 'error');
            }).catch(e => setSt('오류: ' + e, 'error'));
        }
        function doRelease() {
            setBtns(true);
            fetch('/release').then(r => r.json()).then(d => {
                if (!d.success) setSt('실패: ' + d.error, 'error');
            }).catch(e => setSt('오류: ' + e, 'error'));
        }
        function goHome() {
            setBtns(true);
            fetch('/go_home').then(r => r.json()).then(d => {
                if (!d.success) setSt('홈 실패', 'error');
            }).catch(e => setSt('오류: ' + e, 'error'));
        }
        function applyWrist() {
            const params = {
                l_roll:  parseFloat(document.getElementById('l-roll').value)  || 0,
                l_pitch: parseFloat(document.getElementById('l-pitch').value) || 0,
                l_yaw:   parseFloat(document.getElementById('l-yaw').value)   || 0,
                r_roll:  parseFloat(document.getElementById('r-roll').value)  || 0,
                r_pitch: parseFloat(document.getElementById('r-pitch').value) || 0,
                r_yaw:   parseFloat(document.getElementById('r-yaw').value)   || 0,
            };
            const q = Object.entries(params).map(([k,v]) => `${k}=${v}`).join('&');
            fetch('/set_wrist?' + q).then(r => r.json()).then(d => {
                document.getElementById('wrist-msg').textContent = d.success ? '적용됨 ✓' : '실패';
                document.getElementById('wrist-msg').style.color = d.success ? '#4CAF50' : '#ef5350';
            });
        }
        function applyBoxSize() {
            const w = parseFloat(document.getElementById('bx-w').value) / 100;
            const d = parseFloat(document.getElementById('bx-d').value) / 100;
            const h = parseFloat(document.getElementById('bx-h').value) / 100;
            fetch(`/set_box_size?width=${w}&depth=${d}&height=${h}`)
                .then(r => r.json()).then(res => {
                    if (res.success) {
                        const s = res.box_size;
                        const wcm = Math.round(s.width  * 100);
                        const dcm = Math.round(s.depth  * 100);
                        const hcm = Math.round(s.height * 100);
                        const msg = document.getElementById('bx-msg');
                        msg.textContent = `현재: ${wcm} × ${dcm} × ${hcm}`;
                        msg.style.color = '#4CAF50';
                    }
                });
        }
        function setHandover(direction) {
            const yaw = parseFloat(document.getElementById('ho-yaw').value) || 30;
            fetch(`/set_handover_direction?direction=${direction}&yaw_deg=${yaw}`)
                .then(r => r.json()).then(d => {
                    if (d.success) {
                        const label = {left: '왼쪽', center: '중앙', right: '오른쪽'}[d.direction];
                        document.getElementById('ho-msg').textContent =
                            `현재: ${label}` + (d.direction !== 'center' ? ` (${d.yaw_deg}도)` : '');
                        document.getElementById('ho-msg').style.color = '#4CAF50';
                        ['left','center','right'].forEach(k => {
                            const b = document.getElementById('ho-' + k);
                            b.style.background = (k === d.direction) ? '#4CAF50' : '';
                            b.style.color      = (k === d.direction) ? '#000' : '';
                        });
                    }
                });
        }
        function setSt(msg, cls) {
            const el = document.getElementById('status-bar');
            el.textContent = msg;
            el.className   = cls ? `s-${cls}` : '';
        }
        function setBtns(disabled) {
            document.getElementById('btn-start').disabled = disabled;
            document.getElementById('btn-home').disabled  = disabled;
        }
    </script>
</body>
</html>"""


# ==========================================
# 엔드포인트
# ==========================================
@app.get("/", response_class=HTMLResponse)
async def index():
    return HTML_PAGE


@app.get("/video_feed")
async def video_feed():
    return StreamingResponse(generate_frames(),
                             media_type="multipart/x-mixed-replace; boundary=frame")


@app.get("/status")
async def status():
    with pose_lock:
        pose = latest_marker_pose

    visible = pose is not None and is_marker_visible(threshold_sec=0.5)

    torso = None
    yaw_deg = 0.0
    if visible:
        tvec = pose['tvec']
        mx, my, mz = camera_to_torso(tvec[0], tvec[1], tvec[2])
        print(f"[DEBUG] cam tvec=[{tvec[0]:+.3f},{tvec[1]:+.3f},{tvec[2]:+.3f}]  torso=[{mx:+.3f},{my:+.3f},{mz:+.3f}]")
        torso   = {"x": round(mx,3), "y": round(my,3), "z": round(mz,3)}
        yaw_deg = float(np.degrees(np.arctan2(my, mx)))

    in_zone_since = auto_state.get("in_zone_since")
    elapsed = (time.time() - in_zone_since) if in_zone_since else 0.0

    return {
        "grab_active":    grab_state['active'],
        "busy":           grab_state.get('busy', False),
        "marker_found":   visible,
        "marker_id":      pose['id'] if visible else None,
        "torso":          torso,
        "yaw_deg":        round(yaw_deg, 1),
        "stream_started": stream_started,
        "auto_enabled":   auto_mode["enabled"],
        "auto_in_zone":   in_zone_since is not None,
        "auto_elapsed":   round(elapsed, 2),
        "auto_dwell":     auto_mode["dwell_sec"],
    }


def launch_grab(tvec):
    if grab_state.get('busy'):
        return False

    def run():
        global grab_state
        grab_state['busy'] = True
        auto_state['in_zone_since'] = None
        try:
            ok, ll, rl = grab_sequence(tvec)
            grab_state['active']      = ok
            grab_state['lifted_left'] = ll
            grab_state['lifted_right']= rl
            if ok:
                print("[GRAB] 완료 (들고 있음)")
            elif ll is None and rl is None:
                print("[GRAB] 자동 완료")
            else:
                print("[GRAB] 실패")
        finally:
            grab_state['busy'] = False
            auto_state['in_zone_since'] = None

    threading.Thread(target=run, daemon=True).start()
    return True


@app.get("/start_grab")
async def start_grab():
    print("[START] 마커 탐색 (최대 3초)...")
    deadline = time.time() + 3.0
    pose = None
    while time.time() < deadline:
        with pose_lock:
            p = latest_marker_pose
        if p is not None and is_marker_visible(threshold_sec=0.3):
            pose = p
            break
        time.sleep(0.1)

    if pose is None:
        print("[START] 마커 미발견 (타임아웃)")
        return JSONResponse({"success": False, "error": "마커가 감지되지 않았습니다 (3초 대기)"})

    print(f"[START] 마커 발견: ID {pose['id']}")
    speak(MSG_TRIGGER)
    if not launch_grab(pose['tvec']):
        return JSONResponse({"success": False, "error": "이미 동작 중입니다"})
    return JSONResponse({"success": True, "marker_id": pose['id']})


@app.get("/release")
async def release():
    global grab_state

    if not grab_state['active']:
        return JSONResponse({"success": False, "error": "박스를 잡고 있지 않습니다"})

    ll = grab_state['lifted_left']
    rl = grab_state['lifted_right']

    def run():
        global grab_state
        grab_state['busy'] = True
        try:
            robot_move([ll[0], ll[1]+0.10, ll[2]],
                       [rl[0], rl[1]-0.10, rl[2]],
                       1.5, "Release")
            grab_state['active'] = False
        finally:
            grab_state['busy'] = False

    threading.Thread(target=run, daemon=True).start()
    return JSONResponse({"success": True})


@app.get("/go_home")
async def go_home():
    global grab_state

    def run():
        global grab_state
        grab_state['busy'] = True
        try:
            reset_waist()
            robot_move(HOME_LEFT, HOME_RIGHT, 2.0, "Home")
            grab_state['active'] = False
        finally:
            grab_state['busy'] = False

    threading.Thread(target=run, daemon=True).start()
    return JSONResponse({"success": True})


@app.get("/set_wrist")
async def set_wrist(
    l_roll: float=0, l_pitch: float=0, l_yaw: float=0,
    r_roll: float=0, r_pitch: float=0, r_yaw: float=0
):
    global wrist_params
    wrist_params = {
        'left':  {'roll': l_roll, 'pitch': l_pitch, 'yaw': l_yaw},
        'right': {'roll': r_roll, 'pitch': r_pitch, 'yaw': r_yaw},
    }
    print(f"[WRIST] L: R={l_roll} P={l_pitch} Y={l_yaw}  R: R={r_roll} P={r_pitch} Y={r_yaw}")
    return {"success": True, "wrist_params": wrist_params}


@app.get("/set_handover_direction")
async def set_handover_direction(direction: str = "center", yaw_deg: float = None):
    """건네기 방향 변경: center / left / right"""
    global HANDOVER_DIRECTION, HANDOVER_YAW_DEG
    if direction not in ("center", "left", "right"):
        return {"success": False, "error": f"invalid direction: {direction}"}
    HANDOVER_DIRECTION = direction
    if yaw_deg is not None:
        HANDOVER_YAW_DEG = float(yaw_deg)
    print(f"[HANDOVER] 방향={HANDOVER_DIRECTION}, yaw={HANDOVER_YAW_DEG}도")
    return {"success": True, "direction": HANDOVER_DIRECTION, "yaw_deg": HANDOVER_YAW_DEG}


@app.get("/set_box_size")
async def set_box_size(width: float = None, depth: float = None, height: float = None):
    """박스 크기 변경 (단위: m). None이면 해당 항목 유지."""
    global box_size
    if width  is not None: box_size["width"]  = float(width)
    if depth  is not None: box_size["depth"]  = float(depth)
    if height is not None: box_size["height"] = float(height)
    print(f"[BOX] 크기 변경: W={box_size['width']:.3f}, D={box_size['depth']:.3f}, H={box_size['height']:.3f}")
    return {"success": True, "box_size": box_size}


@app.get("/box_size")
async def get_box_size():
    return {"box_size": box_size}


@app.get("/auto_mode")
async def get_auto_mode():
    in_zone_since = auto_state.get("in_zone_since")
    elapsed = (time.time() - in_zone_since) if in_zone_since else 0.0
    return {
        "config": auto_mode,
        "in_zone": in_zone_since is not None,
        "elapsed_sec": round(elapsed, 2),
    }


@app.get("/set_auto_mode")
async def set_auto_mode(
    enabled: bool = None,
    x_min: float = None, x_max: float = None,
    y_min: float = None, y_max: float = None,
    z_min: float = None, z_max: float = None,
    dwell_sec: float = None,
):
    global auto_mode
    for k, v in [("enabled", enabled),
                 ("x_min", x_min), ("x_max", x_max),
                 ("y_min", y_min), ("y_max", y_max),
                 ("z_min", z_min), ("z_max", z_max),
                 ("dwell_sec", dwell_sec)]:
        if v is not None:
            auto_mode[k] = v
    auto_state["in_zone_since"] = None
    print(f"[AUTO] 설정 변경: {auto_mode}")
    return {"success": True, "config": auto_mode}


if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=50000, timeout_graceful_shutdown=2)
