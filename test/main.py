#!/usr/bin/env python3
# Version: 1.8
"""
RealSense D435i + YOLO 박스 검출 + IMU + 마스크 시각화.

v0.4:
  - 마스크 화면 별도 스트림 추가 (/mask_feed)
  - 메인 화면 옆에 마스크 비교 표시
"""
import os
import sys
import time
import threading
import numpy as np
import cv2
import pyrealsense2 as rs
import uvicorn
from contextlib import asynccontextmanager
from fastapi import FastAPI
from fastapi.responses import HTMLResponse, StreamingResponse, JSONResponse
from fastapi.middleware.cors import CORSMiddleware

from box_estimator import BoxEstimator, draw_box_overlay, draw_gravity_overlay


# ==========================================
# 설정
# ==========================================
PORT = 50010

COLOR_W, COLOR_H = 640, 480
FPS              = 30

YOLO_MODEL  = "box.pt"
YOLO_CONF   = 0.4
YOLO_DEVICE = "cpu"

DETECT_FPS      = 5
DETECT_INTERVAL = 1.0 / DETECT_FPS

STREAM_FPS     = 15
STREAM_QUALITY = 80

# === 카메라 tilt 고정 (G1 가슴 카메라) ===
# 카메라가 정면에서 아래로 기울어진 각도
CAM_TILT_DEG = 47.6

# 카메라 좌표계에서 중력 벡터 (월드 좌표계 [0, 1, 0] = 아래)
# 카메라가 X축 기준으로 47.6도 아래로 기울어짐
# gravity_cam = [0, cos(tilt), sin(tilt)]
_t = np.radians(CAM_TILT_DEG)
GRAVITY_CAM = np.array([0.0, np.cos(_t), np.sin(_t)], dtype=np.float64)


# ==========================================
# 전역 상태
# ==========================================
camera_K     = None
depth_scale  = None
pipeline     = None
align        = None
stop_flag    = threading.Event()

latest_color     = None
latest_depth     = None
latest_annotated = None    # OBB 오버레이 화면
latest_mask_view = None    # 마스크 시각화 화면 (NEW)
latest_result    = None
latest_gravity   = None

color_lock      = threading.Lock()
depth_lock      = threading.Lock()
annotated_lock  = threading.Lock()
mask_view_lock  = threading.Lock()
result_lock     = threading.Lock()
gravity_lock    = threading.Lock()

box_estimator = None


# ==========================================
# 슬라이딩 윈도우 평균 (좌표 안정화)
# ==========================================
from collections import deque

# 슬라이딩 윈도우 평균 (안정화)
# - 짧을수록 반응 빠름, 길수록 안정적
# - 시연 (박스 고정): 2~3초 추천
# - 트래킹 (박스 이동): 0.5~1초 추천
SMOOTH_WINDOW_SEC = 2.0

class CoordSmoother:
    """3D 좌표 또는 scalar 값의 시간 기반 슬라이딩 평균."""
    def __init__(self, window_sec=1.0):
        self.window_sec = window_sec
        self.buf = deque()   # (timestamp, value)

    def push(self, value):
        import time
        now = time.time()
        self.buf.append((now, np.asarray(value, dtype=np.float64)))
        while self.buf and now - self.buf[0][0] > self.window_sec:
            self.buf.popleft()

    def get(self):
        if not self.buf:
            return None
        vals = np.stack([v for _, v in self.buf])
        return vals.mean(axis=0)

    def get_median(self):
        """중앙값 (튀는 값에 더 강함)."""
        if not self.buf:
            return None
        vals = np.stack([v for _, v in self.buf])
        return np.median(vals, axis=0)

    def count(self):
        return len(self.buf)

# 키별 smoother
smoothers = {
    'center':     CoordSmoother(SMOOTH_WINDOW_SEC),
    'top_center': CoordSmoother(SMOOTH_WINDOW_SEC),
    'L':          CoordSmoother(SMOOTH_WINDOW_SEC),
    'R':          CoordSmoother(SMOOTH_WINDOW_SEC),
    'TL':         CoordSmoother(SMOOTH_WINDOW_SEC),
    'TR':         CoordSmoother(SMOOTH_WINDOW_SEC),
    'BL':         CoordSmoother(SMOOTH_WINDOW_SEC),
    'BR':         CoordSmoother(SMOOTH_WINDOW_SEC),
    # 박스 사이즈는 안정화된 꼭지점에서 계산하므로 따로 안 둠
    # H는 h_top/h_table 평면 mode 자체가 안정적이므로 그대로
    'box_H':      CoordSmoother(SMOOTH_WINDOW_SEC),
}
smoother_lock = threading.Lock()


def update_smoothers(result):
    """detect 결과를 smoother에 push."""
    if result is None:
        return
    with smoother_lock:
        if 'center_3d' in result:
            smoothers['center'].push(result['center_3d'])
        if 'top_center_3d' in result:
            smoothers['top_center'].push(result['top_center_3d'])
        if 'top_mids_3d' in result:
            m = result['top_mids_3d']
            if m.get('L') is not None:
                smoothers['L'].push(m['L'])
            if m.get('R') is not None:
                smoothers['R'].push(m['R'])
        if 'top_corners_3d' in result:
            c = result['top_corners_3d']
            for k in ['TL', 'TR', 'BL', 'BR']:
                if c.get(k) is not None:
                    smoothers[k].push(c[k])
        # 박스 H만 push (W, D는 안정화된 꼭지점에서 직접 계산)
        if 'box_H_m' in result:
            smoothers['box_H'].push(result['box_H_m'])


def get_smoothed():
    """평균값 dict 반환 (median 사용)."""
    out = {}
    with smoother_lock:
        for k, sm in smoothers.items():
            v = sm.get_median()
            if v is not None:
                out[k] = v
        out['_count'] = max((sm.count() for sm in smoothers.values()),
                              default=0)
    return out


# ==========================================
# 카메라 초기화
# ==========================================
def init_camera():
    global pipeline, align, camera_K, depth_scale

    pipeline = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.color, COLOR_W, COLOR_H, rs.format.bgr8, FPS)
    cfg.enable_stream(rs.stream.depth, COLOR_W, COLOR_H, rs.format.z16,  FPS)

    print("[CALIB] RealSense 시작 중...")
    profile = pipeline.start(cfg)
    align = rs.align(rs.stream.color)

    device = profile.get_device()
    depth_sensor = device.first_depth_sensor()

    if depth_sensor.supports(rs.option.visual_preset):
        depth_sensor.set_option(rs.option.visual_preset, 4)
        print("[CALIB] Visual preset: High Density")

    if depth_sensor.supports(rs.option.laser_power):
        max_p = depth_sensor.get_option_range(rs.option.laser_power).max
        depth_sensor.set_option(rs.option.laser_power, max_p)
        print(f"[CALIB] Laser power: {max_p}")

    color_stream = profile.get_stream(rs.stream.color)
    intr = color_stream.as_video_stream_profile().get_intrinsics()
    camera_K = np.array([
        [intr.fx, 0,       intr.ppx],
        [0,       intr.fy, intr.ppy],
        [0,       0,       1       ]
    ], dtype=np.float32)
    depth_scale = depth_sensor.get_depth_scale()

    print("=" * 60)
    print(f"[CALIB] {intr.width}x{intr.height} @ {FPS}fps")
    print(f"  fx={intr.fx:.4f}  fy={intr.fy:.4f}")
    print(f"  ppx={intr.ppx:.4f}  ppy={intr.ppy:.4f}")
    print(f"  CAM_TILT (fixed): {CAM_TILT_DEG}° "
          f"-> gravity_cam={GRAVITY_CAM}")
    print("=" * 60)

    for _ in range(15):
        pipeline.wait_for_frames()
    print("[CALIB] 카메라 준비 완료\n")


# ==========================================
# 캡처 스레드
# ==========================================
def capture_loop():
    global latest_color, latest_depth, latest_gravity

    depth_to_disp = rs.disparity_transform(True)
    disp_to_depth = rs.disparity_transform(False)
    spatial       = rs.spatial_filter()
    temporal      = rs.temporal_filter()
    hole_filling  = rs.hole_filling_filter(1)

    spatial.set_option(rs.option.filter_magnitude, 2)
    spatial.set_option(rs.option.filter_smooth_alpha, 0.5)
    spatial.set_option(rs.option.filter_smooth_delta, 20)
    spatial.set_option(rs.option.holes_fill, 3)

    # 카메라 tilt 고정 → gravity_cam 고정값
    with gravity_lock:
        latest_gravity = GRAVITY_CAM.copy()

    while not stop_flag.is_set():
        try:
            frames = pipeline.wait_for_frames(timeout_ms=2000)
        except Exception:
            continue

        aligned = align.process(frames)
        cf = aligned.get_color_frame()
        df = aligned.get_depth_frame()
        if not cf or not df:
            continue

        df = depth_to_disp.process(df)
        df = spatial.process(df)
        df = temporal.process(df)
        df = disp_to_depth.process(df)
        df = hole_filling.process(df)

        color = np.asanyarray(cf.get_data())
        depth = np.asanyarray(df.get_data())

        with color_lock:
            latest_color = color.copy()
        with depth_lock:
            latest_depth = depth.copy()


# ==========================================
# 마스크 시각화 생성
# ==========================================
def make_mask_view(color, result, K=None):
    """seg mask + 윗면 + 4 꼭지점 + 좌/우 중심 (OBB와 동일)."""
    if result is None or 'mask' not in result:
        view = color.copy()
        cv2.putText(view, "No mask", (20, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (100, 100, 255), 2)
        return view

    mask = result['mask']
    H, W = color.shape[:2]
    view = color.copy()

    # 전체 mask = 옅은 빨강
    mask_color = np.zeros_like(color)
    mask_color[mask] = (100, 100, 255)
    cv2.addWeighted(mask_color, 0.25, view, 1.0, 0, view)

    # 윗면 = 파랑
    if 'top_pixel_mask' in result:
        top = result['top_pixel_mask']
        top_color = np.zeros_like(color)
        top_color[top] = (255, 150, 0)
        cv2.addWeighted(top_color, 0.5, view, 1.0, 0, view)

    # mask 외곽선 (빨강)
    mask_u8 = (mask.astype(np.uint8)) * 255
    contours, _ = cv2.findContours(mask_u8, cv2.RETR_EXTERNAL,
                                    cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(view, contours, -1, (0, 100, 255), 2)

    if K is None:
        return view

    rvec0 = np.zeros(3, dtype=np.float32)
    tvec0 = np.zeros(3, dtype=np.float32)
    dist  = np.zeros(5, dtype=np.float32)

    # 윗면 4 꼭지점 + 변
    if 'top_corners_2d' in result:
        corners = result['top_corners_2d']
        TL, TR = corners['TL'].astype(int), corners['TR'].astype(int)
        BL, BR = corners['BL'].astype(int), corners['BR'].astype(int)
        pts = np.array([TL, TR, BR, BL])
        cv2.polylines(view, [pts], True, (0, 255, 0), 1)
        for label, pt in [('TL', TL), ('TR', TR), ('BL', BL), ('BR', BR)]:
            cv2.circle(view, tuple(pt), 3, (0, 255, 0), -1)
            cv2.circle(view, tuple(pt), 4, (255, 255, 255), 1)
            cv2.putText(view, label, (pt[0] + 6, pt[1] - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 255, 0), 1)

    # 좌/우 변 중심
    if 'top_mids_2d' in result:
        mids = result['top_mids_2d']
        L_pt = mids['L'].astype(int)
        R_pt = mids['R'].astype(int)
        cv2.line(view, tuple(L_pt), tuple(R_pt), (255, 0, 255), 1)
        for label, pt in [('L', L_pt), ('R', R_pt)]:
            cv2.circle(view, tuple(pt), 5, (255, 0, 255), -1)
            cv2.circle(view, tuple(pt), 6, (255, 255, 255), 1)
            offset_x = -15 if label == 'L' else 9
            cv2.putText(view, label, (pt[0] + offset_x, pt[1] + 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 0, 255), 1)

    # 윗면 중심
    if 'top_center_3d' in result:
        tc = result['top_center_3d'].reshape(1, 3).astype(np.float32)
        p2d, _ = cv2.projectPoints(tc, rvec0, tvec0, K, dist)
        px, py = p2d[0, 0].astype(int)
        cv2.circle(view, (px, py), 5, (255, 0, 255), -1)
        cv2.circle(view, (px, py), 6, (255, 255, 255), 1)
        cv2.putText(view, "T", (px + 9, py - 9),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 0, 255), 1)

    # 정보 표시
    mask_px = int(mask.sum())
    top_px = int(result.get('top_pixels', 0))
    pct = 100 * mask_px / (H * W)
    cv2.putText(view, f"Mask: {mask_px}px ({pct:.1f}%)", (20, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 100, 255), 2)
    if top_px > 0:
        cv2.putText(view, f"Top: {top_px}px ({100*top_px/max(mask_px,1):.0f}% of mask)",
                    (20, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 150, 0), 2)

    return view


def detect_loop():
    global latest_annotated, latest_mask_view, latest_result

    print("[DETECT] 첫 프레임 대기...")
    for _ in range(100):
        with color_lock:
            ready = latest_color is not None
        if ready: break
        time.sleep(0.1)
    print("[DETECT] 검출 시작")

    while not stop_flag.is_set():
        loop_start = time.time()

        with color_lock:
            color = latest_color.copy() if latest_color is not None else None
        with depth_lock:
            depth = latest_depth.copy() if latest_depth is not None else None
        with gravity_lock:
            gravity = latest_gravity.copy() if latest_gravity is not None else None

        if color is None or depth is None:
            time.sleep(0.05)
            continue

        result = box_estimator.detect(color, depth, gravity_cam=gravity)

        # OBB 오버레이 화면
        annotated = color.copy()
        if result is not None:
            draw_box_overlay(annotated, result, camera_K)
            draw_info_panel(annotated, result, gravity)
        else:
            draw_no_detection(annotated, gravity)

        if gravity is not None:
            draw_gravity_overlay(annotated, gravity, camera_K)

        # 마스크 시각화 화면 (NEW)
        mask_view = make_mask_view(color, result, camera_K)

        with annotated_lock:
            latest_annotated = annotated
        with mask_view_lock:
            latest_mask_view = mask_view
        with result_lock:
            latest_result = result

        # 슬라이딩 윈도우 평균 업데이트
        update_smoothers(result)

        elapsed = time.time() - loop_start
        if elapsed < DETECT_INTERVAL:
            time.sleep(DETECT_INTERVAL - elapsed)


def draw_info_panel(img, result, gravity):
    # 새 구조: tvec/dims 없음
    center = result.get('center_3d', np.zeros(3))
    top    = result.get('top_center_3d', None)
    conf   = result.get('conf', 0)
    dist_m = float(result.get('distance_m', 0))
    z_m    = float(center[2])

    tilt_deg = None
    if gravity is not None:
        cos_t = float(gravity[1])
        tilt_deg = float(np.degrees(np.arccos(np.clip(cos_t, -1, 1))))

    panel_w, panel_h = 320, 155
    overlay = img.copy()
    cv2.rectangle(overlay, (10, 10), (10 + panel_w, 10 + panel_h),
                  (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.65, img, 0.35, 0, img)

    GREEN  = (0, 255, 0)
    YELLOW = (0, 255, 255)
    CYAN   = (255, 255, 0)
    MAG    = (255, 100, 255)
    font   = cv2.FONT_HERSHEY_SIMPLEX

    y = 32
    cv2.putText(img, f"Distance: {dist_m*100:.1f} cm",
                (20, y), font, 0.55, YELLOW, 2)
    y += 22
    cv2.putText(img, f"  (Z forward: {z_m*100:.1f} cm)",
                (20, y), font, 0.45, GREEN, 1)
    y += 22
    cv2.putText(img, f"Pos: ({center[0]*100:+.1f}, {center[1]*100:+.1f}, {center[2]*100:+.1f}) cm",
                (20, y), font, 0.45, GREEN, 1)
    y += 22
    if top is not None:
        cv2.putText(img, f"Top: ({top[0]*100:+.1f}, {top[1]*100:+.1f}, {top[2]*100:+.1f}) cm",
                    (20, y), font, 0.45, MAG, 1)
    else:
        cv2.putText(img, "Top: (not detected)",
                    (20, y), font, 0.45, (150, 150, 150), 1)
    y += 22
    cv2.putText(img, f"Conf: {conf:.2f}",
                (20, y), font, 0.5, GREEN, 1)
    y += 22
    if tilt_deg is not None:
        cv2.putText(img, f"cam_tilt: {tilt_deg:.1f}",
                    (20, y), font, 0.45, CYAN, 1)


def draw_no_detection(img, gravity):
    overlay = img.copy()
    panel_h = 70 if gravity is not None else 50
    cv2.rectangle(overlay, (10, 10), (320, 10 + panel_h), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.65, img, 0.35, 0, img)
    cv2.putText(img, "No box detected", (20, 38),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (100, 100, 255), 2)
    if gravity is not None:
        cos_t = float(gravity[1])
        tilt_deg = float(np.degrees(np.arccos(np.clip(cos_t, -1, 1))))
        cv2.putText(img, f"cam_tilt: {tilt_deg:.1f} (IMU active)",
                    (20, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                    (255, 255, 0), 1)


# ==========================================
# FastAPI
# ==========================================
@asynccontextmanager
async def lifespan(app: FastAPI):
    global box_estimator

    init_camera()

    print(f"[YOLO] 모델 로드: {YOLO_MODEL}")
    if not os.path.exists(YOLO_MODEL):
        print(f"[YOLO] 경고: {YOLO_MODEL} 없음")
    box_estimator = BoxEstimator(YOLO_MODEL, camera_K,
                                  conf=YOLO_CONF, device=YOLO_DEVICE)

    print("[YOLO] 워밍업...")
    t0 = time.time()
    box_estimator.detect(np.zeros((COLOR_H, COLOR_W, 3), dtype=np.uint8),
                          np.zeros((COLOR_H, COLOR_W), dtype=np.uint16))
    print(f"[YOLO] 완료 ({(time.time()-t0)*1000:.0f}ms)")

    threading.Thread(target=capture_loop, daemon=True).start()
    threading.Thread(target=detect_loop, daemon=True).start()

    print("[IMU] 안정화 대기...")
    for _ in range(30):
        with gravity_lock:
            if latest_gravity is not None: break
        time.sleep(0.1)

    print(f"\n[SERVER] http://0.0.0.0:{PORT}\n")
    yield

    stop_flag.set()
    time.sleep(0.3)
    if pipeline:
        try: pipeline.stop()
        except Exception: pass


app = FastAPI(title="Box Detection (IMU + Mask)", lifespan=lifespan)
app.add_middleware(CORSMiddleware, allow_origins=["*"],
                   allow_methods=["*"], allow_headers=["*"])


def gen_frames(buf_ref):
    """공통 MJPEG generator."""
    frame_interval = 1.0 / STREAM_FPS
    while True:
        loop_start = time.time()
        img = buf_ref()
        if img is None:
            time.sleep(0.05)
            continue
        ok, buf = cv2.imencode('.jpg', img,
                                [cv2.IMWRITE_JPEG_QUALITY, STREAM_QUALITY])
        if ok:
            yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n'
                   + buf.tobytes() + b'\r\n')
        elapsed = time.time() - loop_start
        if elapsed < frame_interval:
            time.sleep(frame_interval - elapsed)


def _get_annotated():
    with annotated_lock:
        return latest_annotated.copy() if latest_annotated is not None else None

def _get_mask_view():
    with mask_view_lock:
        return latest_mask_view.copy() if latest_mask_view is not None else None

def _get_raw():
    with color_lock:
        return latest_color.copy() if latest_color is not None else None


@app.get("/video_feed")
async def video_feed():
    return StreamingResponse(gen_frames(_get_annotated),
        media_type="multipart/x-mixed-replace; boundary=frame")


@app.get("/mask_feed")
async def mask_feed():
    return StreamingResponse(gen_frames(_get_mask_view),
        media_type="multipart/x-mixed-replace; boundary=frame")


@app.get("/raw_feed")
async def raw_feed():
    return StreamingResponse(gen_frames(_get_raw),
        media_type="multipart/x-mixed-replace; boundary=frame")


@app.get("/status")
async def status():
    with result_lock:
        r = latest_result
    with gravity_lock:
        g = latest_gravity.copy() if latest_gravity is not None else None

    out = {"detected": r is not None, "imu_active": g is not None}
    if g is not None:
        cos_t = float(g[1])
        tilt_deg = float(np.degrees(np.arccos(np.clip(cos_t, -1, 1))))
        out["gravity"] = [round(float(g[0]), 3),
                          round(float(g[1]), 3),
                          round(float(g[2]), 3)]
        out["camera_tilt_deg"] = round(tilt_deg, 1)

    if r is None:
        return out

    out.update({
        "confidence":   round(float(r['conf']), 3),
        "distance_cm":  round(float(r.get('distance_m', 0)) * 100, 1),
        "mask_pixels":  int(r.get('mask_pixels', 0)),
    })
    if 'center_3d' in r:
        c3d = r['center_3d']
        out["center_cm"] = {
            "x": round(float(c3d[0]) * 100, 1),
            "y": round(float(c3d[1]) * 100, 1),
            "z": round(float(c3d[2]) * 100, 1),
        }
    # 윗면 정보
    if 'top_center_3d' in r:
        tc = r['top_center_3d']
        out["top_center_cm"] = {
            "x": round(float(tc[0]) * 100, 1),
            "y": round(float(tc[1]) * 100, 1),
            "z": round(float(tc[2]) * 100, 1),
        }
        out["top_pixels"] = int(r.get('top_pixels', 0))
        out["h_top_cm"] = round(float(r.get('h_top_m', 0)) * 100, 1)
    # 양옆 중심 (L, R)
    if 'top_mids_3d' in r:
        mids = r['top_mids_3d']
        if mids.get('L') is not None and mids.get('R') is not None:
            out["sides_cm"] = {
                "L": [round(float(mids['L'][0])*100, 1),
                       round(float(mids['L'][1])*100, 1),
                       round(float(mids['L'][2])*100, 1)],
                "R": [round(float(mids['R'][0])*100, 1),
                       round(float(mids['R'][1])*100, 1),
                       round(float(mids['R'][2])*100, 1)],
            }
    # 박스 크기 추정
    if 'box_W_m' in r and 'box_D_m' in r:
        out["box_size_cm"] = {
            "W": round(float(r['box_W_m']) * 100, 1),
            "D": round(float(r['box_D_m']) * 100, 1),
        }
        if 'box_H_m' in r:
            out["box_size_cm"]["H"] = round(float(r['box_H_m']) * 100, 1)

    # 슬라이딩 윈도우 평균 (1초)
    sm = get_smoothed()
    if sm.get('_count', 0) > 0:
        def fmt(v):
            return [round(float(v[0])*100, 1),
                    round(float(v[1])*100, 1),
                    round(float(v[2])*100, 1)]
        smooth = {"frames": sm['_count']}
        if 'top_center' in sm:
            smooth['top_center_cm'] = fmt(sm['top_center'])
        if 'L' in sm:
            smooth['L_cm'] = fmt(sm['L'])
        if 'R' in sm:
            smooth['R_cm'] = fmt(sm['R'])
        # 박스 사이즈 — 안정화된 꼭지점에서 직접 계산
        sz = {}
        if all(k in sm for k in ['TL', 'TR', 'BL', 'BR']):
            top_edge    = float(np.linalg.norm(sm['TR'] - sm['TL']))
            bottom_edge = float(np.linalg.norm(sm['BR'] - sm['BL']))
            left_edge   = float(np.linalg.norm(sm['BL'] - sm['TL']))
            right_edge  = float(np.linalg.norm(sm['BR'] - sm['TR']))
            W = (top_edge + bottom_edge) / 2
            D = (left_edge + right_edge) / 2
            if D > W:
                W, D = D, W
            sz['W'] = round(W * 100, 1)
            sz['D'] = round(D * 100, 1)
        if 'box_H' in sm:
            sz['H'] = round(float(sm['box_H']) * 100, 1)
        if sz:
            smooth['box_size_cm'] = sz
        out["smoothed"] = smooth
    return out


HTML_PAGE = """<!DOCTYPE html>
<html>
<head>
    <meta charset="utf-8">
    <title>Box Detection</title>
    <style>
        * { box-sizing: border-box; margin: 0; padding: 0; }
        body { font-family: monospace; background: #1a1a1a; color: #fff; padding: 20px; }
        #wrap { max-width: 1500px; margin: 0 auto; }
        h1 { color: #4CAF50; font-size: 22px; margin-bottom: 4px; }
        .sub { color: #666; font-size: 13px; margin-bottom: 16px; }

        #views { display: flex; gap: 12px; margin-bottom: 16px; flex-wrap: wrap; }
        .view-card { display: flex; flex-direction: column; gap: 4px; }
        .view-card .label { font-size: 12px; color: #888; }
        .v-main { border: 2px solid #4CAF50; }
        .v-mask { border: 2px solid #FFC107; }
        .v-raw  { border: 2px solid #888; }

        #panel { background: #242424; border-radius: 10px; padding: 18px;
                 display: flex; gap: 18px; flex-wrap: wrap; }
        .card { background: #1e1e1e; border-radius: 8px; padding: 12px 14px;
                min-width: 180px; }
        .card-title { color: #4CAF50; font-size: 11px; text-transform: uppercase;
                      letter-spacing: 1px; margin-bottom: 8px; }
        .big { font-size: 24px; color: #fff; font-weight: bold; }
        .unit { font-size: 13px; color: #888; margin-left: 4px; }
        .info-row { display: flex; justify-content: space-between;
                    font-size: 13px; margin: 4px 0; min-width: 160px; }
        .info-key { color: #666; }
        .info-val { color: #ccc; }

        #status-bar { border-radius: 6px; padding: 8px 12px; font-size: 13px;
                       background: #2a2a2a; color: #666; margin-bottom: 12px;
                       display: inline-block; }
        .s-detected { background: #1a2a1a !important; color: #4CAF50 !important; }
        .s-waiting  { background: #2a2a1a !important; color: #FFC107 !important; }
        .badge { display: inline-block; padding: 1px 6px; border-radius: 4px;
                 font-size: 10px; margin-left: 4px; }
        .b-imu { background: #1a4a2a; color: #4CAF50; }
        .b-svd { background: #4a3a1a; color: #FFC107; }
    </style>
</head>
<body>
<div id="wrap">
    <h1>Box Detection</h1>
    <p class="sub">RealSense D435i · YOLO11-seg · port """ + str(PORT) + """</p>

    <div id="status-bar">대기 중</div>

    <div id="views">
        <div class="view-card">
            <div class="label">Detection</div>
            <img class="v-main" src="/video_feed" width="640" height="480">
        </div>
        <div class="view-card">
            <div class="label">Raw (원본)</div>
            <img class="v-raw" src="/raw_feed" width="640" height="480">
        </div>
    </div>

    <div id="panel">
        <div class="card">
            <div class="card-title">Distance</div>
            <span class="big" id="distance">—</span><span class="unit">cm</span>
        </div>
        <div class="card">
            <div class="card-title">Position (cam frame)</div>
            <div class="info-row"><span class="info-key">X</span>
                <span class="info-val" id="px">—</span></div>
            <div class="info-row"><span class="info-key">Y</span>
                <span class="info-val" id="py">—</span></div>
            <div class="info-row"><span class="info-key">Z</span>
                <span class="info-val" id="pz">—</span></div>
        </div>
        <div class="card">
            <div class="card-title">Confidence</div>
            <span class="big" id="conf">—</span>
        </div>
        <div class="card">
            <div class="card-title">Mask Pixels</div>
            <span class="big" id="mpx">—</span>
        </div>
        <div class="card">
            <div class="card-title">Top Center (cam frame)</div>
            <div class="info-row"><span class="info-key">X</span>
                <span class="info-val" id="tx">—</span></div>
            <div class="info-row"><span class="info-key">Y</span>
                <span class="info-val" id="ty">—</span></div>
            <div class="info-row"><span class="info-key">Z</span>
                <span class="info-val" id="tz">—</span></div>
            <div class="info-row"><span class="info-key">Top px</span>
                <span class="info-val" id="tpx">—</span></div>
        </div>
        <div class="card" style="min-width: 220px;">
            <div class="card-title">Sides (cam frame, cm)</div>
            <div class="info-row"><span class="info-key">L</span>
                <span class="info-val" id="sL">—</span></div>
            <div class="info-row"><span class="info-key">R</span>
                <span class="info-val" id="sR">—</span></div>
        </div>
        <div class="card">
            <div class="card-title">Box Size (estimated)</div>
            <span class="big" id="bsize" style="font-size:18px">— × — × —</span>
            <span class="unit">cm</span>
        </div>
        <div class="card" style="min-width: 260px;">
            <div class="card-title">Smoothed (1s avg)
                <span class="badge b-imu" id="smcnt">0</span></div>
            <div class="info-row"><span class="info-key">T</span>
                <span class="info-val" id="smT">—</span></div>
            <div class="info-row"><span class="info-key">L</span>
                <span class="info-val" id="smL">—</span></div>
            <div class="info-row"><span class="info-key">R</span>
                <span class="info-val" id="smR">—</span></div>
        </div>
    </div>
</div>

<script>
function poll() {
    fetch('/status').then(r => r.json()).then(d => {
        const sb = document.getElementById('status-bar');
        if (d.detected) {
            sb.textContent = 'Box detected ✓';
            sb.className = 's-detected';
            document.getElementById('distance').textContent = d.distance_cm.toFixed(1);
            if (d.center_cm) {
                document.getElementById('px').textContent = d.center_cm.x.toFixed(1);
                document.getElementById('py').textContent = d.center_cm.y.toFixed(1);
                document.getElementById('pz').textContent = d.center_cm.z.toFixed(1);
            }
            document.getElementById('conf').textContent = d.confidence.toFixed(2);
            document.getElementById('mpx').textContent = d.mask_pixels;
            // Top center
            if (d.top_center_cm) {
                document.getElementById('tx').textContent = d.top_center_cm.x.toFixed(1);
                document.getElementById('ty').textContent = d.top_center_cm.y.toFixed(1);
                document.getElementById('tz').textContent = d.top_center_cm.z.toFixed(1);
                document.getElementById('tpx').textContent = d.top_pixels;
            } else {
                ['tx','ty','tz','tpx'].forEach(id =>
                    document.getElementById(id).textContent = '—');
            }
            // Sides L/R
            if (d.sides_cm) {
                const fmt = (a) =>
                    `(${a[0].toFixed(1)}, ${a[1].toFixed(1)}, ${a[2].toFixed(1)})`;
                document.getElementById('sL').textContent = fmt(d.sides_cm.L);
                document.getElementById('sR').textContent = fmt(d.sides_cm.R);
            } else {
                ['sL','sR'].forEach(id =>
                    document.getElementById(id).textContent = '—');
            }
            // Box size (smoothed 우선)
            const bs = (d.smoothed && d.smoothed.box_size_cm) || d.box_size_cm;
            if (bs) {
                const w = bs.W !== undefined ? bs.W.toFixed(1) : '—';
                const dd = bs.D !== undefined ? bs.D.toFixed(1) : '—';
                const h = bs.H !== undefined ? bs.H.toFixed(1) : '—';
                document.getElementById('bsize').textContent =
                    `${w} × ${dd} × ${h}`;
            } else {
                document.getElementById('bsize').textContent = '— × — × —';
            }
            // Smoothed
            if (d.smoothed) {
                document.getElementById('smcnt').textContent =
                    `${d.smoothed.frames} frames`;
                const fmt = (a) =>
                    `(${a[0].toFixed(1)}, ${a[1].toFixed(1)}, ${a[2].toFixed(1)})`;
                document.getElementById('smT').textContent =
                    d.smoothed.top_center_cm ? fmt(d.smoothed.top_center_cm) : '—';
                document.getElementById('smL').textContent =
                    d.smoothed.L_cm ? fmt(d.smoothed.L_cm) : '—';
                document.getElementById('smR').textContent =
                    d.smoothed.R_cm ? fmt(d.smoothed.R_cm) : '—';
            } else {
                ['smT','smL','smR'].forEach(id =>
                    document.getElementById(id).textContent = '—');
                document.getElementById('smcnt').textContent = '0';
            }
        } else {
            sb.textContent = '박스 검출 대기...';
            sb.className = 's-waiting';
            ['distance','px','py','pz','conf','mpx',
             'tx','ty','tz','tpx','sL','sR',
             'smT','smL','smR'].forEach(id =>
                document.getElementById(id).textContent = '—');
            document.getElementById('bsize').textContent = '— × — × —';
            document.getElementById('smcnt').textContent = '0';
        }
    }).catch(e => {});
}
setInterval(poll, 200);
poll();
</script>
</body>
</html>"""


@app.get("/", response_class=HTMLResponse)
async def index():
    return HTML_PAGE


@app.get("/set_smooth_window")
async def set_smooth_window(seconds: float):
    """평균 윈도우 시간 변경 (초). 짧으면 반응 빠름, 길면 안정적."""
    global SMOOTH_WINDOW_SEC
    seconds = max(0.1, min(seconds, 10.0))   # 0.1초 ~ 10초 제한
    SMOOTH_WINDOW_SEC = seconds
    with smoother_lock:
        for sm in smoothers.values():
            sm.window_sec = seconds
            # 윈도우 줄였으면 오래된 항목 제거
            import time as _t
            now = _t.time()
            while sm.buf and now - sm.buf[0][0] > seconds:
                sm.buf.popleft()
    print(f"[SMOOTH] window = {seconds:.1f}s")
    return {"success": True, "smooth_window_sec": seconds}


if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=PORT, timeout_graceful_shutdown=2)
