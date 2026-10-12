#!/usr/bin/env python3
# Version: 1.2
# Changes:
#   1.2 - 기본 해상도 640x480 → 848x480 (D435 계열 depth 권장 해상도, 좌우 시야 넓음; RS_W/RS_H 로 변경 가능)
#         GET /intrinsics 추가 — 카메라에서 읽은 color K (detect_box/detect_marker 가 시작 시 읽음)
"""
RealSense → HTTP MJPEG 서버 (canvas viewer)

라우트:
  /            : canvas 기반 뷰어
  /video_feed  : color MJPEG (ik_box.py 등에서 사용)
  /depth_feed  : depth MJPEG (320x240 q60, 디버깅용 시각화)
  /depth_raw   : depth 16bit PNG 스트림 (mm 원본, detect_box용) ← v1.1 추가
  /intrinsics  : color 스트림 내부 파라미터 (fx, fy, ppx, ppy, width, height) ← v1.2 추가
"""

import os
import threading
import cv2
import numpy as np
import pyrealsense2 as rs
import uvicorn
from contextlib import asynccontextmanager
from fastapi import FastAPI
from fastapi.responses import StreamingResponse, HTMLResponse, JSONResponse
from fastapi.middleware.cors import CORSMiddleware


COLOR_W = int(os.environ.get("RS_W", 848))
COLOR_H = int(os.environ.get("RS_H", 480))
FPS     = 30
DEPTH_W, DEPTH_H      = COLOR_W // 2, COLOR_H // 2   # 시각화용 축소 (종횡비 유지)
COLOR_Q, DEPTH_Q      = 80, 60
DEPTH_MAX_MM          = 3000


class FrameBuffer:
    def __init__(self):
        self.data = None
        self.frame_id = 0
        self.cond = threading.Condition()
    def update(self, data: bytes):
        with self.cond:
            self.data = data
            self.frame_id += 1
            self.cond.notify_all()
    def wait_new(self, last_id: int, timeout: float = 1.0):
        with self.cond:
            self.cond.wait_for(lambda: self.frame_id != last_id, timeout=timeout)
            return self.data, self.frame_id


color_buf     = FrameBuffer()
depth_buf     = FrameBuffer()   # 시각화 JPEG
depth_raw_buf = FrameBuffer()   # 16bit PNG (mm 원본)

pipeline  = None
align     = None
intrinsics = None   # color 스트림 K (depth 는 color 에 align)
stop_flag = threading.Event()


def init_camera():
    global pipeline, align, intrinsics
    pipeline = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.color, COLOR_W, COLOR_H, rs.format.bgr8, FPS)
    cfg.enable_stream(rs.stream.depth, COLOR_W, COLOR_H, rs.format.z16,  FPS)
    profile = pipeline.start(cfg)
    align = rs.align(rs.stream.color)
    it = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
    intrinsics = {"width": it.width, "height": it.height,
                  "fx": it.fx, "fy": it.fy, "ppx": it.ppx, "ppy": it.ppy,
                  "model": str(it.model), "coeffs": [float(c) for c in it.coeffs],
                  "fps": FPS}
    print(f"[RS] K: fx={it.fx:.3f} fy={it.fy:.3f} ppx={it.ppx:.3f} ppy={it.ppy:.3f} "
          f"({it.width}x{it.height}, {it.model})")
    for _ in range(15):
        pipeline.wait_for_frames()
    print(f"[RS] 카메라 시작 ({COLOR_W}x{COLOR_H}@{FPS}fps)")


def capture_loop():
    depth_lut = np.clip(
        np.arange(65536, dtype=np.float32) * (255.0 / DEPTH_MAX_MM), 0, 255
    ).astype(np.uint8)
    color_enc = [cv2.IMWRITE_JPEG_QUALITY, COLOR_Q]
    depth_enc = [cv2.IMWRITE_JPEG_QUALITY, DEPTH_Q]
    # PNG 압축 레벨 낮게 (속도 우선)
    png_enc = [cv2.IMWRITE_PNG_COMPRESSION, 1]

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

        # --- color ---
        color_img = np.asanyarray(cf.get_data()).copy()
        ok, buf = cv2.imencode('.jpg', color_img, color_enc)
        if ok:
            color_buf.update(buf.tobytes())

        depth_img = np.asanyarray(df.get_data())   # uint16 mm

        # --- depth 시각화 (JET) ---
        d_color = cv2.applyColorMap(depth_lut[depth_img], cv2.COLORMAP_JET)
        d_color[depth_img == 0] = 0
        d_small = cv2.resize(d_color, (DEPTH_W, DEPTH_H), interpolation=cv2.INTER_NEAREST)
        ok, buf = cv2.imencode('.jpg', d_small, depth_enc)
        if ok:
            depth_buf.update(buf.tobytes())

        # --- depth raw (16bit PNG, mm 원본) ---
        ok, buf = cv2.imencode('.png', depth_img, png_enc)
        if ok:
            depth_raw_buf.update(buf.tobytes())


@asynccontextmanager
async def lifespan(app: FastAPI):
    init_camera()
    threading.Thread(target=capture_loop, daemon=True).start()
    yield
    stop_flag.set()
    if pipeline:
        try: pipeline.stop()
        except Exception: pass


app = FastAPI(title="RealSense MJPEG Stream", lifespan=lifespan)
app.add_middleware(CORSMiddleware, allow_origins=["*"],
                   allow_methods=["*"], allow_headers=["*"])


def mjpeg_generator(buffer: FrameBuffer, content_type=b'image/jpeg'):
    last_id = -1
    boundary = b'--frame\r\nContent-Type: ' + content_type + b'\r\n\r\n'
    while True:
        data, fid = buffer.wait_new(last_id, timeout=1.0)
        if data is None or fid == last_id:
            continue
        last_id = fid
        yield boundary + data + b'\r\n'


@app.get("/video_feed")
async def video_feed():
    return StreamingResponse(mjpeg_generator(color_buf),
        media_type="multipart/x-mixed-replace; boundary=frame")


@app.get("/depth_feed")
async def depth_feed():
    return StreamingResponse(mjpeg_generator(depth_buf),
        media_type="multipart/x-mixed-replace; boundary=frame")


@app.get("/depth_raw")
async def depth_raw():
    """16bit PNG (mm 원본) 스트림 — detect_box.py가 디코딩해서 사용."""
    return StreamingResponse(mjpeg_generator(depth_raw_buf, content_type=b'image/png'),
        media_type="multipart/x-mixed-replace; boundary=frame")


@app.get("/intrinsics")
async def get_intrinsics():
    """카메라에서 읽은 color K. depth_raw 는 color 에 align 되어 있어 같은 K 를 쓴다."""
    if intrinsics is None:
        return JSONResponse({"error": "camera not started"}, status_code=503)
    return intrinsics


@app.get("/")
async def index():
    return HTMLResponse("<h2>RealSense Stream</h2>"
                        "<p>/video_feed (color), /depth_feed (vis), /depth_raw (16bit mm), /intrinsics (K)</p>"
                        '<img src="/video_feed" width="480">')


if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=50001, timeout_graceful_shutdown=2)
