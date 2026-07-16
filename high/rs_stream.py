#!/usr/bin/env python3
"""
RealSense → HTTP MJPEG 서버 (canvas viewer)

라우트 (원본과 동일):
  /            : canvas 기반 뷰어
  /video_feed  : color MJPEG (ik_box.py 등에서 사용)
  /depth_feed  : depth MJPEG (320x240 q60, 디버깅용)
"""

import threading
import cv2
import numpy as np
import pyrealsense2 as rs
import uvicorn
from contextlib import asynccontextmanager
from fastapi import FastAPI
from fastapi.responses import StreamingResponse, HTMLResponse
from fastapi.middleware.cors import CORSMiddleware


# ==========================================
# 설정
# ==========================================
COLOR_W, COLOR_H, FPS = 640, 480, 30
DEPTH_W, DEPTH_H      = 320, 240
COLOR_Q, DEPTH_Q      = 80, 60
DEPTH_MAX_MM          = 3000


# ==========================================
# Frame buffer (1 producer / N consumer)
# ==========================================
class FrameBuffer:
    def __init__(self):
        self.jpeg = None
        self.frame_id = 0
        self.cond = threading.Condition()

    def update(self, jpeg_bytes: bytes):
        with self.cond:
            self.jpeg = jpeg_bytes
            self.frame_id += 1
            self.cond.notify_all()

    def wait_new(self, last_id: int, timeout: float = 1.0):
        with self.cond:
            self.cond.wait_for(lambda: self.frame_id != last_id, timeout=timeout)
            return self.jpeg, self.frame_id


color_buf = FrameBuffer()
depth_buf = FrameBuffer()

pipeline  = None
align     = None
stop_flag = threading.Event()


# ==========================================
# 카메라
# ==========================================
def init_camera():
    global pipeline, align
    pipeline = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.color, COLOR_W, COLOR_H, rs.format.bgr8, FPS)
    cfg.enable_stream(rs.stream.depth, COLOR_W, COLOR_H, rs.format.z16,  FPS)
    pipeline.start(cfg)
    align = rs.align(rs.stream.color)
    for _ in range(15):
        pipeline.wait_for_frames()
    print(f"[RS] 카메라 시작 ({COLOR_W}x{COLOR_H}@{FPS}fps)")


def capture_loop():
    depth_lut = np.clip(
        np.arange(65536, dtype=np.float32) * (255.0 / DEPTH_MAX_MM), 0, 255
    ).astype(np.uint8)
    color_enc = [cv2.IMWRITE_JPEG_QUALITY, COLOR_Q]
    depth_enc = [cv2.IMWRITE_JPEG_QUALITY, DEPTH_Q]

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

        # --- depth (320x240, q60) ---
        depth_img = np.asanyarray(df.get_data())
        d_color = cv2.applyColorMap(depth_lut[depth_img], cv2.COLORMAP_JET)
        d_color[depth_img == 0] = 0
        d_small = cv2.resize(d_color, (DEPTH_W, DEPTH_H), interpolation=cv2.INTER_NEAREST)
        ok, buf = cv2.imencode('.jpg', d_small, depth_enc)
        if ok:
            depth_buf.update(buf.tobytes())


# ==========================================
# FastAPI
# ==========================================
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


def mjpeg_generator(buffer: FrameBuffer):
    last_id = -1
    boundary = b'--frame\r\nContent-Type: image/jpeg\r\n\r\n'
    while True:
        data, fid = buffer.wait_new(last_id, timeout=1.0)
        if data is None or fid == last_id:
            continue
        last_id = fid
        yield boundary + data + b'\r\n'


@app.get("/video_feed")
async def video_feed():
    return StreamingResponse(
        mjpeg_generator(color_buf),
        media_type="multipart/x-mixed-replace; boundary=frame",
    )


@app.get("/depth_feed")
async def depth_feed():
    return StreamingResponse(
        mjpeg_generator(depth_buf),
        media_type="multipart/x-mixed-replace; boundary=frame",
    )


@app.get("/")
async def index():
    return HTMLResponse(r"""
<!DOCTYPE html>
<html><head><meta charset="utf-8"><title>RealSense Canvas Viewer</title>
<style>
  body { background:#1a1a1a; color:#fff; font-family:monospace; padding:20px; margin:0; }
  h2 { color:#4CAF50; margin-top:0; }
  .row { display:flex; gap:20px; flex-wrap:wrap; }
  .card { display:flex; flex-direction:column; gap:6px; }
  .label { font-size:13px; }
  .stats { font-size:12px; color:#888; }
  canvas { display:block; background:#000; }
  #c_color { border:2px solid #4CAF50; }
  #c_depth { border:2px solid #FF9800; }
  /* MJPEG 수신용 hidden img */
  .hidden-src { position:absolute; left:-9999px; width:1px; height:1px; }
</style></head>
<body>
  <h2>RealSense Canvas Viewer</h2>
  <div class="row">
    <div class="card">
      <div class="label">Color (640×480)</div>
      <canvas id="c_color" width="640" height="480"></canvas>
      <div class="stats" id="s_color">— fps</div>
    </div>
    <div class="card">
      <div class="label">Depth (320×240, 0~3m JET)</div>
      <canvas id="c_depth" width="320" height="240"></canvas>
      <div class="stats" id="s_depth">— fps</div>
    </div>
  </div>

  <!-- MJPEG는 브라우저가 알아서 decode → hidden img → canvas로 복사 -->
  <img id="src_color" class="hidden-src" src="/video_feed" crossorigin="anonymous">
  <img id="src_depth" class="hidden-src" src="/depth_feed" crossorigin="anonymous">

<script>
function attachCanvas(imgId, canvasId, statsId) {
  const img    = document.getElementById(imgId);
  const canvas = document.getElementById(canvasId);
  const ctx    = canvas.getContext('2d');
  const statEl = document.getElementById(statsId);

  let n = 0, t0 = performance.now();

  function tick() {
    if (img.naturalWidth) {
      ctx.drawImage(img, 0, 0, canvas.width, canvas.height);
      n++;
      const now = performance.now();
      if (now - t0 >= 1000) {
        statEl.textContent = `${(n * 1000 / (now - t0)).toFixed(1)} fps`;
        n = 0; t0 = now;
      }
    }
    requestAnimationFrame(tick);
  }
  requestAnimationFrame(tick);
}
attachCanvas('src_color', 'c_color', 's_color');
attachCanvas('src_depth', 'c_depth', 's_depth');
</script>
</body></html>
    """)


if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=50001, timeout_graceful_shutdown=2)
