#!/usr/bin/env python3
# Version: 0.6
# Changes:
#   0.6 - 미검출 3프레임 연속 시 smoother 버퍼 클리어(박스 치우면 잔상 제거)
#   0.5 - OpenVINO 모델 자동 감지(box_openvino_model), intel:cpu
#   0.4 - 카메라 K를 ik_box 검증값(606)으로 통일
#   0.3 - update_smoothers 키 수정(top_mids_3d), depth 수신 견고화
#   0.2 - YOLO_MODEL 경로 ../models/box.pt, box_estimator import
#   0.1 - 박스 인식 전용 서버 초기본
"""
detect_box.py — 박스 인식 전용 서버 (50010)

box_estimator.py(YOLO seg + depth)로 박스 윗면/L/R/크기 측정.
로봇(arm) 절대 안 건드림.

카메라 소스:
  · color : 50001 /video_feed (MJPEG)
  · depth : 50001 /depth_raw  (16bit PNG, mm) ← rs_stream에 추가 필요
  좌표계는 카메라 frame, gravity는 47.6도 고정.

동작:
  · 1초 윈도우 median 안정화 (좌표 → 크기 자동 안정)
  · 자동 모드: 영역 안 dwell 만족 → robot_server(50003) POST /grab_at
  · GET /pose : 현재 박스 좌표 (수동 잡기용)
  · 작은 웹 UI

robot_server active_mode == "box"일 때만 POST (그쪽 게이트).
"""
import os
import io
import time
import json
import threading
import urllib.request
from collections import deque
import numpy as np
import cv2
import uvicorn
from contextlib import asynccontextmanager
from fastapi import FastAPI
from fastapi.responses import HTMLResponse, StreamingResponse, JSONResponse
from fastapi.middleware.cors import CORSMiddleware

from box_estimator import BoxEstimator, draw_box_overlay

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))   # high/ctrl
_HIGH_DIR = os.path.dirname(_THIS_DIR)                     # high


# ==========================================
# 설정
# ==========================================
PORT          = 50010
COLOR_URL     = os.environ.get("RS_COLOR_URL", "http://localhost:50001/video_feed")
DEPTH_URL     = os.environ.get("RS_DEPTH_URL", "http://localhost:50001/depth_raw")
ROBOT_SERVER  = os.environ.get("ROBOT_SERVER", "http://localhost:50000")

# 카메라 K (ik_box 검증값 — detect_marker와 동일하게 통일)
CAM_FX, CAM_FY = 606.756104, 606.583374
CAM_PPX, CAM_PPY = 316.739441, 258.982391
camera_K = np.array([[CAM_FX,0,CAM_PPX],[0,CAM_FY,CAM_PPY],[0,0,1]], dtype=np.float32)

# 카메라 tilt 고정 (G1, 47.6도)
CAM_TILT_DEG = 47.6
_t = np.radians(CAM_TILT_DEG)
GRAVITY_CAM = np.array([0.0, np.cos(_t), np.sin(_t)], dtype=np.float64)

# camera_to_torso (영역 판정용)
CAMERA_X, CAMERA_Y, CAMERA_Z = 0.0576235, 0.03003, 0.42987
CAMERA_PITCH_URDF = 0.8307767239493009

_OV_DIR = os.path.join(_HIGH_DIR, "models", "box_openvino_model")
_PT     = os.path.join(_HIGH_DIR, "models", "box.pt")
# OpenVINO 모델 폴더가 있으면 우선 사용 (Intel CPU에서 2~4배 빠름)
_DEFAULT_MODEL = _OV_DIR if os.path.isdir(_OV_DIR) else _PT
YOLO_MODEL = os.environ.get("YOLO_MODEL", _DEFAULT_MODEL)
YOLO_CONF  = 0.4
# OpenVINO면 intel:cpu, 아니면 cpu
YOLO_DEVICE = os.environ.get("YOLO_DEVICE",
                             "intel:cpu" if YOLO_MODEL.endswith("openvino_model") else "cpu")

SMOOTH_WINDOW_SEC = 2.0
STREAM_FPS_MAX = 15
STREAM_QUALITY = 70


# ==========================================
# 전역
# ==========================================
latest_color    = None
latest_depth    = None       # uint16 mm
latest_annotated = None
latest_result   = None

color_lock     = threading.Lock()
depth_lock     = threading.Lock()
annotated_lock = threading.Lock()
result_lock    = threading.Lock()
stream_started = False

estimator = None

auto_mode = {"enabled": False,
             "x_min":0.30,"x_max":0.45,"y_min":-0.20,"y_max":0.20,
             "z_min":-0.15,"z_max":0.25,"dwell_sec":1.5}
auto_state = {"in_zone_since": None}


# ==========================================
# 좌표 변환 + smoother
# ==========================================
def camera_to_torso(cx, cy, cz):
    cos_p, sin_p = np.cos(CAMERA_PITCH_URDF), np.sin(CAMERA_PITCH_URDF)
    cy_r =  cy*cos_p + cz*sin_p
    cz_r = -cy*sin_p + cz*cos_p
    return float(cz_r+CAMERA_X), float(-cx+CAMERA_Y), float(-cy_r+CAMERA_Z)


class Smoother:
    def __init__(self, win=2.0):
        self.win = win; self.buf = deque()
    def push(self, v):
        now = time.time()
        self.buf.append((now, np.asarray(v, dtype=np.float64)))
        while self.buf and now-self.buf[0][0] > self.win:
            self.buf.popleft()
    def median(self):
        if not self.buf: return None
        return np.median(np.stack([v for _,v in self.buf]), axis=0)
    def count(self): return len(self.buf)
    def clear(self): self.buf.clear()

smoothers = {k: Smoother(SMOOTH_WINDOW_SEC)
             for k in ['top_center','L','R','box_H']}
smoother_lock = threading.Lock()


_miss_count = 0

def update_smoothers(result):
    global _miss_count
    # 박스 있나? (검출 성공 + top_center 존재)
    has_box = result is not None and result.get('top_center_3d') is not None
    if not has_box:
        _miss_count += 1
        # 연속 미검출이면 버퍼 비워서 옛 값 잔상 제거
        if _miss_count >= 3:
            with smoother_lock:
                for sm in smoothers.values():
                    sm.clear()
        return
    _miss_count = 0
    with smoother_lock:
        smoothers['top_center'].push(result['top_center_3d'])
        mids = result.get('top_mids_3d')
        if mids:
            if mids.get('L') is not None:
                smoothers['L'].push(mids['L'])
            if mids.get('R') is not None:
                smoothers['R'].push(mids['R'])
        if result.get('box_H_m') is not None:
            smoothers['box_H'].push([result['box_H_m']])


def get_smoothed():
    out = {}
    with smoother_lock:
        for k, sm in smoothers.items():
            v = sm.median()
            if v is not None: out[k] = v
        out['_count'] = max((sm.count() for sm in smoothers.values()), default=0)
    return out


# ==========================================
# 스트림 수신 (color MJPEG + depth raw)
# ==========================================
def color_reader_loop():
    global latest_color, stream_started
    while True:
        try:
            req = urllib.request.urlopen(COLOR_URL, timeout=5)
            stream_started = True
            print("[COLOR] 연결")
            buf = b""
            while True:
                chunk = req.read(4096)
                if not chunk: break
                buf += chunk
                while True:
                    soi = buf.find(b'\xff\xd8')
                    eoi = buf.find(b'\xff\xd9', soi+2) if soi>=0 else -1
                    if soi<0 or eoi<0: break
                    jpg = buf[soi:eoi+2]; buf = buf[eoi+2:]
                    img = cv2.imdecode(np.frombuffer(jpg,np.uint8), cv2.IMREAD_COLOR)
                    if img is not None:
                        with color_lock:
                            latest_color = img
        except Exception as e:
            print(f"[COLOR] 오류: {e}"); stream_started=False; time.sleep(2.0)


def depth_reader_loop():
    """50001 /depth_raw — 16bit PNG (mm) 스트림."""
    global latest_depth
    while True:
        try:
            req = urllib.request.urlopen(DEPTH_URL, timeout=5)
            print("[DEPTH] 연결")
            buf = b""
            while True:
                chunk = req.read(16384)
                if not chunk:
                    break
                buf += chunk
                # 완성된 PNG 프레임 추출
                while True:
                    soi = buf.find(b'\x89PNG')
                    if soi < 0:
                        if len(buf) > 4: buf = buf[-4:]   # 헤더 일부 보존
                        break
                    end = buf.find(b'IEND', soi+4)
                    if end < 0:
                        break
                    eoi = end + 8
                    if eoi > len(buf):
                        break
                    png = buf[soi:eoi]
                    buf = buf[eoi:]
                    try:
                        d = cv2.imdecode(np.frombuffer(png, np.uint8), cv2.IMREAD_UNCHANGED)
                        if d is not None and d.dtype == np.uint16:
                            with depth_lock:
                                latest_depth = d
                    except Exception:
                        pass
                # 버퍼 폭주 방지
                if len(buf) > 4_000_000:
                    buf = b""
        except Exception as e:
            print(f"[DEPTH] 오류: {e}")
            time.sleep(1.0)


def detect_loop():
    global latest_annotated, latest_result
    print("[DETECT] 첫 프레임 대기...")
    for _ in range(100):
        with color_lock:
            if latest_color is not None: break
        time.sleep(0.1)
    print("[DETECT] 시작")
    while True:
        # box 모드 아닐 때 CPU 절약
        if robot_mode() != "box":
            time.sleep(0.5); continue
        with color_lock:
            color = latest_color.copy() if latest_color is not None else None
        with depth_lock:
            depth = latest_depth.copy() if latest_depth is not None else None
        if color is None or depth is None:
            time.sleep(0.05); continue
        # depth 해상도 맞추기 (color 기준)
        if depth.shape[:2] != color.shape[:2]:
            depth = cv2.resize(depth, (color.shape[1], color.shape[0]),
                               interpolation=cv2.INTER_NEAREST)
        result = estimator.detect(color, depth, gravity_cam=GRAVITY_CAM)
        annotated = color.copy()
        if result is not None:
            draw_box_overlay(annotated, result, camera_K)
        with annotated_lock:
            latest_annotated = annotated
        with result_lock:
            latest_result = result
        update_smoothers(result)
        time.sleep(1.0/10.0)


# ==========================================
# 자동 모니터 → POST
# ==========================================
def robot_mode():
    try:
        d = json.loads(urllib.request.urlopen(f"{ROBOT_SERVER}/active_mode", timeout=0.5).read())
        return d.get("mode","none")
    except Exception:
        return "none"


def robot_busy():
    try:
        d = json.loads(urllib.request.urlopen(f"{ROBOT_SERVER}/active_mode", timeout=0.5).read())
        if d.get("mode") != "box": return True
        return bool(d.get("busy") or d.get("is_running"))
    except Exception:
        return True


def post_grab(sm):
    body = json.dumps({
        "type": "cardboard",
        "L": [float(v) for v in sm['L']],
        "R": [float(v) for v in sm['R']],
        "top_center": [float(v) for v in sm['top_center']],
        "box_h": float(sm['box_H'][0]) if 'box_H' in sm else None,
    }).encode()
    try:
        req = urllib.request.Request(f"{ROBOT_SERVER}/grab_at", data=body,
                                     headers={"Content-Type":"application/json"})
        r = urllib.request.urlopen(req, timeout=1.0)
        print(f"[POST] grab_at: {r.read().decode()[:80]}")
    except Exception as e:
        print(f"[POST] 실패: {e}")


def auto_monitor_loop():
    while True:
        time.sleep(0.1)
        if not auto_mode["enabled"]:
            auto_state["in_zone_since"]=None; continue
        sm = get_smoothed()
        if sm.get('_count',0) < 3 or 'top_center' not in sm:
            auto_state["in_zone_since"]=None; continue
        if robot_busy():
            auto_state["in_zone_since"]=None; continue

        mx, my, mz = camera_to_torso(*sm['top_center'])
        in_zone = (auto_mode["x_min"]<=mx<=auto_mode["x_max"] and
                   auto_mode["y_min"]<=my<=auto_mode["y_max"] and
                   auto_mode["z_min"]<=mz<=auto_mode["z_max"])
        if not in_zone:
            auto_state["in_zone_since"]=None; continue
        if auto_state["in_zone_since"] is None:
            auto_state["in_zone_since"]=time.time(); continue
        if time.time()-auto_state["in_zone_since"] >= auto_mode["dwell_sec"]:
            print("[AUTO] dwell 만족 → POST grab")
            post_grab(sm)
            auto_state["in_zone_since"]=None
            time.sleep(2.0)


# ==========================================
# FastAPI
# ==========================================
@asynccontextmanager
async def lifespan(app: FastAPI):
    global estimator
    print(f"[YOLO] 로드: {YOLO_MODEL} (device={YOLO_DEVICE})")
    estimator = BoxEstimator(YOLO_MODEL, camera_K, conf=YOLO_CONF, device=YOLO_DEVICE)
    # 워밍업
    estimator.detect(np.zeros((480,640,3),np.uint8),
                     np.zeros((480,640),np.uint16), gravity_cam=GRAVITY_CAM)
    threading.Thread(target=color_reader_loop, daemon=True).start()
    threading.Thread(target=depth_reader_loop, daemon=True).start()
    threading.Thread(target=detect_loop, daemon=True).start()
    threading.Thread(target=auto_monitor_loop, daemon=True).start()
    print(f"[detect_box] http://0.0.0.0:{PORT}/")
    yield


app = FastAPI(title="Detect Box", lifespan=lifespan)
app.add_middleware(CORSMiddleware, allow_origins=["*"],
                   allow_methods=["*"], allow_headers=["*"])


def gen_frames():
    interval = 1.0/STREAM_FPS_MAX
    nxt = 0.0
    while True:
        now = time.time()
        if now < nxt: time.sleep(max(0,nxt-now))
        nxt = time.time()+interval
        with annotated_lock:
            img = None if latest_annotated is None else latest_annotated.copy()
        if img is None:
            time.sleep(0.05); continue
        _, buf = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, STREAM_QUALITY])
        yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n'+buf.tobytes()+b'\r\n')


@app.get("/video_feed")
async def video_feed():
    return StreamingResponse(gen_frames(),
                             media_type="multipart/x-mixed-replace; boundary=frame")


@app.get("/pose")
async def pose():
    """현재 박스 좌표 (수동 잡기용) — 안정화된 값."""
    sm = get_smoothed()
    if sm.get('_count',0) < 3 or 'L' not in sm or 'R' not in sm:
        return {"found": False}
    out = {"found": True, "type": "cardboard",
           "L": [float(v) for v in sm['L']],
           "R": [float(v) for v in sm['R']],
           "top_center": [float(v) for v in sm['top_center']] if 'top_center' in sm else None,
           "box_h": float(sm['box_H'][0]) if 'box_H' in sm else None}
    return out


@app.get("/status")
async def status():
    sm = get_smoothed()
    found = sm.get('_count',0) >= 3 and 'top_center' in sm
    in_zone_since = auto_state.get("in_zone_since")
    elapsed = (time.time()-in_zone_since) if in_zone_since else 0.0
    out = {"found": found, "frames": sm.get('_count',0),
           "stream_started": stream_started,
           "auto_enabled": auto_mode["enabled"],
           "auto_in_zone": in_zone_since is not None,
           "auto_elapsed": round(elapsed,2),
           "auto_dwell": auto_mode["dwell_sec"]}
    if found:
        mx,my,mz = camera_to_torso(*sm['top_center'])
        out["torso"] = {"x":round(mx,3),"y":round(my,3),"z":round(mz,3)}
        if 'box_H' in sm:
            out["box_h_cm"] = round(float(sm['box_H'][0])*100,1)
    return out


@app.get("/set_auto_mode")
async def set_auto_mode(enabled: bool=None,
                        x_min: float=None, x_max: float=None,
                        y_min: float=None, y_max: float=None,
                        z_min: float=None, z_max: float=None,
                        dwell_sec: float=None):
    for k,v in [("enabled",enabled),("x_min",x_min),("x_max",x_max),
                ("y_min",y_min),("y_max",y_max),("z_min",z_min),
                ("z_max",z_max),("dwell_sec",dwell_sec)]:
        if v is not None: auto_mode[k]=v
    auto_state["in_zone_since"]=None
    return {"success": True, "config": auto_mode}


@app.get("/", response_class=HTMLResponse)
async def index():
    return HTML_PAGE


HTML_PAGE = """<!DOCTYPE html><html><head><meta charset="utf-8">
<title>Detect Box</title>
<style>
body{font-family:monospace;background:#1a1a1a;color:#fff;padding:20px}
h1{color:#FF9800;font-size:18px}
#wrap{display:flex;gap:20px}
img{border:2px solid #FF9800}
.panel{width:280px}
.card{background:#242424;border-radius:8px;padding:14px;margin-bottom:12px}
.card-title{color:#FF9800;font-size:12px;text-transform:uppercase;margin-bottom:8px}
.info-row{display:flex;justify-content:space-between;font-size:13px;margin:3px 0}
.info-key{color:#666}.info-val{color:#ccc}
input{background:#333;border:1px solid #444;color:#fff;padding:4px;border-radius:4px;width:70px}
button{background:#FF9800;border:none;color:#000;padding:8px;border-radius:5px;cursor:pointer;width:100%;font-weight:bold;margin-top:8px}
.bar{height:6px;background:#333;border-radius:3px;overflow:hidden;margin-top:8px}
.bar-fill{height:100%;width:0;background:#FF9800;transition:width .15s}
</style></head><body>
<h1>📦 Detect Box (50010)</h1>
<div id="wrap">
  <img src="/video_feed" width="640" height="480">
  <div class="panel">
    <div class="card">
      <div class="card-title">박스</div>
      <div id="bstatus">대기...</div>
      <div class="info-row"><span class="info-key">torso X</span><span class="info-val" id="tx">-</span></div>
      <div class="info-row"><span class="info-key">torso Y</span><span class="info-val" id="ty">-</span></div>
      <div class="info-row"><span class="info-key">torso Z</span><span class="info-val" id="tz">-</span></div>
      <div class="info-row"><span class="info-key">box H</span><span class="info-val" id="bh">-</span></div>
      <div class="info-row"><span class="info-key">frames</span><span class="info-val" id="fr">-</span></div>
    </div>
    <div class="card">
      <div class="card-title">자동 모드</div>
      <label><input type="checkbox" id="auto" onchange="toggleAuto()" style="width:auto"> 자동 잡기</label>
      <div style="display:grid;grid-template-columns:1fr 1fr;gap:5px;margin-top:8px;font-size:11px;color:#888">
        <div>X min<input id="ax-min" value="0.30"></div><div>X max<input id="ax-max" value="0.45"></div>
        <div>Y min<input id="ay-min" value="-0.20"></div><div>Y max<input id="ay-max" value="0.20"></div>
        <div>Z min<input id="az-min" value="-0.15"></div><div>Z max<input id="az-max" value="0.25"></div>
        <div>dwell<input id="dwell" value="1.5"></div>
      </div>
      <button onclick="applyZone()">영역 적용</button>
      <div class="bar"><div class="bar-fill" id="bar"></div></div>
      <div id="amsg" style="font-size:11px;color:#666;margin-top:6px">대기</div>
    </div>
  </div>
</div>
<script>
function poll(){fetch('/status').then(r=>r.json()).then(d=>{
  document.getElementById('bstatus').textContent=d.found?'검출됨 ✓':'대기...';
  document.getElementById('fr').textContent=d.frames;
  if(d.torso){document.getElementById('tx').textContent=d.torso.x.toFixed(3);
    document.getElementById('ty').textContent=d.torso.y.toFixed(3);
    document.getElementById('tz').textContent=d.torso.z.toFixed(3);}
  document.getElementById('bh').textContent=d.box_h_cm?d.box_h_cm+' cm':'-';
  document.getElementById('auto').checked=d.auto_enabled;
  const pct=d.auto_dwell>0?Math.min(100,d.auto_elapsed/d.auto_dwell*100):0;
  document.getElementById('bar').style.width=pct+'%';
  document.getElementById('amsg').textContent=
    !d.auto_enabled?'OFF':(d.auto_in_zone?`영역 안 ${d.auto_elapsed.toFixed(1)}/${d.auto_dwell}`:'영역 밖');
});}
setInterval(poll,300);
function toggleAuto(){fetch('/set_auto_mode?enabled='+document.getElementById('auto').checked);}
function applyZone(){
  const g=(id)=>document.getElementById(id).value;
  const q=`x_min=${g('ax-min')}&x_max=${g('ax-max')}&y_min=${g('ay-min')}&y_max=${g('ay-max')}&z_min=${g('az-min')}&z_max=${g('az-max')}&dwell_sec=${g('dwell')}`;
  fetch('/set_auto_mode?'+q).then(()=>document.getElementById('amsg').textContent='적용됨');
}
</script></body></html>"""


if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=PORT)
