#!/usr/bin/env python3
# Version: 0.3 (detect_box.py v0.6 기반 — 컨베이어 전용)
# Changes:
#   0.3 - 검출 깜빡임 대응: SMOOTH 0.3→0.8s, 짧은 프레임 드롭은 무시(LOST_FRAMES)
#         → dwell 타이머가 한 프레임 미검출에 리셋되던 문제 수정
#   0.2 - staged: 박스 감지 즉시 POST /conveyor_arm (대기손 하강 요청, 1회)
#         · robot arming 중엔 grab 안 보냄(active_mode.arming 확인)
#         · 박스 사라지면 armed_sent 리셋 → 다음 박스에 재-arm
# Base changes vs detect_box.py:
#   - PORT 50010 -> 50012
#   - type "cardboard" -> "conveyor",  robot mode "box" -> "conveyor"
#   - SMOOTH_WINDOW 2.0 -> 0.3s (움직이는 박스: median 지연 최소화)
#   - zone 기본값 좁게 + dwell 0.3s (박스가 대기손에 막혀 정지 → range 진입 시 발사)
"""
detect_box_conv.py — 컨베이어 박스 인식 서버 (50012)

기존 detect_box.py(50010, 정지 테이블용)를 건드리지 않고 복제한 별도 프로세스.
컨베이어 시연 전용:

  · 로봇은 한쪽 손(left/right)을 벨트 위 대기 위치(conv_wait_pos)에 미리 내려둠
  · 흘러온 박스가 대기 손에 막혀 대략 중앙에서 정지
  · 박스 중심(top_center)이 torso 좌표 range 안에 dwell 만족 → POST /grab_at (type=conveyor)
  · robot_server ACTIVE_MODE == "conveyor" 일 때만 발사 (그쪽 게이트)

박스가 정지한 뒤 인식하므로 스무딩 윈도우를 짧게(0.3s) 둬서
대기 전 이동 구간의 잔상/지연을 줄인다.
robot(arm) 절대 안 건드림. detect_box.py 와 동시 실행 가능(모드 안 겹치면 서로 sleep).
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
PORT          = 50012                       # ← detect_box=50010, conveyor=50012
ROBOT_MODE    = "conveyor"                   # ← 이 서버가 담당하는 robot_server 모드
GRAB_TYPE     = "conveyor"                   # ← POST /grab_at 로 보낼 type
COLOR_URL     = os.environ.get("RS_COLOR_URL", "http://localhost:50001/video_feed")
DEPTH_URL     = os.environ.get("RS_DEPTH_URL", "http://localhost:50001/depth_raw")
ROBOT_SERVER  = os.environ.get("ROBOT_SERVER", "http://localhost:50000")

# 카메라 K (ik_box 검증값 — detect_box 와 동일)
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
_DEFAULT_MODEL = _OV_DIR if os.path.isdir(_OV_DIR) else _PT
YOLO_MODEL = os.environ.get("YOLO_MODEL", _DEFAULT_MODEL)
YOLO_CONF  = 0.4
YOLO_DEVICE = os.environ.get("YOLO_DEVICE",
                             "intel:cpu" if YOLO_MODEL.endswith("openvino_model") else "cpu")

# 박스는 대기손에 막혀 catch 지점에서 정지하므로 창을 조금 길게 잡아 검출 깜빡임 흡수.
SMOOTH_WINDOW_SEC = 0.8
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

# range = "로봇 중심 range" (torso 좌표). 대기 손 위치(conv_wait_pos)에 맞춰 튜닝할 것.
# 움직이는 박스라 dwell 은 짧게(0.3s) — 몇 프레임 연속 range 안이면 발사.
auto_mode = {"enabled": False,
             "x_min":0.28,"x_max":0.42,"y_min":-0.15,"y_max":0.15,
             "z_min":-0.15,"z_max":0.25,"dwell_sec":0.3}
auto_state = {"in_zone_since": None, "armed_sent": False}


# ==========================================
# 좌표 변환 + smoother
# ==========================================
def camera_to_torso(cx, cy, cz):
    cos_p, sin_p = np.cos(CAMERA_PITCH_URDF), np.sin(CAMERA_PITCH_URDF)
    cy_r =  cy*cos_p + cz*sin_p
    cz_r = -cy*sin_p + cz*cos_p
    return float(cz_r+CAMERA_X), float(-cx+CAMERA_Y), float(-cy_r+CAMERA_Z)


class Smoother:
    def __init__(self, win=0.3):
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
    has_box = result is not None and result.get('top_center_3d') is not None
    if not has_box:
        _miss_count += 1
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
# 스트림 수신 (color MJPEG + depth raw)  — detect_box 와 동일
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
                while True:
                    soi = buf.find(b'\x89PNG')
                    if soi < 0:
                        if len(buf) > 4: buf = buf[-4:]
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
        # conveyor 모드 아닐 때 CPU 절약 (detect_box 와 서로 배타)
        if robot_mode() != ROBOT_MODE:
            time.sleep(0.5); continue
        with color_lock:
            color = latest_color.copy() if latest_color is not None else None
        with depth_lock:
            depth = latest_depth.copy() if latest_depth is not None else None
        if color is None or depth is None:
            time.sleep(0.05); continue
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
        time.sleep(1.0/15.0)   # 컨베이어: 조금 더 자주


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
        if d.get("mode") != ROBOT_MODE: return True
        # 잡는 중 or 모션 중 or 대기손 하강 중(arming) 이면 busy
        return bool(d.get("busy") or d.get("is_running") or d.get("arming"))
    except Exception:
        return True


def robot_armed():
    try:
        d = json.loads(urllib.request.urlopen(f"{ROBOT_SERVER}/active_mode", timeout=0.5).read())
        return bool(d.get("armed"))
    except Exception:
        return False


def post_arm():
    """박스 접근 → 대기손 내리기 요청."""
    try:
        req = urllib.request.Request(f"{ROBOT_SERVER}/conveyor_arm", data=b"", method="POST")
        r = urllib.request.urlopen(req, timeout=1.0)
        print(f"[ARM] conveyor_arm: {r.read().decode()[:80]}")
    except Exception as e:
        print(f"[ARM] 실패: {e}")


def post_grab(sm):
    body = json.dumps({
        "type": GRAB_TYPE,
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
    LOST_FRAMES = 12          # 이만큼 연속 미검출이어야 "박스 진짜 사라짐"으로 판단
    miss = 0
    while True:
        time.sleep(0.05)
        if not auto_mode["enabled"]:
            auto_state["in_zone_since"]=None; auto_state["armed_sent"]=False; miss=0; continue
        sm = get_smoothed()
        found = sm.get('_count',0) >= 1 and 'top_center' in sm
        if not found:
            miss += 1
            if miss >= LOST_FRAMES:
                # 진짜 사라짐 → 다음 박스에 재-arm 하도록 리셋
                auto_state["in_zone_since"]=None; auto_state["armed_sent"]=False
            # 짧은 드롭아웃은 무시: dwell 타이머/armed 유지
            continue
        miss = 0
        if robot_busy():
            auto_state["in_zone_since"]=None; continue

        # ── 1단계: 박스 감지되면 대기손 내리기 요청 (박스당 1회) ──
        if not auto_state["armed_sent"]:
            print("[AUTO] 박스 감지 → conveyor_arm 요청")
            post_arm()
            auto_state["armed_sent"]=True
            continue   # 다음 루프부터 arming=busy 로 잡혀 대기

        # ── 2단계: 박스 중심이 중앙 range 진입 → grab ──
        mx, my, mz = camera_to_torso(*sm['top_center'])
        in_zone = (auto_mode["x_min"]<=mx<=auto_mode["x_max"] and
                   auto_mode["y_min"]<=my<=auto_mode["y_max"] and
                   auto_mode["z_min"]<=mz<=auto_mode["z_max"])
        if not in_zone:
            auto_state["in_zone_since"]=None; continue
        if auto_state["in_zone_since"] is None:
            auto_state["in_zone_since"]=time.time(); continue
        if time.time()-auto_state["in_zone_since"] >= auto_mode["dwell_sec"]:
            print("[AUTO] 중앙 range dwell 만족 → POST grab")
            post_grab(sm)
            auto_state["in_zone_since"]=None
            time.sleep(2.0)   # 재발사 방지


# ==========================================
# FastAPI
# ==========================================
@asynccontextmanager
async def lifespan(app: FastAPI):
    global estimator
    print(f"[YOLO] 로드: {YOLO_MODEL} (device={YOLO_DEVICE})")
    estimator = BoxEstimator(YOLO_MODEL, camera_K, conf=YOLO_CONF, device=YOLO_DEVICE)
    estimator.detect(np.zeros((480,640,3),np.uint8),
                     np.zeros((480,640),np.uint16), gravity_cam=GRAVITY_CAM)
    threading.Thread(target=color_reader_loop, daemon=True).start()
    threading.Thread(target=depth_reader_loop, daemon=True).start()
    threading.Thread(target=detect_loop, daemon=True).start()
    threading.Thread(target=auto_monitor_loop, daemon=True).start()
    print(f"[detect_box_conv] http://0.0.0.0:{PORT}/  (mode={ROBOT_MODE})")
    yield


app = FastAPI(title="Detect Box (Conveyor)", lifespan=lifespan)
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
    """현재 박스 좌표 (수동/재감지용) — 안정화된 값."""
    sm = get_smoothed()
    if sm.get('_count',0) < 1 or 'L' not in sm or 'R' not in sm:
        return {"found": False}
    out = {"found": True, "type": GRAB_TYPE,
           "L": [float(v) for v in sm['L']],
           "R": [float(v) for v in sm['R']],
           "top_center": [float(v) for v in sm['top_center']] if 'top_center' in sm else None,
           "box_h": float(sm['box_H'][0]) if 'box_H' in sm else None}
    return out


@app.get("/status")
async def status():
    sm = get_smoothed()
    found = sm.get('_count',0) >= 1 and 'top_center' in sm
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
<title>Detect Box (Conveyor)</title>
<style>
body{font-family:monospace;background:#1a1a1a;color:#fff;padding:20px}
h1{color:#4CAF50;font-size:18px}
#wrap{display:flex;gap:20px}
img{border:2px solid #4CAF50}
.panel{width:280px}
.card{background:#242424;border-radius:8px;padding:14px;margin-bottom:12px}
.card-title{color:#4CAF50;font-size:12px;text-transform:uppercase;margin-bottom:8px}
.info-row{display:flex;justify-content:space-between;font-size:13px;margin:3px 0}
.info-key{color:#666}.info-val{color:#ccc}
input{background:#333;border:1px solid #444;color:#fff;padding:4px;border-radius:4px;width:70px}
button{background:#4CAF50;border:none;color:#000;padding:8px;border-radius:5px;cursor:pointer;width:100%;font-weight:bold;margin-top:8px}
.bar{height:6px;background:#333;border-radius:3px;overflow:hidden;margin-top:8px}
.bar-fill{height:100%;width:0;background:#4CAF50;transition:width .15s}
</style></head><body>
<h1>🏭 Detect Box — Conveyor (50012)</h1>
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
      <div class="card-title">자동 (range 진입 시 잡기)</div>
      <label><input type="checkbox" id="auto" onchange="toggleAuto()" style="width:auto"> 자동 잡기</label>
      <div style="display:grid;grid-template-columns:1fr 1fr;gap:5px;margin-top:8px;font-size:11px;color:#888">
        <div>X min<input id="ax-min" value="0.28"></div><div>X max<input id="ax-max" value="0.42"></div>
        <div>Y min<input id="ay-min" value="-0.15"></div><div>Y max<input id="ay-max" value="0.15"></div>
        <div>Z min<input id="az-min" value="-0.15"></div><div>Z max<input id="az-max" value="0.25"></div>
        <div>dwell<input id="dwell" value="0.3"></div>
      </div>
      <button onclick="applyZone()">range 적용</button>
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
    !d.auto_enabled?'OFF':(d.auto_in_zone?`range 안 ${d.auto_elapsed.toFixed(1)}/${d.auto_dwell}`:'range 밖');
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
