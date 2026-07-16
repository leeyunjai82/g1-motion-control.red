#!/usr/bin/env python3
# Version: 0.1
"""
detect_marker.py — 마커 인식 전용 서버 (50011)

ik_box.py의 ArUco 인식 부분만 추출. 로봇(arm) 절대 안 건드림.

  · 50001 color MJPEG → ArUco 검출 → 카메라 좌표 tvec/rvec
  · 1초 윈도우 중앙값 안정화
  · 자동 모드: 영역 안 dwell 만족 → robot_server(50003) POST /grab_at
  · GET /pose : 현재 마커 좌표 (robot_server 수동 잡기용)
  · 작은 웹 UI: 영역/dwell 설정, 자동 on/off

robot_server의 active_mode가 "marker"일 때만 POST (그쪽에서 게이트).
"""
import os
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


# ==========================================
# 설정
# ==========================================
PORT          = 50011
RS_STREAM_URL = os.environ.get("RS_STREAM_URL", "http://localhost:50001/video_feed")
ROBOT_SERVER  = os.environ.get("ROBOT_SERVER", "http://localhost:50000")

CAM_FX, CAM_FY = 606.756104, 606.583374
CAM_PPX, CAM_PPY = 316.739441, 258.982391
CAM_DIST = [0.0, 0.0, 0.0, 0.0, 0.0]

ARUCO_DICT_TYPE = cv2.aruco.DICT_4X4_50
MARKER_SIZE     = 0.045

# camera_to_torso (영역 판정용, ik_box 동일)
CAMERA_X, CAMERA_Y, CAMERA_Z = 0.0576235, 0.03003, 0.42987
CAMERA_PITCH_URDF = 0.8307767239493009

AVERAGING_WINDOW_SEC  = 1.0
AVERAGING_MIN_SAMPLES = 3

STREAM_FPS_MAX = 15
STREAM_QUALITY = 70

MARKER_OBJ_PTS = np.array([
    [-MARKER_SIZE/2,  MARKER_SIZE/2, 0],
    [ MARKER_SIZE/2,  MARKER_SIZE/2, 0],
    [ MARKER_SIZE/2, -MARKER_SIZE/2, 0],
    [-MARKER_SIZE/2, -MARKER_SIZE/2, 0],
], dtype=np.float32)


# ==========================================
# 전역
# ==========================================
camera_matrix = np.array([[CAM_FX,0,CAM_PPX],[0,CAM_FY,CAM_PPY],[0,0,1]], dtype=np.float32)
dist_coeffs   = np.array(CAM_DIST, dtype=np.float32)

latest_image    = None
latest_annotated = None
latest_pose     = None       # {'id','tvec','rvec'}
marker_last_seen = 0.0
pose_history = deque(maxlen=120)

image_lock     = threading.Lock()
annotated_lock = threading.Lock()
pose_lock      = threading.Lock()
stream_started = False

aruco_dict = aruco_params = aruco_detector = None

# 자동 모드
auto_mode = {"enabled": False,
             "x_min":0.30,"x_max":0.40,"y_min":-0.15,"y_max":0.15,
             "z_min":-0.10,"z_max":0.20,"dwell_sec":1.0}
auto_state = {"in_zone_since": None}


# ==========================================
# 좌표 변환
# ==========================================
def camera_to_torso(cx, cy, cz):
    cos_p, sin_p = np.cos(CAMERA_PITCH_URDF), np.sin(CAMERA_PITCH_URDF)
    cy_r =  cy*cos_p + cz*sin_p
    cz_r = -cy*sin_p + cz*cos_p
    return float(cz_r+CAMERA_X), float(-cx+CAMERA_Y), float(-cy_r+CAMERA_Z)


# ==========================================
# ArUco
# ==========================================
def init_aruco():
    global aruco_dict, aruco_params, aruco_detector
    aruco_dict   = cv2.aruco.getPredefinedDictionary(ARUCO_DICT_TYPE)
    aruco_params = cv2.aruco.DetectorParameters()
    try:
        aruco_detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
    except AttributeError:
        aruco_detector = None


def draw_marker_box(image, rvec, tvec):
    # 간단 축 표시
    cv2.drawFrameAxes(image, camera_matrix, dist_coeffs, rvec, tvec, MARKER_SIZE*0.7)


def detect_aruco(image):
    global latest_pose, marker_last_seen
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    if aruco_detector:
        corners, ids, _ = aruco_detector.detectMarkers(gray)
    else:
        corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)

    best = None
    if ids is not None:
        for i, mid in enumerate(ids.flatten()):
            c = corners[i][0]
            ok, rvec, tvec = cv2.solvePnP(MARKER_OBJ_PTS, c, camera_matrix,
                                          dist_coeffs, flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if not ok:
                continue
            rvec = rvec.flatten(); tvec = tvec.flatten()
            draw_marker_box(image, rvec, tvec)
            if best is None:
                best = {'id': int(mid), 'rvec': rvec, 'tvec': tvec}

    # 1초 중앙값 안정화
    if best is not None:
        now = time.time()
        pose_history.append({'time':now,'id':best['id'],
                             'rvec':best['rvec'].copy(),'tvec':best['tvec'].copy()})
        cutoff = now - AVERAGING_WINDOW_SEC
        recent = [p for p in pose_history if p['time']>=cutoff and p['id']==best['id']]
        if len(recent) >= AVERAGING_MIN_SAMPLES:
            best = {'id':best['id'],
                    'tvec':np.median([p['tvec'] for p in recent],axis=0),
                    'rvec':np.median([p['rvec'] for p in recent],axis=0)}
        with pose_lock:
            latest_pose = best
            marker_last_seen = time.time()
    return image


def is_visible(threshold=0.5):
    return (time.time() - marker_last_seen) < threshold


# ==========================================
# 스트림 수신
# ==========================================
def stream_reader_loop():
    global latest_image, stream_started
    while True:
        try:
            req = urllib.request.urlopen(RS_STREAM_URL, timeout=5)
            stream_started = True
            print("[STREAM] 연결 성공")
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
                        with image_lock:
                            latest_image = img
        except Exception as e:
            print(f"[STREAM] 오류: {e}")
            stream_started = False
            time.sleep(2.0)


def detect_loop():
    global latest_annotated
    while True:
        with image_lock:
            img = None if latest_image is None else latest_image.copy()
        if img is None:
            time.sleep(0.05); continue
        detect_aruco(img)
        with annotated_lock:
            latest_annotated = img
        time.sleep(1.0/30.0)


# ==========================================
# 자동 모니터 — robot_server에 POST
# ==========================================
def post_grab(pose):
    body = json.dumps({
        "type": "marker",
        "tvec": [float(v) for v in pose['tvec']],
        "rvec": [float(v) for v in pose['rvec']],
    }).encode()
    try:
        req = urllib.request.Request(f"{ROBOT_SERVER}/grab_at", data=body,
                                     headers={"Content-Type":"application/json"})
        r = urllib.request.urlopen(req, timeout=1.0)
        print(f"[POST] grab_at: {r.read().decode()[:80]}")
    except Exception as e:
        print(f"[POST] 실패: {e}")


def robot_busy():
    """robot_server가 busy거나 모드가 marker 아니면 True."""
    try:
        d = json.loads(urllib.request.urlopen(f"{ROBOT_SERVER}/active_mode", timeout=0.5).read())
        if d.get("mode") != "marker": return True
        if d.get("busy") or d.get("is_running"): return True
        return False
    except Exception:
        return True


def auto_monitor_loop():
    while True:
        time.sleep(0.1)
        if not auto_mode["enabled"]:
            auto_state["in_zone_since"] = None; continue
        with pose_lock:
            pose = latest_pose
        if pose is None or not is_visible(0.3):
            auto_state["in_zone_since"] = None; continue
        if robot_busy():
            auto_state["in_zone_since"] = None; continue

        mx, my, mz = camera_to_torso(*pose['tvec'])
        in_zone = (auto_mode["x_min"]<=mx<=auto_mode["x_max"] and
                   auto_mode["y_min"]<=my<=auto_mode["y_max"] and
                   auto_mode["z_min"]<=mz<=auto_mode["z_max"])
        if not in_zone:
            auto_state["in_zone_since"] = None; continue
        if auto_state["in_zone_since"] is None:
            auto_state["in_zone_since"] = time.time(); continue
        if time.time() - auto_state["in_zone_since"] >= auto_mode["dwell_sec"]:
            print("[AUTO] dwell 만족 → POST grab")
            post_grab(pose)
            auto_state["in_zone_since"] = None
            time.sleep(2.0)   # 연속 트리거 방지


# ==========================================
# FastAPI
# ==========================================
@asynccontextmanager
async def lifespan(app: FastAPI):
    init_aruco()
    threading.Thread(target=stream_reader_loop, daemon=True).start()
    threading.Thread(target=detect_loop, daemon=True).start()
    threading.Thread(target=auto_monitor_loop, daemon=True).start()
    print(f"[detect_marker] http://0.0.0.0:{PORT}/")
    yield


app = FastAPI(title="Detect Marker", lifespan=lifespan)
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
    """현재 마커 좌표 (robot_server 수동 잡기용)."""
    with pose_lock:
        p = latest_pose
    if p is None or not is_visible(0.5):
        return {"found": False}
    mx, my, mz = camera_to_torso(*p['tvec'])
    return {"found": True, "type": "marker",
            "id": int(p['id']),
            "tvec": [float(v) for v in p['tvec']],
            "rvec": [float(v) for v in p['rvec']],
            "torso_cm": [round(mx*100,1), round(my*100,1), round(mz*100,1)]}


@app.get("/status")
async def status():
    with pose_lock:
        p = latest_pose
    vis = p is not None and is_visible(0.5)
    in_zone_since = auto_state.get("in_zone_since")
    elapsed = (time.time()-in_zone_since) if in_zone_since else 0.0
    out = {"found": vis, "marker_id": int(p['id']) if vis else None,
           "stream_started": stream_started,
           "auto_enabled": auto_mode["enabled"],
           "auto_in_zone": in_zone_since is not None,
           "auto_elapsed": round(elapsed,2),
           "auto_dwell": auto_mode["dwell_sec"]}
    if vis:
        mx, my, mz = camera_to_torso(*p['tvec'])
        out["torso"] = {"x":round(mx,3),"y":round(my,3),"z":round(mz,3)}
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
        if v is not None: auto_mode[k] = v
    auto_state["in_zone_since"] = None
    return {"success": True, "config": auto_mode}


@app.get("/", response_class=HTMLResponse)
async def index():
    return HTML_PAGE


HTML_PAGE = """<!DOCTYPE html><html><head><meta charset="utf-8">
<title>Detect Marker</title>
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
<h1>🎯 Detect Marker (50011)</h1>
<div id="wrap">
  <img src="/video_feed" width="640" height="480">
  <div class="panel">
    <div class="card">
      <div class="card-title">마커</div>
      <div id="mstatus">대기...</div>
      <div class="info-row"><span class="info-key">torso X</span><span class="info-val" id="tx">-</span></div>
      <div class="info-row"><span class="info-key">torso Y</span><span class="info-val" id="ty">-</span></div>
      <div class="info-row"><span class="info-key">torso Z</span><span class="info-val" id="tz">-</span></div>
    </div>
    <div class="card">
      <div class="card-title">자동 모드</div>
      <label><input type="checkbox" id="auto" onchange="toggleAuto()" style="width:auto"> 자동 잡기</label>
      <div style="display:grid;grid-template-columns:1fr 1fr;gap:5px;margin-top:8px;font-size:11px;color:#888">
        <div>X min<input id="ax-min" value="0.30"></div><div>X max<input id="ax-max" value="0.40"></div>
        <div>Y min<input id="ay-min" value="-0.15"></div><div>Y max<input id="ay-max" value="0.15"></div>
        <div>Z min<input id="az-min" value="-0.10"></div><div>Z max<input id="az-max" value="0.20"></div>
        <div>dwell<input id="dwell" value="1.0"></div>
      </div>
      <button onclick="applyZone()">영역 적용</button>
      <div class="bar"><div class="bar-fill" id="bar"></div></div>
      <div id="amsg" style="font-size:11px;color:#666;margin-top:6px">대기</div>
    </div>
  </div>
</div>
<script>
function poll(){fetch('/status').then(r=>r.json()).then(d=>{
  document.getElementById('mstatus').textContent=d.found?`ID ${d.marker_id} ✓`:'대기...';
  if(d.torso){document.getElementById('tx').textContent=d.torso.x.toFixed(3);
    document.getElementById('ty').textContent=d.torso.y.toFixed(3);
    document.getElementById('tz').textContent=d.torso.z.toFixed(3);}
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
