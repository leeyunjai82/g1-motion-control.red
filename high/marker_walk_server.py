"""
marker_walk_server.py — 마커 추종 보행 웹 콘솔 (포트 50040)

detect_marker(50011) 의 /pose 를 폐루프 조향에 사용.
웹에서 시작/정지, 파라미터 조정, 카메라·상태 실시간 확인.

실행
  python marker_walk_server.py <iface>       # 예: enp46s0
  브라우저: http://localhost:50040/

전제
  - detect_marker.py(50011) 실행 중
  - 로봇 FSM 501, 보행 가능 상태. hold/release 는 여기서 안 건드림
  - 추종 중에는 robot_server 웹 방향키 사용 금지 (명령 이중 송신)
"""

import json
import math
import sys
import threading
import time
import urllib.request

import uvicorn
from fastapi import FastAPI
from fastapi.responses import HTMLResponse

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient

POSE_URL = "http://localhost:50011/pose"
PORT = 50040

# ---- 파라미터 (웹에서 변경 가능) ----
P = {
    "vx_max": 0.25,     # 전진 상한 m/s
    "vx_min": 0.10,     # 근접 최저 m/s
    "kp_yaw": 0.8,      # 조향 게인
    "vyaw_max": 0.4,    # 조향 상한 rad/s
    "yaw_sign": 1,      # 마커 왼쪽인데 오른쪽으로 틀면 -1
    "stop_dist": 0.8,   # 도착 거리 m
    "slow_dist": 1.5,   # 감속 시작 m
    "lost_stop": 1.5,   # 미검출 정지 s
    "timeout": 60.0,    # 최대 추종 시간 s
}

S = {  # 상태 (웹 표시용)
    "state": "idle",    # idle | following | arrived | lost | timeout | stopped
    "found": False, "mx": 0.0, "my": 0.0,
    "vx": 0.0, "vyaw": 0.0, "t_run": 0.0,
}

loco = None
_run = threading.Event()
_lock = threading.Lock()


def get_pose():
    try:
        d = json.loads(urllib.request.urlopen(POSE_URL, timeout=0.3).read())
        if not d.get("found"):
            return False, 0.0, 0.0
        cm = d["torso_cm"]
        return True, cm[0] / 100.0, cm[1] / 100.0
    except Exception:
        return False, 0.0, 0.0


def follow_loop():
    dt = 0.1
    t_start = time.time()
    lost_since = None
    S["state"] = "following"
    try:
        while _run.is_set():
            t0 = time.time()
            S["t_run"] = round(t0 - t_start, 1)
            if S["t_run"] > P["timeout"]:
                S["state"] = "timeout"; break

            found, mx, my = get_pose()
            S["found"], S["mx"], S["my"] = found, round(mx, 2), round(my, 2)

            if not found:
                if lost_since is None:
                    lost_since = t0
                elif t0 - lost_since > P["lost_stop"]:
                    S["state"] = "lost"; break
                loco.Move(0, 0, 0)
                S["vx"], S["vyaw"] = 0.0, 0.0
                time.sleep(dt); continue
            lost_since = None

            if mx <= P["stop_dist"]:
                S["state"] = "arrived"; break

            bearing = math.atan2(my, mx)
            vyaw = max(-P["vyaw_max"], min(P["vyaw_max"],
                       P["yaw_sign"] * P["kp_yaw"] * bearing))
            if mx < P["slow_dist"]:
                vx = P["vx_min"] + (P["vx_max"] - P["vx_min"]) * \
                     (mx - P["stop_dist"]) / max(0.01, P["slow_dist"] - P["stop_dist"])
            else:
                vx = P["vx_max"]
            vx = max(P["vx_min"], min(P["vx_max"], vx))

            loco.Move(vx, 0.0, vyaw)
            S["vx"], S["vyaw"] = round(vx, 2), round(vyaw, 2)
            time.sleep(max(0, dt - (time.time() - t0)))
        else:
            S["state"] = "stopped"
    finally:
        _run.clear()
        try:
            loco.StopMove()
        except Exception:
            pass
        S["vx"], S["vyaw"] = 0.0, 0.0


app = FastAPI(title="Marker Walk")


@app.get("/status")
async def status():
    return {**S, "params": P, "running": _run.is_set()}


@app.post("/start")
async def start():
    with _lock:
        if _run.is_set():
            return {"ok": False, "reason": "이미 추종 중"}
        found, mx, my = get_pose()
        if not found:
            return {"ok": False, "reason": "마커 미검출 - 위치/50011 확인"}
        _run.set()
        threading.Thread(target=follow_loop, daemon=True).start()
    return {"ok": True, "mx": round(mx, 2), "my": round(my, 2)}


@app.post("/stop")
async def stop():
    _run.clear()
    try:
        loco.StopMove()
    except Exception:
        pass
    S["state"] = "stopped"
    return {"ok": True}


@app.post("/params")
async def set_params(body: dict):
    for k, v in body.items():
        if k in P:
            P[k] = type(P[k])(v)
    return {"ok": True, "params": P}


HTML = """<!DOCTYPE html><html><head><meta charset="utf-8"><title>Marker Walk</title>
<style>
body{font-family:sans-serif;background:#101418;color:#dde;margin:0;padding:16px}
.row{display:flex;gap:16px;flex-wrap:wrap}
.card{background:#1a2028;border:1px solid #2a3340;border-radius:8px;padding:12px;min-width:300px}
h3{margin:4px 0 10px;font-size:14px;color:#7fd}
img{width:480px;max-width:100%;border-radius:6px}
button{font-size:15px;padding:10px 18px;border:none;border-radius:6px;cursor:pointer;margin-right:8px}
#btn-start{background:#1f8f5f;color:#fff}#btn-stop{background:#a33;color:#fff}
table{font-size:13px;border-collapse:collapse}td{padding:3px 10px 3px 0}
.stat{font-size:22px;font-weight:700;margin:6px 0}
input{width:70px;background:#0d1116;color:#dde;border:1px solid #2a3340;border-radius:4px;padding:4px}
.hint{color:#889;font-size:12px}
</style></head><body>
<div class="row">
 <div class="card"><h3>Camera (50011)</h3>
  <img src="http://__HOST__:50011/video_feed">
 </div>
 <div class="card"><h3>Control</h3>
  <div class="stat" id="state">-</div>
  <button id="btn-start" onclick="go('/start')">▶ 추종 시작</button>
  <button id="btn-stop" onclick="go('/stop')">⏹ 정지</button>
  <table style="margin-top:10px">
   <tr><td>마커</td><td id="found">-</td></tr>
   <tr><td>전방 거리</td><td id="mx">-</td></tr>
   <tr><td>좌우 오프셋</td><td id="my">-</td></tr>
   <tr><td>vx / vyaw</td><td id="cmd">-</td></tr>
   <tr><td>경과</td><td id="trun">-</td></tr>
  </table>
  <div class="hint" style="margin-top:8px">
   부호 확인: 마커를 왼쪽에 두고 시작 → 왼쪽으로 틀면 정상, 반대면 yaw_sign=-1
  </div>
 </div>
 <div class="card"><h3>Params</h3>
  <table id="ptab"></table>
  <button style="margin-top:8px;background:#345;color:#fff" onclick="saveParams()">적용</button>
 </div>
</div>
<script>
const host=location.hostname;
document.querySelector('img').src=`http://${host}:50011/video_feed`;
const PK=["vx_max","vx_min","kp_yaw","vyaw_max","yaw_sign","stop_dist","slow_dist","lost_stop","timeout"];
function buildParams(p){document.getElementById('ptab').innerHTML=
 PK.map(k=>`<tr><td>${k}</td><td><input id="p-${k}" value="${p[k]}"></td></tr>`).join('');}
async function saveParams(){const b={};PK.forEach(k=>b[k]=parseFloat(document.getElementById('p-'+k).value));
 await fetch('/params',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(b)});}
async function go(u){const r=await fetch(u,{method:'POST'});const d=await r.json();
 if(!d.ok&&d.reason)alert(d.reason);}
let built=false;
async function tick(){try{const d=await(await fetch('/status')).json();
 document.getElementById('state').textContent=d.state+(d.running?' (실행 중)':'');
 document.getElementById('found').textContent=d.found?'검출':'미검출';
 document.getElementById('mx').textContent=d.mx+' m';
 document.getElementById('my').textContent=d.my+' m';
 document.getElementById('cmd').textContent=d.vx+' / '+d.vyaw;
 document.getElementById('trun').textContent=d.t_run+' s';
 if(!built){buildParams(d.params);built=true;}
}catch(e){}}
setInterval(tick,300);tick();
</script></body></html>"""


@app.get("/", response_class=HTMLResponse)
async def index():
    return HTML


if __name__ == "__main__":
    iface = sys.argv[1] if len(sys.argv) > 1 else "eth0"
    ChannelFactoryInitialize(0, iface)
    loco = LocoClient()
    loco.SetTimeout(10.0)
    loco.Init()
    print(f"[marker_walk_server] http://0.0.0.0:{PORT}/")
    uvicorn.run(app, host="0.0.0.0", port=PORT)
