"""
pilot_server.py — SLAM/외부 프로그램 재현용 파일럿 (포트 50040)
Version: 0.1

목적:
  실제 시나리오에서 "다른 소프트웨어(SLAM)"가 하는 역할을 재현한다.
    1) 자기 LocoClient 로 방향키 조작해 테이블 근처까지 이동
    2) 이동 명령 송신을 완전히 멈춘 뒤 mission_server /start 호출
       → 미션이 마커 보고 세부 정렬·파지·후진까지 수행
    3) 미션 완료(done) 후 다시 방향키로 박스 든 채 왔다갔다
    4) 원하는 위치에서 mission_server /place 호출 → 미션이 내려놓음

인계 규약 (이 파일이 그대로 구현):
  - 미션이 진행 중(running)일 때는 이 서버의 방향키 입력을 거부한다
    → "동시에 이동 명령을 내는 소스는 하나" 규칙
  - /mission/start 호출 직전에 자기 loco 를 먼저 정지한다

전제: mission_server(50030) + 그 하위 스택이 떠 있어야 함.

API:
  POST /loco/move {vx,vy,vyaw}   방향 이동 (50ms 간격 반복 호출 방식)
  POST /loco/stop
  POST /mission/start            자기 loco 정지 → 미션 시작
  POST /mission/place            미션에 내려놓기 요청
  POST /mission/stop             미션 중단
  GET  /status                   미션 상태 포함
"""

import os
import sys
import json
import time
import urllib.request
import urllib.error

import uvicorn
from fastapi import FastAPI
from fastapi.responses import HTMLResponse, JSONResponse
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from contextlib import asynccontextmanager

current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_dir)

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from ctrl.arm_controller_wrapper import LocoClientWrapper

PORT = 50040
MISSION = "http://localhost:50030"

loco = None


def _mission(path, method="POST", timeout=5.0):
    req = urllib.request.Request(MISSION + path, data=b"{}" if method == "POST" else None,
                                 headers={"Content-Type": "application/json"},
                                 method=method)
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return json.loads(r.read())


def mission_status():
    try:
        return _mission("/status", method="GET", timeout=1.0)
    except Exception:
        return {"state": "unreachable", "running": False}


class MoveReq(BaseModel):
    vx: float = 0.0
    vy: float = 0.0
    vyaw: float = 0.0


@asynccontextmanager
async def lifespan(app):
    global loco
    print("[pilot_server] 시작 (SLAM 재현용)")
    ChannelFactoryInitialize(0)
    loco = LocoClientWrapper()
    st = mission_status()
    print(f"[pilot_server] mission_server: {st.get('state')}")
    print(f"[pilot_server] 준비 완료  http://localhost:{PORT}/")
    yield
    try:
        loco.stop()
    except Exception:
        pass
    print("[pilot_server] 종료")


app = FastAPI(title="G1 Pilot (SLAM 재현)", lifespan=lifespan)
app.add_middleware(CORSMiddleware, allow_origins=["*"],
                   allow_methods=["*"], allow_headers=["*"])


@app.get("/status")
async def status():
    return {"mission": mission_status()}


@app.post("/loco/move")
async def loco_move(req: MoveReq):
    # 인계 규약: 미션 진행 중엔 이동 명령을 내지 않는다
    ms = mission_status()
    if ms.get("running"):
        return JSONResponse({"ok": False, "reason": "미션 진행 중 — 이동 금지",
                             "mission": ms.get("state")}, status_code=409)
    loco.move(req.vx, req.vy, req.vyaw)
    return {"ok": True}


@app.post("/loco/stop")
async def loco_stop():
    loco.stop()
    return {"ok": True}


@app.post("/mission/start")
async def mission_start():
    """자기 loco 정지 → 미션 시작 (실제 SLAM 이 지켜야 할 순서 그대로)."""
    loco.stop()
    time.sleep(0.3)          # 정지 확정 후 인계
    try:
        d = _mission("/start")
    except urllib.error.HTTPError as e:
        return JSONResponse(json.loads(e.read()), status_code=e.code)
    except Exception as e:
        return JSONResponse({"ok": False, "reason": f"mission_server 호출 실패: {e}"})
    return d


@app.post("/mission/place")
async def mission_place():
    loco.stop()
    time.sleep(0.3)
    try:
        d = _mission("/place")
    except urllib.error.HTTPError as e:
        return JSONResponse(json.loads(e.read()), status_code=e.code)
    except Exception as e:
        return JSONResponse({"ok": False, "reason": f"mission_server 호출 실패: {e}"})
    return d


@app.post("/mission/stop")
async def mission_stop():
    try:
        return _mission("/stop")
    except Exception as e:
        return JSONResponse({"ok": False, "reason": str(e)})


# ==========================================
# 웹 UI — 방향키 패드 + 미션 버튼
# ==========================================
PAGE = """<!DOCTYPE html><html lang="ko"><head><meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>G1 Pilot</title>
<style>
body{margin:0;background:#0e1116;color:#c9d4e0;font-family:ui-monospace,Menlo,monospace;
  min-height:100vh;display:flex;flex-direction:column;align-items:center;gap:16px;padding:24px}
h1{font-size:17px;color:#4aa8ff;margin:0}
.state{font-size:14px;padding:9px 18px;border:1px solid #2a3340;border-radius:10px;background:#161b22}
.state b{color:#3ddc97}
.pad{display:grid;grid-template-columns:repeat(5,64px);grid-template-rows:repeat(3,54px);gap:6px}
.pad button{background:#1c232d;border:1px solid #2a3340;color:#c9d4e0;border-radius:9px;
  font-size:20px;cursor:pointer;font-family:inherit}
.pad button.active{background:#3ddc97;color:#05221a}
#f{grid-area:1/3}#bk{grid-area:3/3}#l{grid-area:2/2}#r{grid-area:2/4}
#st{grid-area:2/3;color:#ff6b6b}#tl{grid-area:2/1}#tr{grid-area:2/5}
.spd{display:flex;gap:10px;align-items:center;font-size:12px;color:#6b7785;width:340px}
.spd input{flex:1}
.mrow{display:flex;gap:10px}
.mbtn{font:inherit;border:none;border-radius:10px;cursor:pointer;padding:16px 26px;font-size:16px;font-weight:700}
#m-start{background:#3ddc97;color:#05221a}
#m-place{background:#4aa8ff;color:#04121f}
#m-stop{background:#ff6b6b;color:#2a0505}
.hint{color:#6b7785;font-size:11px;text-align:center;line-height:1.7}
</style></head><body>
<h1>G1 Pilot · SLAM 재현 (방향키 + 미션 호출)</h1>
<div class="state">mission: <b id="mstate">-</b> <span id="mdetail"></span></div>
<div class="spd"><span>Speed</span><input type="range" id="speed" min="0.1" max="0.4" step="0.05" value="0.25">
<span id="spdv" style="color:#3ddc97">0.25</span></div>
<div class="pad">
  <button id="f" data-cmd="forward">▲</button>
  <button id="tl" data-cmd="turn_left">↺</button>
  <button id="l" data-cmd="left">◀</button>
  <button id="st" onclick="stopLoco()">■</button>
  <button id="r" data-cmd="right">▶</button>
  <button id="tr" data-cmd="turn_right">↻</button>
  <button id="bk" data-cmd="backward">▼</button>
</div>
<div class="mrow">
  <button class="mbtn" id="m-start" onclick="mission('/mission/start')">▶ 미션 시작 (파지)</button>
  <button class="mbtn" id="m-place" onclick="mission('/mission/place')">⬇ 내려놓기</button>
  <button class="mbtn" id="m-stop" onclick="mission('/mission/stop')">■ 미션 중단</button>
</div>
<div class="hint">↑↓←→ 이동 · Q,E 회전 · Esc 정지<br>
시나리오: 방향키로 근처 이동 → 미션 시작(마커 정렬·파지·후진) → 방향키로 운반 → 내려놓기<br>
미션 진행 중엔 방향키가 잠깁니다 (이동 명령 단일 소스 규칙)</div>
<script>
const speed=document.getElementById('speed'),spdv=document.getElementById('spdv');
speed.oninput=()=>spdv.textContent=(+speed.value).toFixed(2);
const cmap={forward:()=>({vx:+speed.value,vy:0,vyaw:0}),backward:()=>({vx:-speed.value,vy:0,vyaw:0}),
  left:()=>({vx:0,vy:+speed.value,vyaw:0}),right:()=>({vx:0,vy:-speed.value,vyaw:0}),
  turn_left:()=>({vx:0,vy:0,vyaw:+speed.value}),turn_right:()=>({vx:0,vy:0,vyaw:-speed.value})};
let lt=null,ab=null,lockToast=false;
function startLoco(c,b){if(lt)return;ab=b;if(b)b.classList.add('active');
  const s=()=>fetch('/loco/move',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify(cmap[c]())}).then(r=>{if(r.status===409&&!lockToast){lockToast=true;
      alert('미션 진행 중 — 이동 잠금');stopLoco();}}).catch(()=>{});
  s();lt=setInterval(s,50);}
function stopLoco(){lockToast=false;if(lt){clearInterval(lt);lt=null;}
  if(ab){ab.classList.remove('active');ab=null;}
  fetch('/loco/stop',{method:'POST'}).catch(()=>{});}
document.querySelectorAll('.pad button[data-cmd]').forEach(b=>{const c=b.dataset.cmd;
  b.addEventListener('mousedown',e=>{e.preventDefault();startLoco(c,b);});
  b.addEventListener('mouseup',e=>{e.preventDefault();stopLoco();});
  b.addEventListener('mouseleave',()=>stopLoco());
  b.addEventListener('touchstart',e=>{e.preventDefault();startLoco(c,b);},{passive:false});
  b.addEventListener('touchend',e=>{e.preventDefault();stopLoco();});});
const km={'ArrowUp':'forward','ArrowDown':'backward','ArrowLeft':'left','ArrowRight':'right',
  'q':'turn_left','e':'turn_right'};
document.addEventListener('keydown',e=>{if(e.repeat)return;
  if(['INPUT','SELECT'].includes(e.target.tagName))return;
  const c=km[e.key];if(c){e.preventDefault();startLoco(c,document.querySelector(`[data-cmd="${c}"]`));}
  if(e.key==='Escape'){e.preventDefault();stopLoco();}});
document.addEventListener('keyup',e=>{if(km[e.key]){e.preventDefault();stopLoco();}});
window.addEventListener('beforeunload',stopLoco);
async function mission(p){stopLoco();
  try{const r=await fetch(p,{method:'POST'});const d=await r.json();
    if(d.reason)alert(d.reason);}catch(e){alert(e.message);}}
setInterval(async()=>{try{const d=await(await fetch('/status')).json();
  const m=d.mission||{};
  document.getElementById('mstate').textContent=(m.state||'-')+(m.running?' ●':'');
  document.getElementById('mdetail').textContent=(m.step&&m.step!=='-'?' · '+m.step:'')+(m.detail?' · '+m.detail:'');
}catch(e){}},400);
</script></body></html>"""


@app.get("/", include_in_schema=False)
async def index():
    return HTMLResponse(PAGE)


if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=PORT)
