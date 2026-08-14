#!/usr/bin/env python3
# Version: 0.1
"""
loco_test.py — G1 보행 전용 최소 테스트 서버 (포트 50020)

공식 예제(unitree_sdk2_python/example/g1/high_level/g1_loco_client_example.py)
기준으로 작성. 팔/허리는 전혀 건드리지 않는다.

공식 예제와의 대응:
  · ChannelFactoryInitialize(0, networkInterface)   ← 인자로 iface 받음
  · sport_client.SetTimeout(10.0)                   ← 0.0001 아님. 10초.
  · sport_client.Init()
  · sport_client.Move(vx, vy, vyaw)                 ← continous_move 인자 없이 호출
  · sport_client.Damp() / Squat2StandUp() / StandUp2Squat()

사용:
    python3 loco_test.py eth0
    (인터페이스 이름은 `ip addr` 로 확인. 로봇과 연결된 유선 iface)

    브라우저에서 http://localhost:50020/

동작 방식:
    · 방향키를 누르고 있는 동안 브라우저가 100ms 마다 /move 호출
    · 손을 떼면 /stop 2회 전송
    · 서버 워치독: 0.4초간 /move 가 안 오면 자동 정지
"""

import sys
import time
import threading

import uvicorn
from contextlib import asynccontextmanager
from fastapi import FastAPI, HTTPException
from fastapi.responses import HTMLResponse, JSONResponse
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient

PORT = 50020

# 명령 주기 / 워치독
WATCHDOG_SEC = 0.4     # 이 시간 동안 /move 가 안 오면 자동 정지
STOP_REPEAT  = 3       # 정지 명령 반복 횟수 (유실 대비)
STOP_GAP     = 0.05    # 반복 간격

client = None
_last_cmd = 0.0
_active = False
_lock = threading.Lock()


class MoveReq(BaseModel):
    vx: float = 0.0
    vy: float = 0.0
    vyaw: float = 0.0


def do_stop():
    """정지 — Move(0,0,0) 을 몇 번 보내고 StopMove 가 있으면 그것도."""
    global _active
    if client is None:
        return
    with _lock:
        _active = False
        for _ in range(STOP_REPEAT):
            client.Move(0.0, 0.0, 0.0)
            time.sleep(STOP_GAP)
        fn = getattr(client, "StopMove", None)
        if fn is not None:
            try:
                fn()
            except Exception as e:
                print(f"[LOCO] StopMove 실패: {e}")
    print("[LOCO] 정지")


def watchdog_loop():
    """브라우저가 키를 떼면 /move 호출만 끊긴다. 그때 자동 정지."""
    while True:
        time.sleep(0.1)
        if _active and (time.time() - _last_cmd) > WATCHDOG_SEC:
            print("[LOCO] 워치독 — 명령 끊김, 자동 정지")
            do_stop()


@asynccontextmanager
async def lifespan(app: FastAPI):
    global client

    iface = sys.argv[1] if len(sys.argv) > 1 else None
    if iface:
        print(f"[INIT] ChannelFactoryInitialize(0, '{iface}')")
        ChannelFactoryInitialize(0, iface)
    else:
        print("[INIT] ChannelFactoryInitialize(0)  ※ iface 인자 없이 실행됨")
        ChannelFactoryInitialize(0)

    client = LocoClient()
    client.SetTimeout(10.0)      # 공식 예제와 동일
    client.Init()
    print("[INIT] LocoClient 준비 완료")
    print("[INIT] 사용 가능 메서드:")
    print("  " + ", ".join(sorted(m for m in dir(client) if not m.startswith("_"))))

    threading.Thread(target=watchdog_loop, daemon=True).start()
    print(f"[INIT] http://0.0.0.0:{PORT}/")
    yield

    print("[EXIT] 정지 후 종료")
    try:
        do_stop()
    except Exception:
        pass


app = FastAPI(title="G1 Loco Test", lifespan=lifespan)
app.add_middleware(CORSMiddleware, allow_origins=["*"],
                   allow_methods=["*"], allow_headers=["*"])


# ==========================================
# 이동
# ==========================================
@app.post("/move")
async def move(req: MoveReq):
    """브라우저가 키 누르고 있는 동안 반복 호출."""
    global _last_cmd, _active
    if client is None:
        raise HTTPException(503, "LocoClient 미초기화")
    _last_cmd = time.time()
    _active = True
    client.Move(req.vx, req.vy, req.vyaw)     # 공식 예제와 동일한 호출
    return {"ok": True, "vx": req.vx, "vy": req.vy, "vyaw": req.vyaw}


@app.post("/stop")
async def stop():
    do_stop()
    return {"ok": True}


# ==========================================
# 자세 (공식 예제 항목)
# ==========================================
@app.post("/damp")
async def damp():
    do_stop()
    client.Damp()
    return {"ok": True}


@app.post("/stand_up")
async def stand_up():
    """Damp → 0.5초 → Squat2StandUp (공식 예제 id=1 과 동일)"""
    client.Damp()
    time.sleep(0.5)
    client.Squat2StandUp()
    return {"ok": True}


@app.post("/squat")
async def squat():
    client.StandUp2Squat()
    return {"ok": True}


@app.post("/high_stand")
async def high_stand():
    client.HighStand()
    return {"ok": True}


@app.post("/low_stand")
async def low_stand():
    client.LowStand()
    return {"ok": True}


@app.get("/status")
async def status():
    return {"ready": client is not None,
            "active": _active,
            "since_last_cmd": round(time.time() - _last_cmd, 2) if _last_cmd else None}


@app.get("/methods")
async def methods():
    """이 SDK 버전에서 LocoClient 가 실제로 제공하는 메서드 목록."""
    if client is None:
        return {"methods": []}
    return {"methods": sorted(m for m in dir(client) if not m.startswith("_"))}


# ==========================================
# 웹 UI
# ==========================================
HTML = """<!DOCTYPE html><html lang="ko"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>G1 Loco Test</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:ui-monospace,Menlo,monospace;background:#0e1116;color:#c9d4e0;
  padding:24px;display:flex;flex-direction:column;align-items:center;gap:18px}
h1{font-size:18px;color:#3ddc97}
.card{background:#161b22;border:1px solid #2a3340;border-radius:10px;padding:18px;
  width:100%;max-width:420px}
.card-h{font-size:10px;letter-spacing:1.5px;text-transform:uppercase;color:#6b7785;
  margin-bottom:12px}
.spd{display:flex;align-items:center;gap:10px;font-size:12px;color:#6b7785}
.spd input[type=range]{flex:1}
.pad{display:grid;grid-template-columns:repeat(5,1fr);grid-template-rows:repeat(3,56px);
  gap:6px;margin:14px 0}
.pad button{background:#1c232d;border:1px solid #2a3340;color:#c9d4e0;border-radius:8px;
  cursor:pointer;font-size:20px;font-family:inherit;user-select:none;touch-action:none}
.pad button:hover{border-color:#3ddc97}
.pad button.active{background:#3ddc97;color:#05221a;border-color:#3ddc97}
#f{grid-area:1/3/2/4}#bk{grid-area:3/3/4/4}#l{grid-area:2/2/3/3}#r{grid-area:2/4/3/5}
#st{grid-area:2/3/3/4;color:#ff6b6b;font-size:16px}#tl{grid-area:2/1/3/2}#tr{grid-area:2/5/3/6}
.row{display:grid;grid-template-columns:1fr 1fr;gap:8px}
.btn{background:#1c232d;border:1px solid #2a3340;color:#c9d4e0;padding:11px;
  border-radius:8px;cursor:pointer;font-family:inherit;font-size:12px}
.btn:hover{border-color:#4aa8ff;color:#4aa8ff}
.btn.warn{border-color:#5a2b2b;color:#ff6b6b}
#log{background:#0c1117;border:1px solid #2a3340;border-radius:8px;padding:10px;
  font-size:11px;height:130px;overflow-y:auto;color:#6b7785;width:100%;max-width:420px}
.hint{font-size:11px;color:#6b7785;margin-top:10px;line-height:1.6}
</style></head><body>

<h1>G1 Loco Test · 50020</h1>

<div class="card">
  <div class="card-h">Locomotion</div>
  <div class="spd"><span>Speed</span>
    <input type="range" id="spd" min="0.1" max="0.6" step="0.05" value="0.3">
    <span id="spdv" style="color:#3ddc97;width:36px">0.30</span></div>
  <div class="pad">
    <button id="f"  data-cmd="forward">▲</button>
    <button id="tl" data-cmd="turn_left">↺</button>
    <button id="l"  data-cmd="left">◀</button>
    <button id="st" onclick="hardStop()">STOP</button>
    <button id="r"  data-cmd="right">▶</button>
    <button id="tr" data-cmd="turn_right">↻</button>
    <button id="bk" data-cmd="backward">▼</button>
  </div>
  <div class="hint">↑↓←→ 이동 · Q/E 회전 · 스페이스 정지<br>
    누르고 있는 동안만 전진. 떼면 정지.</div>
</div>

<div class="card">
  <div class="card-h">Posture</div>
  <div class="row">
    <button class="btn" onclick="post('/stand_up')">Stand Up</button>
    <button class="btn" onclick="post('/squat')">Squat</button>
    <button class="btn" onclick="post('/high_stand')">High Stand</button>
    <button class="btn" onclick="post('/low_stand')">Low Stand</button>
  </div>
  <div class="row" style="margin-top:8px">
    <button class="btn warn" onclick="post('/damp')">Damp</button>
    <button class="btn" onclick="showMethods()">SDK Methods</button>
  </div>
</div>

<div id="log"></div>

<script>
const SEND_MS = 100;              // 서버 워치독 0.4초보다 충분히 짧게
const spd = document.getElementById('spd');
const spdv = document.getElementById('spdv');
spd.oninput = () => spdv.textContent = (+spd.value).toFixed(2);

const cmap = {
  forward:    () => ({vx:+spd.value, vy:0, vyaw:0}),
  backward:   () => ({vx:-spd.value, vy:0, vyaw:0}),
  left:       () => ({vx:0, vy:+spd.value, vyaw:0}),
  right:      () => ({vx:0, vy:-spd.value, vyaw:0}),
  turn_left:  () => ({vx:0, vy:0, vyaw:+spd.value}),
  turn_right: () => ({vx:0, vy:0, vyaw:-spd.value}),
};

function log(m){
  const el = document.getElementById('log');
  const t = new Date().toLocaleTimeString();
  el.innerHTML = `<div>[${t}] ${m}</div>` + el.innerHTML;
}

let timer = null, activeBtn = null;

function startLoco(cmd, btn){
  if (timer) return;
  activeBtn = btn;
  if (btn) btn.classList.add('active');
  const send = () => fetch('/move', {
    method:'POST', headers:{'Content-Type':'application/json'},
    body: JSON.stringify(cmap[cmd]())
  }).catch(()=>{});
  send();
  timer = setInterval(send, SEND_MS);
  log('▶ ' + cmd);
}

function stopLoco(){
  const was = (timer !== null);
  if (timer){ clearInterval(timer); timer = null; }
  if (activeBtn){ activeBtn.classList.remove('active'); activeBtn = null; }
  if (!was) return;
  const s = () => fetch('/stop', {method:'POST'}).catch(()=>{});
  s(); setTimeout(s, 150);
  log('■ stop');
}

function hardStop(){
  if (timer){ clearInterval(timer); timer = null; }
  if (activeBtn){ activeBtn.classList.remove('active'); activeBtn = null; }
  const s = () => fetch('/stop', {method:'POST'}).catch(()=>{});
  s(); setTimeout(s, 150);
  log('■ STOP (버튼)');
}

function post(path){
  fetch(path, {method:'POST'}).then(r=>r.json())
    .then(()=>log('✓ ' + path)).catch(e=>log('✗ ' + path + ' ' + e));
}

function showMethods(){
  fetch('/methods').then(r=>r.json())
    .then(d=>log('SDK: ' + d.methods.join(', ')));
}

document.querySelectorAll('.pad button[data-cmd]').forEach(b=>{
  const c = b.dataset.cmd;
  b.addEventListener('mousedown', e=>{e.preventDefault(); startLoco(c,b);});
  b.addEventListener('mouseup',   e=>{e.preventDefault(); stopLoco();});
  b.addEventListener('mouseleave',()=>{ if(timer) stopLoco(); });
  b.addEventListener('touchstart',e=>{e.preventDefault(); startLoco(c,b);},{passive:false});
  b.addEventListener('touchend',  e=>{e.preventDefault(); stopLoco();});
  b.addEventListener('touchcancel',e=>{e.preventDefault(); stopLoco();});
});

const km = {'ArrowUp':'forward','ArrowDown':'backward',
            'ArrowLeft':'left','ArrowRight':'right',
            'q':'turn_left','e':'turn_right','Q':'turn_left','E':'turn_right'};
document.addEventListener('keydown', e=>{
  if (e.repeat) return;
  if (e.key === ' '){ e.preventDefault(); hardStop(); return; }
  const c = km[e.key];
  if (c){ e.preventDefault(); startLoco(c, document.querySelector(`[data-cmd="${c}"]`)); }
});
document.addEventListener('keyup', e=>{
  if (km[e.key]){ e.preventDefault(); stopLoco(); }
});

// 창 전환 / 탭 닫힘에도 확실히 정지
document.addEventListener('visibilitychange', ()=>{ if(document.hidden && timer) stopLoco(); });
window.addEventListener('blur', ()=>{ if(timer) stopLoco(); });
window.addEventListener('beforeunload', ()=>{
  if (timer){ clearInterval(timer); timer = null; }
  if (navigator.sendBeacon) navigator.sendBeacon('/stop');
});

log('준비 완료');
</script>
</body></html>"""


@app.get("/", response_class=HTMLResponse)
async def index():
    return HTML


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(f"사용법: python3 {sys.argv[0]} <networkInterface>")
        print("  예: python3 loco_test.py eth0")
        print("  인터페이스 이름은 `ip addr` 로 확인하세요.")
        print("  (인자 없이도 실행은 되지만 공식 예제와 달라집니다)")
    uvicorn.run(app, host="0.0.0.0", port=PORT, timeout_graceful_shutdown=2)
