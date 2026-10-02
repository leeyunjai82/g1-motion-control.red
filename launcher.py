#!/usr/bin/env python3
"""
launcher.py — start_fsm.sh / start_robot.sh 를 버튼으로 실행하는 웹 (포트 50080)

  ./launcher.sh             (start_fsm.sh 와 같이 sudo 로 tv 환경 python 실행)
  → http://<robot-ip>:50080/

버튼
  · Stand / Sit / Damp : ./start_fsm.sh <cmd>   (한 번에 하나만 실행)
  · Robot 시작 / 정지   : ./start_robot.sh 실행 / SIGTERM(= Ctrl+C 와 같은 종료 시퀀스)

권한
  · launcher 는 root 로 돈다 (launcher.sh 의 sudo) → start_fsm.sh 안의 sudo 가 비밀번호 없이 통과.
  · start_robot.sh 는 sudo 를 실행한 원래 사용자(SUDO_USER)로 내려서 실행한다.
    root 로 띄우면 logs/ 등 파일 소유자가 root 가 되어 수동 실행 시 쓰기 실패.
주의
  · 웹 버튼은 비상정지가 아니다. 물리 E-STOP / 리모컨을 항상 손에 둘 것.
"""

import os
import pwd
import signal
import subprocess
import threading
import time
from collections import deque

import uvicorn
from contextlib import asynccontextmanager
from fastapi import FastAPI, HTTPException
from fastapi.responses import HTMLResponse

PORT = 50080
ROOT = os.path.dirname(os.path.abspath(__file__))
LOG_DIR = os.path.join(ROOT, "logs")

# sudo 로 실행됐으면 원래 사용자 정보 (start_robot.sh 를 이 사용자로 실행)
SUDO_USER = os.environ.get("SUDO_USER") if os.geteuid() == 0 else None
USER_PW = pwd.getpwnam(SUDO_USER) if SUDO_USER else None


def _own(path):
    """root 로 만든 파일/폴더를 원래 사용자 소유로."""
    if USER_PW:
        try:
            os.chown(path, USER_PW.pw_uid, USER_PW.pw_gid)
        except OSError:
            pass


os.makedirs(LOG_DIR, exist_ok=True)
_own(LOG_DIR)


def _as_user():
    """자식 프로세스를 원래 사용자 권한으로 (그룹 포함 — video/plugdev 등 장치 접근)."""
    os.initgroups(USER_PW.pw_name, USER_PW.pw_gid)
    os.setgid(USER_PW.pw_gid)
    os.setuid(USER_PW.pw_uid)


def _user_env():
    env = dict(os.environ)
    env.update(HOME=USER_PW.pw_dir, USER=USER_PW.pw_name, LOGNAME=USER_PW.pw_name)
    for k in ("SUDO_USER", "SUDO_UID", "SUDO_GID", "SUDO_COMMAND"):
        env.pop(k, None)
    return env

FSM_CMDS = ("stand", "sit", "damp")
ROBOT_STOP_WAIT = 20.0      # start_robot.sh 종료 시퀀스(최대 8초 + sweep) 여유


class Job:
    """자식 프로세스 1개 + 출력 버퍼."""

    def __init__(self, name):
        self.name = name
        self.proc = None
        self.label = ""
        self.started = 0.0
        self.ended = 0.0
        self.rc = None
        self.lines = deque(maxlen=300)
        self.lock = threading.Lock()

    def running(self):
        return self.proc is not None and self.proc.poll() is None

    def start(self, args, label, as_user=False):
        with self.lock:
            if self.running():
                raise HTTPException(409, f"{self.name} 실행 중: {self.label}")
            self.lines.clear()
            self.label, self.rc = label, None
            self.started, self.ended = time.time(), 0.0
            day = time.strftime("%Y%m%d")
            logpath = os.path.join(LOG_DIR, f"launcher_{self.name}_{day}.log")
            logf = open(logpath, "a", encoding="utf-8")
            _own(logpath)
            drop = as_user and USER_PW is not None
            logf.write(f"\n===== {time.strftime('%Y-%m-%d %H:%M:%S')} {label} =====\n")
            self.proc = subprocess.Popen(
                args, cwd=ROOT, stdin=subprocess.DEVNULL,
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                start_new_session=True, text=True, bufsize=1,
                preexec_fn=_as_user if drop else None,
                env=_user_env() if drop else None)
            threading.Thread(target=self._pump, args=(self.proc, logf), daemon=True).start()

    def _pump(self, proc, logf):
        for line in proc.stdout:
            line = line.rstrip("\n")
            self.lines.append(f"[{time.strftime('%H:%M:%S')}] {line}")
            logf.write(line + "\n")
            logf.flush()
        proc.wait()
        self.rc, self.ended = proc.returncode, time.time()
        self.lines.append(f"[{time.strftime('%H:%M:%S')}] --- 종료 (rc={self.rc}) ---")
        if self.rc != 0 and any(("password" in l or "terminal is required" in l)
                                for l in self.lines):
            self.lines.append("[launcher] sudo 비밀번호 요구로 실패 — launcher 를 ./launcher.sh 로 "
                              "(sudo) 실행했는지 확인")
        logf.close()

    def state(self):
        return {"running": self.running(), "label": self.label, "rc": self.rc,
                "started": self.started, "ended": self.ended,
                "elapsed": round((time.time() if self.running() else (self.ended or time.time()))
                                 - self.started, 1) if self.started else 0,
                "log": list(self.lines)[-120:]}


fsm = Job("fsm")
robot = Job("robot")


@asynccontextmanager
async def lifespan(app: FastAPI):
    yield
    # launcher 종료(Ctrl+C) 시 로봇 서버도 정상 종료 시퀀스로 정지.
    # 남겨두면 다음 start_robot.sh 의 sweep 이 SIGKILL 로 죽여 arm 제어권 반납이 생략된다.
    if robot.running():
        print("[launcher] 종료 — start_robot.sh 정지 중...")
        _stop_robot()


app = FastAPI(title="G1 Launcher", lifespan=lifespan)


@app.post("/fsm/{cmd}")
async def run_fsm(cmd: str):
    if cmd not in FSM_CMDS:
        raise HTTPException(400, f"cmd 는 {FSM_CMDS}")
    fsm.start(["bash", os.path.join(ROOT, "start_fsm.sh"), cmd], f"start_fsm.sh {cmd}")
    return {"ok": True}


@app.post("/robot/start")
async def robot_start():
    robot.start(["bash", os.path.join(ROOT, "start_robot.sh")], "start_robot.sh", as_user=True)
    return {"ok": True}


def _stop_robot():
    p = robot.proc
    if p is None or p.poll() is not None:
        return
    # bash 에만 SIGTERM → start_robot.sh 의 trap cleanup (Ctrl+C 와 동일 시퀀스)
    p.send_signal(signal.SIGTERM)
    try:
        p.wait(ROBOT_STOP_WAIT)
    except subprocess.TimeoutExpired:
        robot.lines.append("[launcher] 종료 지연 — 프로세스 그룹 SIGKILL")
        try:
            os.killpg(p.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass


@app.post("/robot/stop")
async def robot_stop():
    if not robot.running():
        return {"ok": True, "already": True}
    threading.Thread(target=_stop_robot, daemon=True).start()
    return {"ok": True}


@app.get("/status")
async def status():
    return {"fsm": fsm.state(), "robot": robot.state()}


@app.get("/", response_class=HTMLResponse)
async def index():
    return HTML


HTML = r"""<!DOCTYPE html>
<html lang="ko"><head><meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>G1 Launcher</title>
<style>
:root{--bg:#0e1116;--panel:#161b22;--panel2:#1c232d;--line:#2a3340;--ink:#c9d4e0;--dim:#6b7785;
  --accent:#3ddc97;--accent2:#4aa8ff;--warn:#ff6b6b;--amber:#ffb454}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--ink);font:13px/1.45 ui-monospace,Menlo,monospace}
.top{display:flex;align-items:center;gap:12px;padding:12px 18px;border-bottom:1px solid var(--line)}
.top b{color:var(--accent)} .top .r{margin-left:auto;color:var(--dim);font-size:11px}
.wrap{display:grid;grid-template-columns:1fr 1fr;gap:14px;padding:14px;max-width:1300px;margin:0 auto}
.card{background:var(--panel);border:1px solid var(--line);border-radius:10px;overflow:hidden;display:flex;flex-direction:column}
.h{font-size:10px;letter-spacing:1.5px;text-transform:uppercase;color:var(--dim);padding:9px 13px;
  border-bottom:1px solid var(--line);background:var(--panel2);display:flex;justify-content:space-between}
.b{padding:13px;display:flex;flex-direction:column;gap:10px;flex:1}
.row{display:flex;gap:8px}.row>*{flex:1}
button{font:inherit;font-size:14px;font-weight:700;padding:16px 8px;border-radius:8px;cursor:pointer;
  background:var(--panel2);border:1px solid var(--line);color:var(--ink)}
button small{display:block;font-size:10px;font-weight:400;color:var(--dim);margin-top:3px}
button:disabled{opacity:.35;cursor:not-allowed}
.go{border-color:#1f5a43;color:var(--accent)} .go:hover:not(:disabled){background:#12301f}
.st{border-color:#5a2b2b;color:var(--warn)} .st:hover:not(:disabled){background:#2e1515}
.ok{border-color:#1f4a73;color:var(--accent2)} .ok:hover:not(:disabled){background:#10243a}
.state{display:flex;align-items:center;gap:10px;padding:9px 11px;border-radius:8px;border:1px solid var(--line);background:var(--panel2)}
.state::before{content:"";width:9px;height:9px;border-radius:50%;background:var(--dim)}
.state.run{border-color:#6a4a1f;color:var(--amber)}.state.run::before{background:var(--amber);animation:bl 1s infinite}
.state.on{border-color:#1f5a43;color:var(--accent)}.state.on::before{background:var(--accent)}
.state.err{border-color:#5a2b2b;color:var(--warn)}.state.err::before{background:var(--warn)}
@keyframes bl{50%{opacity:.25}}
pre{margin:0;flex:1;min-height:280px;max-height:52vh;overflow:auto;background:#0a0d12;border:1px solid var(--line);
  border-radius:8px;padding:10px;font-size:11.5px;white-space:pre-wrap;word-break:break-all}
.note{font-size:11px;color:var(--dim)}
.links a{color:var(--accent2);margin-right:12px}
@media(max-width:900px){.wrap{grid-template-columns:1fr}}
</style></head><body>
<div class="top"><b>G1 Launcher</b><span class="r">:50080 · 웹 버튼은 비상정지가 아닙니다 — E-STOP/리모컨을 손에 두세요</span></div>
<div class="wrap">
  <div class="card">
    <div class="h"><span>자세 <code style="text-transform:none">start_fsm.sh</code></span><span id="fsm-t"></span></div>
    <div class="b">
      <div class="row">
        <button class="go" id="b-stand" onclick="fsm('stand')">Stand<small>1 → 4 → 501 (약 15초)</small></button>
        <button class="ok" id="b-sit" onclick="fsm('sit')">Sit<small>천천히 앉기</small></button>
        <button class="st" id="b-damp" onclick="fsm('damp')">Damp<small>힘 빼기</small></button>
      </div>
      <div class="state" id="fsm-s">대기</div>
      <pre id="fsm-log"></pre>
    </div>
  </div>
  <div class="card">
    <div class="h"><span>로봇 서버 <code style="text-transform:none">start_robot.sh</code></span><span id="rb-t"></span></div>
    <div class="b">
      <div class="row">
        <button class="go" id="b-rstart" onclick="robotStart()">Robot 시작<small>6개 서버 기동</small></button>
        <button class="st" id="b-rstop" onclick="robotStop()">Robot 정지<small>Ctrl+C 와 동일</small></button>
      </div>
      <div class="state" id="rb-s">정지됨</div>
      <pre id="rb-log"></pre>
      <div class="links note" id="links"></div>
    </div>
  </div>
</div>
<script>
const host=location.hostname;
document.getElementById('links').innerHTML=
  `<a href="http://${host}:50000/" target="_blank">Control :50000</a>`+
  `<a href="http://${host}:50003/dashboard" target="_blank">Dashboard :50003</a>`;
const CONFIRM={
  stand:'Stand 를 실행합니다.\n\n· 로봇을 사람이 붙잡고 있습니까?\n· 스탠드가 어깨에 단단히 묶여 있습니까?',
  sit:'Sit 를 실행합니다.\n\n· 팔을 몸 옆으로 내렸습니까?\n· 앉는 동안 로봇을 받치고 있습니까?\n· Robot 서버가 실행 중이면 먼저 정지하는 것을 권장합니다.',
  damp:'Damp 를 실행합니다. 모터 힘이 빠집니다.\n\n· 로봇을 받치고 있거나 앉아 있는 상태입니까?'};
async function post(u){const r=await fetch(u,{method:'POST'});const d=await r.json().catch(()=>({}));
  if(!r.ok)throw new Error(d.detail||r.status);return d;}
async function fsm(c){if(!confirm(CONFIRM[c]))return;
  try{await post('/fsm/'+c);}catch(e){alert(e.message);}poll();}
async function robotStart(){if(!confirm('start_robot.sh 를 실행합니다.\n\n· Stand(501) 가 완료된 상태입니까?\n· arm_server 가 기동하면서 팔/허리 제어권을 잡습니다.'))return;
  try{await post('/robot/start');}catch(e){alert(e.message);}poll();}
async function robotStop(){if(!confirm('로봇 서버를 정지합니다 (팔 제어권 반납 후 종료).'))return;
  try{await post('/robot/stop');}catch(e){alert(e.message);}poll();}
function fill(pre,lines){const atBottom=pre.scrollTop+pre.clientHeight>=pre.scrollHeight-20;
  pre.textContent=lines.join('\n');if(atBottom)pre.scrollTop=pre.scrollHeight;}
function setState(id,cls,txt){const el=document.getElementById(id);el.className='state'+(cls?' '+cls:'');el.textContent=txt;}
async function poll(){try{const d=await(await fetch('/status')).json();
  const f=d.fsm,r=d.robot;
  ['b-stand','b-sit','b-damp'].forEach(i=>document.getElementById(i).disabled=f.running);
  if(f.running)setState('fsm-s','run',`실행 중: ${f.label} (${f.elapsed}s)`);
  else if(f.label)setState('fsm-s',f.rc===0?'on':'err',`${f.label} 완료 (rc=${f.rc})`+(f.rc===0?'':' — 로그 확인'));
  else setState('fsm-s','','대기');
  fill(document.getElementById('fsm-log'),f.log);
  document.getElementById('b-rstart').disabled=r.running;
  document.getElementById('b-rstop').disabled=!r.running;
  if(r.running)setState('rb-s','on',`실행 중 (${Math.floor(r.elapsed/60)}분 ${Math.floor(r.elapsed%60)}초)`);
  else if(r.label)setState('rb-s',r.rc===0?'':'err',`정지됨 (rc=${r.rc})`);
  else setState('rb-s','','정지됨');
  fill(document.getElementById('rb-log'),r.log);
}catch(e){setState('fsm-s','err','launcher 연결 끊김');}}
poll();setInterval(poll,1000);
</script></body></html>"""


if __name__ == "__main__":
    if os.geteuid() != 0:
        print("[launcher] ⚠️ root 가 아님 — Stand/Sit/Damp 가 sudo 비밀번호에서 실패할 수 있음. ./launcher.sh 로 실행 권장")
    print(f"[launcher] http://0.0.0.0:{PORT}/  (start_robot.sh 실행 사용자: {SUDO_USER or os.environ.get('USER')})")
    uvicorn.run(app, host="0.0.0.0", port=PORT)
