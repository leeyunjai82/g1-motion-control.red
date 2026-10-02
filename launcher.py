#!/usr/bin/env python3
"""
launcher.py — G1 자세(FSM) 제어 + start_robot.sh 실행 웹 (포트 50080)

  ./launcher.sh             (start_fsm.sh 와 같이 sudo 로 tv 환경 python 실행)
  → http://<robot-ip>:50080/

자세 (LocoClient.SetFsmId 직접 호출 — utils/init_fsm.py 와 같은 순서·대기시간)
  · Stand : 1 → 5s → 4 → 10s → 501
  · Sit   : 3s 대기 → 3
  · Damp  : 3s 대기 → 1
  추가 안전장치 (init_fsm.py 에는 없음)
  · 현재 FSM(GetFsmId) 확인 — 이미 서 있으면(4/500/501 등) Stand 거부.
    (Stand 의 첫 단계 FSM 1 = Damp. 서 있는 상태에서 다시 Stand 하면 5초간 힘이 빠져 넘어진다)
  · 단계마다 SetFsmId 반환 코드 확인 — 실패 시 다음 단계를 보내지 않고 중단
  · Robot 서버 실행 중이면 Sit 거부 / FSM 501 이 아니면 Robot 시작 거부
  · Damp 는 진행 중인 Stand/Sit 시퀀스를 끊고 실행 (항상 가능)

  ./start_fsm.sh / utils/init_fsm.py 는 수정하지 않았다 — 터미널에서 따로 쓸 수 있다.
  단, 둘을 동시에 쓰면 서로의 진행을 모른다. 한쪽만 쓸 것.

로봇 서버
  · Robot 시작 / 정지 : ./start_robot.sh 실행 / SIGTERM(= Ctrl+C 와 같은 종료 시퀀스)

권한
  · launcher 는 root 로 돈다 (launcher.sh 의 sudo). FSM RPC 도 root 로 호출 (start_fsm.sh 와 동일).
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
ROBOT_STOP_WAIT = 20.0      # start_robot.sh 종료 시퀀스(최대 8초 + sweep) 여유

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


def _logfile(name):
    path = os.path.join(LOG_DIR, f"launcher_{name}_{time.strftime('%Y%m%d')}.log")
    f = open(path, "a", encoding="utf-8")
    _own(path)
    return f


# ==========================================
# FSM (자세) — LocoClient 직접 호출
# ==========================================
FSM_NAME = {0: "Zero Torque", 1: "Damp", 3: "Sit", 4: "Stand",
            500: "Start", 501: "밸런싱 (arm_sdk)"}
STAND_FROM = {0, 1, 3}            # Stand 를 시작해도 되는 상태 (힘 빠짐 / 앉음)
STANDING = {4, 500, 501}          # 서 있는 상태 — Sit 허용
POLL_SEC = 1.0


class FsmAbort(Exception):
    pass


class FsmCtl:
    def __init__(self):
        self.client = None
        self.init_err = None
        self._dds_inited = False
        self.call_lock = threading.Lock()      # RPC 직렬화 (폴링 / 시퀀스)
        self.cur = None                        # 마지막으로 읽은 FSM id (None=미확인)
        self.cur_err = None
        self.get_supported = True
        self.task = None                       # 실행 중 시퀀스 이름
        self.task_started = 0.0
        self.step = ""
        self.result = None                     # (ok, msg)
        self.cancel = threading.Event()
        self.lines = deque(maxlen=300)
        self.logf = _logfile("fsm")
        self.lock = threading.Lock()

    # ---- 로그 ----
    def log(self, msg):
        line = f"[{time.strftime('%H:%M:%S')}] {msg}"
        self.lines.append(line)
        print(f"[fsm] {msg}")
        self.logf.write(f"{time.strftime('%Y-%m-%d')} {line}\n")
        self.logf.flush()

    # ---- DDS / LocoClient (지연 초기화 — 로봇 미연결이어도 웹은 뜬다) ----
    def _ensure_client(self):
        if self.client is not None:
            return True
        try:
            from unitree_sdk2py.core.channel import ChannelFactoryInitialize
            from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
            if not self._dds_inited:          # 프로세스당 1회만
                ChannelFactoryInitialize(0)
                self._dds_inited = True
            c = LocoClient()
            c.Init()
            c.SetTimeout(10.0)            # init_fsm.py 와 동일
            self.client = c
            self.init_err = None
            self.get_supported = hasattr(c, "GetFsmId")
            return True
        except Exception as e:
            self.init_err = str(e)
            return False

    def read_fsm(self):
        """현재 FSM id 읽기 → id 또는 None. 결과는 self.cur 에 저장."""
        if not self._ensure_client():
            self.cur, self.cur_err = None, f"LocoClient 초기화 실패: {self.init_err}"
            return None
        if not self.get_supported:
            self.cur, self.cur_err = None, "이 SDK 에 GetFsmId 없음"
            return None
        with self.call_lock:
            try:
                code, data = self.client.GetFsmId()
            except Exception as e:
                code, data = -1, str(e)
        if code != 0 or data is None:
            self.cur, self.cur_err = None, f"GetFsmId 실패 (code={code})"
            return None
        self.cur, self.cur_err = int(data), None
        return self.cur

    def _set(self, fsm_id):
        if self.cancel.is_set():
            raise FsmAbort("중단됨")
        self.log(f"SetFsmId({fsm_id}) …")
        with self.call_lock:
            code = self.client.SetFsmId(fsm_id)
        self.log(f"SetFsmId({fsm_id}) → code={code}")
        if code != 0:
            raise FsmAbort(f"SetFsmId({fsm_id}) 실패 (code={code}) — 다음 단계 보내지 않음")
        now = self.read_fsm()
        if now is not None:
            self.log(f"현재 FSM = {now} ({FSM_NAME.get(now, '?')})")

    def _wait(self, sec, why):
        self.step = why
        end = time.time() + sec
        while time.time() < end:
            if self.cancel.wait(0.1):
                raise FsmAbort("중단됨")

    # ---- 시퀀스 (init_fsm.py 와 동일 순서·대기) ----
    def _seq_stand(self):
        self.step = "1 Damp"
        self._set(1)
        self._wait(5, "1 → 4 대기 (5초)")
        self.step = "4 Stand"
        self._set(4)
        self._wait(10, "4 → 501 대기 (10초)")
        self.step = "501 밸런싱"
        self._set(501)

    def _seq_sit(self):
        self._wait(3, "Sit 대기 (3초)")
        self.step = "3 Sit"
        self._set(3)

    def _seq_damp(self):
        self._wait(3, "Damp 대기 (3초)")
        self.step = "1 Damp"
        self._set(1)

    def _run(self, name, fn):
        self.log(f"===== {name} 시작 =====")
        try:
            fn()
            self.result = (True, f"{name} 완료")
            self.log(f"===== {name} 완료 =====")
        except FsmAbort as e:
            self.result = (False, f"{name} 중단: {e}")
            self.log(f"===== {name} 중단: {e} =====")
        except Exception as e:
            self.result = (False, f"{name} 오류: {e}")
            self.log(f"===== {name} 오류: {e} =====")
        finally:
            with self.lock:
                self.task, self.step = None, ""

    # ---- 진입점 (검사 후 스레드 실행) ----
    def start(self, cmd, robot_running):
        with self.lock:
            if cmd == "damp":
                # Damp 는 항상 허용 — 진행 중 시퀀스가 있으면 끊고 실행
                if self.task:
                    self.log(f"Damp 요청 — 진행 중인 {self.task} 중단")
                    self.cancel.set()
            elif self.task:
                raise HTTPException(409, f"{self.task} 진행 중")

        if cmd == "damp" and self.task:
            t0 = time.time()
            while self.task and time.time() - t0 < 12:   # RPC timeout(10s) 여유
                time.sleep(0.05)
            if self.task:
                raise HTTPException(409, f"{self.task} 가 끝나지 않음 — 리모컨/E-STOP 사용")

        if not self._ensure_client():
            raise HTTPException(503, f"LocoClient 초기화 실패: {self.init_err}")
        cur = self.read_fsm()

        if cmd == "stand":
            if cur is None:
                raise HTTPException(409, f"현재 FSM 확인 불가({self.cur_err}) — "
                                         "중복 Stand 위험이 있어 웹에서는 실행하지 않음. "
                                         "터미널 ./start_fsm.sh stand 사용")
            if cur not in STAND_FROM:
                raise HTTPException(409, f"이미 서 있음 (FSM {cur} {FSM_NAME.get(cur, '')}) — "
                                         "Stand 는 첫 단계가 Damp(1)라 서 있는 상태에서 실행하면 넘어진다")
        elif cmd == "sit":
            if robot_running:
                raise HTTPException(409, "Robot 서버 실행 중 — 먼저 [Robot 정지] 후 Sit")
            if cur is not None and cur not in STANDING:
                raise HTTPException(409, f"서 있는 상태가 아님 (FSM {cur} {FSM_NAME.get(cur, '')})")

        fn = {"stand": self._seq_stand, "sit": self._seq_sit, "damp": self._seq_damp}[cmd]
        with self.lock:
            self.cancel.clear()
            self.task, self.task_started, self.result = cmd, time.time(), None
        threading.Thread(target=self._run, args=(cmd, fn), daemon=True).start()

    def poll_loop(self):
        while True:
            if not self.task:                 # 시퀀스 중에는 _set 이 갱신
                self.read_fsm()
            time.sleep(POLL_SEC)

    def state(self):
        return {"cur": self.cur, "cur_name": FSM_NAME.get(self.cur, "") if self.cur is not None else "",
                "cur_err": self.cur_err, "task": self.task, "step": self.step,
                "elapsed": round(time.time() - self.task_started, 1) if self.task else 0,
                "result": self.result, "log": list(self.lines)[-120:]}


# ==========================================
# start_robot.sh (자식 프로세스)
# ==========================================
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
            logf = _logfile(self.name)
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
        logf.close()

    def state(self):
        # rc 는 프로세스에서 직접 — 고아 자식이 stdout 파이프를 잡고 있으면 _pump 가 끝나지 않을 수 있다
        rc = self.proc.poll() if self.proc is not None else None
        if rc is not None and not self.ended:
            self.ended = time.time()
        return {"running": self.running(), "label": self.label, "rc": rc,
                "elapsed": round((time.time() if self.running() else (self.ended or time.time()))
                                 - self.started, 1) if self.started else 0,
                "log": list(self.lines)[-120:]}


fsm = FsmCtl()
robot = Job("robot")


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


@asynccontextmanager
async def lifespan(app: FastAPI):
    threading.Thread(target=fsm.poll_loop, daemon=True).start()
    yield
    # launcher 종료(Ctrl+C) 시 로봇 서버도 정상 종료 시퀀스로 정지.
    # 남겨두면 다음 start_robot.sh 의 sweep 이 SIGKILL 로 죽여 arm 제어권 반납이 생략된다.
    if robot.running():
        print("[launcher] 종료 — start_robot.sh 정지 중...")
        _stop_robot()


app = FastAPI(title="G1 Launcher", lifespan=lifespan)


@app.post("/fsm/{cmd}")
def run_fsm(cmd: str):
    if cmd not in ("stand", "sit", "damp"):
        raise HTTPException(400, "cmd 는 stand / sit / damp")
    fsm.start(cmd, robot.running())
    return {"ok": True}


@app.post("/robot/start")
def robot_start():
    if fsm.task:
        raise HTTPException(409, f"자세 전환({fsm.task}) 진행 중")
    cur = fsm.read_fsm()
    if cur is not None and cur != 501:
        raise HTTPException(409, f"FSM {cur} ({FSM_NAME.get(cur, '')}) — Stand 로 501(밸런싱) 진입 후 시작")
    robot.start(["bash", os.path.join(ROOT, "start_robot.sh")], "start_robot.sh", as_user=True)
    if cur is None:
        robot.lines.append(f"[launcher] ⚠️ FSM 확인 불가({fsm.cur_err}) — 501 상태인지 직접 확인할 것")
    return {"ok": True}


@app.post("/robot/stop")
def robot_stop():
    if not robot.running():
        return {"ok": True, "already": True}
    threading.Thread(target=_stop_robot, daemon=True).start()
    return {"ok": True}


@app.get("/status")
def status():
    return {"fsm": fsm.state(), "robot": robot.state()}


@app.get("/", response_class=HTMLResponse)
def index():
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
.h code{text-transform:none}
.b{padding:13px;display:flex;flex-direction:column;gap:10px;flex:1}
.row{display:flex;gap:8px}.row>*{flex:1}
button{font:inherit;font-size:14px;font-weight:700;padding:16px 8px;border-radius:8px;cursor:pointer;
  background:var(--panel2);border:1px solid var(--line);color:var(--ink)}
button small{display:block;font-size:10px;font-weight:400;color:var(--dim);margin-top:3px}
button:disabled{opacity:.35;cursor:not-allowed}
.go{border-color:#1f5a43;color:var(--accent)} .go:hover:not(:disabled){background:#12301f}
.st{border-color:#5a2b2b;color:var(--warn)} .st:hover:not(:disabled){background:#2e1515}
.ok{border-color:#1f4a73;color:var(--accent2)} .ok:hover:not(:disabled){background:#10243a}
.fsmnow{display:flex;align-items:baseline;gap:12px;padding:12px 14px;border-radius:8px;border:1px solid var(--line);background:#0a0d12}
.fsmnow .k{font-size:11px;color:var(--dim)}
.fsmnow .v{font-size:22px;font-weight:700}
.fsmnow .n{font-size:13px;color:var(--dim)}
.fsmnow.bal .v{color:var(--accent)} .fsmnow.std .v{color:var(--accent2)} .fsmnow.low .v{color:var(--amber)}
.fsmnow.unk .v{color:var(--warn);font-size:14px}
.state{display:flex;align-items:center;gap:10px;padding:9px 11px;border-radius:8px;border:1px solid var(--line);background:var(--panel2)}
.state::before{content:"";width:9px;height:9px;border-radius:50%;background:var(--dim);flex:none}
.state.run{border-color:#6a4a1f;color:var(--amber)}.state.run::before{background:var(--amber);animation:bl 1s infinite}
.state.on{border-color:#1f5a43;color:var(--accent)}.state.on::before{background:var(--accent)}
.state.err{border-color:#5a2b2b;color:var(--warn)}.state.err::before{background:var(--warn)}
@keyframes bl{50%{opacity:.25}}
pre{margin:0;flex:1;min-height:240px;max-height:46vh;overflow:auto;background:#0a0d12;border:1px solid var(--line);
  border-radius:8px;padding:10px;font-size:11.5px;white-space:pre-wrap;word-break:break-all}
.note{font-size:11px;color:var(--dim)}
.links a{color:var(--accent2);margin-right:12px}
@media(max-width:900px){.wrap{grid-template-columns:1fr}}
</style></head><body>
<div class="top"><b>G1 Launcher</b><span class="r">:50080 · 웹 버튼은 비상정지가 아닙니다 — E-STOP/리모컨을 손에 두세요</span></div>
<div class="wrap">
  <div class="card">
    <div class="h"><span>자세 (FSM)</span><span></span></div>
    <div class="b">
      <div class="fsmnow unk" id="fsmnow"><span class="k">현재 FSM</span><span class="v" id="fsm-v">확인 중</span><span class="n" id="fsm-n"></span></div>
      <div class="row">
        <button class="go" id="b-stand" onclick="fsm('stand')">Stand<small>1 → 4 → 501 (약 15초)</small></button>
        <button class="ok" id="b-sit" onclick="fsm('sit')">Sit<small>3초 후 천천히 앉기</small></button>
        <button class="st" id="b-damp" onclick="fsm('damp')">Damp<small>3초 후 힘 빼기 · 진행 중 시퀀스 중단</small></button>
      </div>
      <div class="state" id="fsm-s">대기</div>
      <pre id="fsm-log"></pre>
    </div>
  </div>
  <div class="card">
    <div class="h"><span>로봇 서버 <code>start_robot.sh</code></span><span></span></div>
    <div class="b">
      <div class="row">
        <button class="go" id="b-rstart" onclick="robotStart()">Robot 시작<small>FSM 501 에서만 · 6개 서버</small></button>
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
  stand:'Stand 를 실행합니다 (1 Damp → 4 → 501).\n\n· 로봇을 사람이 붙잡고 있습니까?\n· 스탠드가 어깨에 단단히 묶여 있습니까?',
  sit:'Sit 를 실행합니다 (3초 후).\n\n· 팔을 몸 옆으로 내렸습니까?\n· 앉는 동안 로봇을 받치고 있습니까?',
  damp:'Damp 를 실행합니다 (3초 후). 모터 힘이 빠집니다.\n진행 중인 Stand/Sit 가 있으면 중단합니다.\n\n· 로봇을 받치고 있거나 앉아 있는 상태입니까?'};
async function post(u){const r=await fetch(u,{method:'POST'});const d=await r.json().catch(()=>({}));
  if(!r.ok)throw new Error(d.detail||r.status);return d;}
async function fsm(c){if(!confirm(CONFIRM[c]))return;
  try{await post('/fsm/'+c);}catch(e){alert(e.message);}poll();}
async function robotStart(){if(!confirm('start_robot.sh 를 실행합니다.\n\n· arm_server 가 기동하면서 팔/허리 제어권(weight=1)을 잡습니다.'))return;
  try{await post('/robot/start');}catch(e){alert(e.message);}poll();}
async function robotStop(){if(!confirm('로봇 서버를 정지합니다 (팔 제어권 반납 후 종료).'))return;
  try{await post('/robot/stop');}catch(e){alert(e.message);}poll();}
function fill(pre,lines){const atBottom=pre.scrollTop+pre.clientHeight>=pre.scrollHeight-20;
  pre.textContent=lines.join('\n');if(atBottom)pre.scrollTop=pre.scrollHeight;}
function setState(id,cls,txt){const el=document.getElementById(id);el.className='state'+(cls?' '+cls:'');el.textContent=txt;}
async function poll(){try{const d=await(await fetch('/status')).json();
  const f=d.fsm,r=d.robot;
  // 현재 FSM
  const nw=document.getElementById('fsmnow');
  if(f.cur===null){nw.className='fsmnow unk';document.getElementById('fsm-v').textContent='확인 불가';
    document.getElementById('fsm-n').textContent=f.cur_err||'';}
  else{nw.className='fsmnow '+(f.cur===501?'bal':[4,500].includes(f.cur)?'std':'low');
    document.getElementById('fsm-v').textContent=f.cur;document.getElementById('fsm-n').textContent=f.cur_name;}
  // 버튼: 진행 중엔 Stand/Sit 잠금 (Damp 는 항상 가능)
  const standing=[4,500,501].includes(f.cur);
  document.getElementById('b-stand').disabled=!!f.task||f.cur===null||standing;
  document.getElementById('b-sit').disabled=!!f.task||r.running||(f.cur!==null&&!standing);
  document.getElementById('b-damp').disabled=f.task==='damp';
  if(f.task)setState('fsm-s','run',`${f.task} 진행 중 — ${f.step} (${f.elapsed}s)`);
  else if(f.result)setState('fsm-s',f.result[0]?'on':'err',f.result[1]);
  else setState('fsm-s','','대기');
  fill(document.getElementById('fsm-log'),f.log);
  // 로봇 서버
  document.getElementById('b-rstart').disabled=r.running||!!f.task||(f.cur!==null&&f.cur!==501);
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
        print("[launcher] ⚠️ root 아님 — ./launcher.sh (sudo) 로 실행 권장")
    print(f"[launcher] http://0.0.0.0:{PORT}/  (start_robot.sh 실행 사용자: {SUDO_USER or os.environ.get('USER')})")
    uvicorn.run(app, host="0.0.0.0", port=PORT)
