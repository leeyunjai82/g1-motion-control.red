#!/usr/bin/env python3
"""
run_launcher.py — G1 자세(FSM) 제어 + start_robot.sh 실행 웹 (포트 50080)

  ./launcher.sh             (start_fsm.sh 와 같이 sudo 로 tv 환경 python 실행)
  → http://<robot-ip>:50080/

자세 — FSM 단계별 버튼 (LocoClient.SetFsmId 직접 호출, 연속 시퀀스 없음)
  사람이 로봇 상태를 보고 한 단계씩 누른다. init_fsm.py 의 stand = 1 → 4 → 501.
  현재 FSM(GetFsmId)에서 갈 수 있는 단계만 허용:
  · 1   Damping              : 항상 (서 있으면 넘어짐 — 경고창)
  · 4   Lock Standing        : FSM 1 에서 / 501 에서(= no-bal, Robot 정지 상태만)
  · 501 Walk (3DoF waist)    : FSM 4 에서 / 706 일어선 상태(추정)에서
  · 3   Sit Down             : FSM 4·501 에서, Robot 정지 상태만 (경고창)
  · 706 Balance Squat ↔ Stand: FSM 501 에서(쪼그리기) / 706 에서(토글), Robot 정지 상태만 (경고창)
        같은 ID 로 앉기·서기를 오간다(SDK StandUp2Squat / Squat2StandUp 모두 706).
        FSM 번호로는 쪼그림/섬을 구분할 수 없어 launcher 가 보낸 순서로 추정한다.
  · FSM 확인 불가 시 Damping 만 허용
  · Robot 시작은 FSM 501 에서만

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
# FSM (자세) — LocoClient 직접 호출, 단계별
# ==========================================
# Unitree G1 FSM ID (ai_sport)
FSM_NAME = {0: "Zero Torque", 1: "Damping", 2: "Squat (위치제어)", 3: "Sit Down (위치제어)",
            4: "Lock Standing", 500: "Walk", 501: "Walk (3DoF waist)",
            702: "Lie Down ↔ Stand Up", 706: "Balance Squat ↔ Stand", 801: "Run"}
FSM_BAL = {500: True, 501: True, 702: True, 706: True, 801: True}   # 밸런스 제어 여부 (나머지 없음)
STANDING = {4, 500, 501}
POLL_SEC = 1.0
STEPS = (1, 4, 501, 3, 706)
ROBOT_BUSY = "Robot 서버 실행 중 — 먼저 [Robot 정지]"


def allowed(target, cur, robot_running, squat=None):
    """(허용 여부, 거부 사유). UI 버튼 활성화와 서버 검사에 같은 규칙을 쓴다.
    squat: FSM 706 일 때 launcher 추정 자세 ('squat' | 'stand' | None=모름)."""
    if target == 1:
        return True, ""
    if cur is None:
        return False, "현재 FSM 확인 불가 — Damping 만 가능 (터미널 ./start_fsm.sh 사용)"
    if target == 4:
        if cur == 1:
            return True, ""
        if cur == 501:
            return (False, ROBOT_BUSY) if robot_running else (True, "")
        return False, f"4 는 FSM 1(Damping) 또는 501 에서만 (현재 {cur})"
    if target == 501:
        if cur == 4:
            return True, ""
        if cur == 706:
            return (True, "") if squat == "stand" else \
                   (False, "706 쪼그린 상태(또는 모름) — 706 으로 먼저 일어선 뒤 501")
        return False, f"501 은 FSM 4 또는 706(일어선 상태)에서만 (현재 {cur})"
    if target == 3:
        if robot_running:
            return False, ROBOT_BUSY
        return (True, "") if cur in STANDING else (False, f"Sit 은 서 있을 때만 (현재 {cur})")
    if target == 706:
        if robot_running:
            return False, ROBOT_BUSY
        return (True, "") if cur in (501, 706) else (False, f"706 은 FSM 501 또는 706 에서만 (현재 {cur})")
    return False, "알 수 없는 단계"


class FsmCtl:
    def __init__(self):
        self.client = None
        self.init_err = None
        self._dds_inited = False
        self.call_lock = threading.Lock()      # RPC 직렬화 (폴링 / 명령)
        self.cur = None                        # 마지막으로 읽은 FSM id (None=미확인)
        self.cur_err = None
        self.get_supported = True
        self.busy = None                       # 전송 중인 FSM id
        self.squat = None                      # 706 자세 추정 ('squat' | 'stand' | None)
        self.result = None                     # (ok, msg)
        self.lines = deque(maxlen=300)
        self.logf = _logfile("fsm")
        self.lock = threading.Lock()

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
        if self.cur != 706:
            self.squat = None                  # 706 을 벗어나면 추정 초기화
        return self.cur

    def set(self, target, robot_running):
        if target not in STEPS:
            raise HTTPException(400, f"FSM 은 {STEPS} 중 하나")
        with self.lock:
            if self.busy is not None:
                raise HTTPException(409, f"FSM {self.busy} 전송 중")
            self.busy = target
        try:
            if not self._ensure_client():
                raise HTTPException(503, f"LocoClient 초기화 실패: {self.init_err}")
            cur = self.read_fsm()
            ok, why = allowed(target, cur, robot_running, self.squat)
            if not ok:
                self.log(f"FSM {target} 거부 — {why}")
                raise HTTPException(409, why)
            self.log(f"SetFsmId({target}) … (현재 {cur})")
            with self.call_lock:
                code = self.client.SetFsmId(target)
            now = self.read_fsm()
            self.log(f"SetFsmId({target}) → code={code}, 현재 FSM = {now} ({FSM_NAME.get(now, '?')})")
            if code != 0:
                self.result = (False, f"FSM {target} 실패 (code={code})")
                raise HTTPException(502, self.result[1])
            if target == 706:
                # 501 → 706 은 쪼그리기, 706 → 706 은 토글
                self.squat = "squat" if cur != 706 else ("stand" if self.squat == "squat" else "squat")
                self.log(f"706 자세 추정: {'쪼그림' if self.squat == 'squat' else '일어섬'}")
            self.result = (True, f"FSM {target} {FSM_NAME.get(target, '')} 전송 완료")
        finally:
            with self.lock:
                self.busy = None

    def poll_loop(self):
        while True:
            if self.busy is None:
                self.read_fsm()
            time.sleep(POLL_SEC)

    def state(self, robot_running):
        return {"cur": self.cur, "cur_name": FSM_NAME.get(self.cur, "") if self.cur is not None else "",
                "cur_bal": FSM_BAL.get(self.cur, False), "squat": self.squat,
                "cur_err": self.cur_err, "busy": self.busy, "result": self.result,
                "allowed": {str(t): allowed(t, self.cur, robot_running, self.squat)[0] for t in STEPS},
                "log": list(self.lines)[-120:]}


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


@app.post("/fsm/{target}")
def run_fsm(target: int):
    fsm.set(target, robot.running())
    return {"ok": True}


@app.post("/robot/start")
def robot_start():
    if fsm.busy is not None:
        raise HTTPException(409, f"FSM {fsm.busy} 전송 중")
    cur = fsm.read_fsm()
    if cur is not None and cur != 501:
        raise HTTPException(409, f"FSM {cur} ({FSM_NAME.get(cur, '')}) — 1 → 4 → 501 로 Walk(3DoF waist) 진입 후 시작")
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
    return {"fsm": fsm.state(robot.running()), "robot": robot.state()}


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
html,body{height:100%}
body{margin:0;background:var(--bg);color:var(--ink);font:13px/1.45 ui-monospace,Menlo,monospace;
  display:flex;flex-direction:column;min-height:640px}
.top{display:flex;align-items:center;gap:12px;padding:12px 18px;border-bottom:1px solid var(--line)}
.top b{color:var(--accent)} .top .r{margin-left:auto;color:var(--dim);font-size:11px}
.wrap{flex:1;min-height:0;display:grid;grid-template-columns:1fr 1fr;gap:14px;padding:14px}
.card{background:var(--panel);border:1px solid var(--line);border-radius:10px;overflow:hidden;display:flex;flex-direction:column;min-height:0}
.h{font-size:10px;letter-spacing:1.5px;text-transform:uppercase;color:var(--dim);padding:9px 13px;
  border-bottom:1px solid var(--line);background:var(--panel2);display:flex;justify-content:space-between}
.h code{text-transform:none}
.b{padding:13px;display:flex;flex-direction:column;gap:10px;flex:1;min-height:0}
.row{display:flex;gap:8px}.row>*{flex:1}
button{font:inherit;font-size:14px;font-weight:700;padding:16px 8px;border-radius:8px;cursor:pointer;
  background:var(--panel2);border:1px solid var(--line);color:var(--ink)}
button small{display:block;font-size:10px;font-weight:400;color:var(--dim);margin-top:3px}
button:disabled{opacity:.3;cursor:not-allowed}
button .no{display:block;font-size:20px;margin-bottom:2px}
button.next:not(:disabled){box-shadow:0 0 0 2px var(--accent) inset;animation:nx 1.6s infinite}
@keyframes nx{50%{box-shadow:0 0 0 2px transparent inset}}
.go{border-color:#1f5a43;color:var(--accent)} .go:hover:not(:disabled){background:#12301f}
.st{border-color:#5a2b2b;color:var(--warn)} .st:hover:not(:disabled){background:#2e1515}
.ok{border-color:#1f4a73;color:var(--accent2)} .ok:hover:not(:disabled){background:#10243a}
.fsmnow{display:flex;align-items:baseline;gap:12px;padding:12px 14px;border-radius:8px;border:1px solid var(--line);background:#0a0d12}
.fsmnow .k{font-size:11px;color:var(--dim)}
.fsmnow .v{font-size:22px;font-weight:700}
.fsmnow .n{font-size:13px;color:var(--dim)}
.fsmnow .bal{margin-left:auto;font-size:11px;padding:2px 8px;border-radius:999px;border:1px solid var(--line)}
.fsmnow .bal.on{color:var(--accent);border-color:#1f5a43}.fsmnow .bal.off{color:var(--amber);border-color:#6a4a1f}
.fsmnow.bal .v{color:var(--accent)} .fsmnow.std .v{color:var(--accent2)} .fsmnow.low .v{color:var(--amber)}
.fsmnow.unk .v{color:var(--warn);font-size:14px}
.state{display:flex;align-items:center;gap:10px;padding:9px 11px;border-radius:8px;border:1px solid var(--line);background:var(--panel2)}
.state::before{content:"";width:9px;height:9px;border-radius:50%;background:var(--dim);flex:none}
.state.run{border-color:#6a4a1f;color:var(--amber)}.state.run::before{background:var(--amber);animation:bl 1s infinite}
.state.on{border-color:#1f5a43;color:var(--accent)}.state.on::before{background:var(--accent)}
.state.err{border-color:#5a2b2b;color:var(--warn)}.state.err::before{background:var(--warn)}
@keyframes bl{50%{opacity:.25}}
pre{margin:0;flex:1;min-height:120px;overflow:auto;background:#0a0d12;border:1px solid var(--line);
  border-radius:8px;padding:10px;font-size:11.5px;white-space:pre-wrap;word-break:break-all}
.note{font-size:11px;color:var(--dim)}
.links a{color:var(--accent2);margin-right:12px}
/* 큰 화면: 버튼 키움 */
@media(min-height:900px){button{padding:22px 8px;font-size:15px}button .no{font-size:24px}}
@media(max-width:900px){body{height:auto}.wrap{grid-template-columns:1fr}pre{min-height:240px}}
.modal{position:fixed;inset:0;background:#000b;display:none;align-items:center;justify-content:center;z-index:9}
.modal.show{display:flex}
.mbox{width:min(460px,92vw);background:var(--panel);border:2px solid var(--warn);border-radius:12px;overflow:hidden}
.mbox .mt{background:#2e1515;color:var(--warn);font-size:16px;font-weight:700;padding:12px 16px}
.mbox .mm{padding:16px;font-size:13.5px;line-height:1.7;white-space:pre-line}
.mbox .mm .big{display:block;color:var(--warn);font-weight:700;font-size:15px;margin-bottom:8px}
.mbox .mb{display:flex;gap:8px;padding:0 16px 16px}.mbox .mb button{flex:1;padding:12px}
.mbox .run{background:#5a1f1f;border-color:var(--warn);color:#fff}
</style></head><body>
<div class="top"><b>G1 Launcher</b><span class="r">:50080 · 웹 버튼은 비상정지가 아닙니다 — E-STOP/리모컨을 손에 두세요</span></div>
<div class="wrap">
  <div class="card">
    <div class="h"><span>자세 (FSM)</span><span></span></div>
    <div class="b">
      <div class="fsmnow unk" id="fsmnow"><span class="k">현재 FSM</span><span class="v" id="fsm-v">확인 중</span><span class="n" id="fsm-n"></span><span class="bal" id="fsm-b"></span></div>
      <div class="note">일어서기: <b>1 → 4 → 501</b> 순서로, 로봇이 자리 잡은 걸 보고 다음 단계를 누르세요</div>
      <div class="row">
        <button class="st" id="b-1" onclick="fsm(1)"><span class="no">1</span>Damping<small>힘 빼기 · 항상 가능</small></button>
        <button class="go" id="b-4" onclick="fsm(4)"><span class="no">4</span>Lock Standing<small>밸런스 없음 · 1 / 501 에서</small></button>
        <button class="go" id="b-501" onclick="fsm(501)"><span class="no">501</span>Walk 3DoF waist<small>밸런스 · arm_sdk · 4 에서</small></button>
      </div>
      <div class="row">
        <button class="ok" id="b-706" onclick="fsm(706)"><span class="no">706</span><span id="b-706-l">Balance Squat</span><small id="b-706-s">밸런스 쪼그리기 ↔ 일어서기 · 501 에서</small></button>
        <button class="ok" id="b-3" onclick="fsm(3)"><span class="no">3</span>Sit Down<small>밸런스 없음 · 서 있을 때</small></button>
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
<div class="modal" id="modal"><div class="mbox">
  <div class="mt" id="m-t">⚠️ 경고</div><div class="mm" id="m-m"></div>
  <div class="mb"><button id="m-no">취소</button><button class="run" id="m-yes">실행</button></div>
</div></div>
<script>
const host=location.hostname;
// 경고 모달 — Promise<bool>
function warn(title,big,body){return new Promise(res=>{
  const m=document.getElementById('modal');
  document.getElementById('m-t').textContent='⚠️ '+title;
  document.getElementById('m-m').innerHTML=(big?`<span class="big">${big}</span>`:'')+body;
  const done=v=>{m.classList.remove('show');document.removeEventListener('keydown',esc);res(v);};
  const esc=e=>{if(e.key==='Escape')done(false);};
  document.getElementById('m-no').onclick=()=>done(false);
  document.getElementById('m-yes').onclick=()=>done(true);
  document.addEventListener('keydown',esc);
  m.classList.add('show');document.getElementById('m-no').focus();});}
document.getElementById('links').innerHTML=
  `<a href="http://${host}:50000/" target="_blank">Control :50000</a>`+
  `<a href="http://${host}:50003/dashboard" target="_blank">Dashboard :50003</a>`;
let CUR=null,SQUAT=null;
// Damp / Sit 만 경고 후 실행, 4 / 501 은 바로 실행
async function confirmFsm(t){
  const standing=[4,500,501,706].includes(CUR);
  if(t===1)return warn('Damp (FSM 1)',
    standing?'지금 서 있습니다 — 힘이 빠져 넘어집니다!':'모터 힘이 빠집니다',
    '· 로봇을 사람이 받치고 있거나 스탠드에 묶여 있습니까?\n· 주변에 사람/장애물이 없습니까?');
  if(t===3)return warn('Sit Down (FSM 3)','로봇이 천천히 앉습니다 (밸런스 제어 없음)',
    '· 팔을 몸 옆으로 내렸습니까?\n· 앉는 동안 로봇을 받치고 있습니까?\n· 엉덩이 아래 공간이 비어 있습니까?');
  if(t===706)return warn('Balance Squat ↔ Stand (FSM 706)',
    SQUAT==='squat'?'쪼그린 상태에서 일어섭니다':'균형을 잡으며 쪼그려 앉습니다',
    '· 처음 쓰는 모드입니다 — 로봇을 받칠 준비가 되어 있습니까?\n· 주변·아래 공간이 비어 있습니까?\n· 같은 버튼(706)으로 앉기·서기를 오갑니다');
  return true;}
async function post(u){const r=await fetch(u,{method:'POST'});const d=await r.json().catch(()=>({}));
  if(!r.ok)throw new Error(d.detail||r.status);return d;}
async function fsm(t){if(!await confirmFsm(t))return;
  try{await post('/fsm/'+t);}catch(e){alert(e.message);}poll();}
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
  else{nw.className='fsmnow '+([500,501].includes(f.cur)?'bal':[4,706].includes(f.cur)?'std':'low');
    document.getElementById('fsm-v').textContent=f.cur;
    document.getElementById('fsm-n').textContent=f.cur_name+
      (f.cur===706?(f.squat==='squat'?' · 쪼그림(추정)':f.squat==='stand'?' · 일어섬(추정)':' · 자세 모름'):'');}
  const fb=document.getElementById('fsm-b');
  if(f.cur===null){fb.textContent='';fb.className='bal';}
  else{fb.textContent=f.cur_bal?'밸런스 제어':'밸런스 없음';fb.className='bal '+(f.cur_bal?'on':'off');}
  SQUAT=f.squat;
  document.getElementById('b-706-l').textContent=f.cur===706?(f.squat==='squat'?'Squat → Stand':'Stand → Squat'):'Balance Squat';
  // 버튼: 현재 FSM 에서 갈 수 있는 단계만 활성, 다음 단계 강조 (1→4→501)
  CUR=f.cur;
  const next=f.cur===706?(f.squat==='stand'?501:706):{1:4,4:501}[f.cur];
  [1,4,501,3,706].forEach(t=>{const b=document.getElementById('b-'+t);
    b.disabled=f.busy!==null||!f.allowed[t];b.classList.toggle('next',t===next);});
  if(f.busy!==null)setState('fsm-s','run',`FSM ${f.busy} 전송 중…`);
  else if(f.result)setState('fsm-s',f.result[0]?'on':'err',f.result[1]);
  else setState('fsm-s','','대기');
  fill(document.getElementById('fsm-log'),f.log);
  // 로봇 서버
  document.getElementById('b-rstart').disabled=r.running||f.busy!==null||(f.cur!==null&&f.cur!==501);
  document.getElementById('b-rstart').classList.toggle('next',f.cur===501&&!r.running);
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
