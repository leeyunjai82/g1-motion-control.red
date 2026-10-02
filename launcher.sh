#!/bin/bash
# G1 Launcher 웹 (http://<robot-ip>/ , 포트 80) — FSM 버튼 / start_robot.sh 실행
# start_fsm.sh 와 같이 sudo 로 tv 환경 python 을 실행한다 (비밀번호는 여기서 한 번만).
#
# 기동 전 정리 (이전 실행이 남아 있으면):
#   1) 이전 run_launcher.py   — SIGTERM (launcher 가 start_robot.sh 를 정상 종료시킴) → 25초 후 KILL
#   2) 남은 start_robot.sh    — SIGTERM (스크립트의 Ctrl+C 종료 시퀀스)              → 15초 후 KILL
#   3) 남은 로봇 서버 6개      — SIGTERM (arm_server 는 weight 반납 후 종료)          → 5초 후 KILL
#   start_robot.sh 의 sweep 은 바로 SIGKILL 이지만, 여기서는 TERM 을 먼저 보내
#   arm_server 가 팔 제어권을 천천히 반납할 시간을 준다.
#
# 종료: Ctrl+C  (실행 중인 Robot 서버도 정상 종료 시퀀스로 같이 정지)

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PY=/home/circulus/miniconda3/envs/tv/bin/python
TARGETS=("rs_stream" "arm_server" "robot_server" "dashboard" "detect_marker" "detect_box")

# 실행 형태에만 맞는 패턴 (vim/tail 등 파일명이 들어간 다른 명령은 건드리지 않게)
#   launcher : "<...>python <...>run_launcher.py"  ← TERM 은 python 에만.
#              sudo 부모까지 TERM 을 받으면 uvicorn 이 신호를 두 번 받아 강제 종료(force_exit)되어
#              lifespan 정리(start_robot.sh 정상 종료)를 건너뛸 수 있다. sudo 는 자식이 끝나면 같이 끝난다.
#   start_robot.sh : "<...>bash <...>start_robot.sh"
#   서버     : "<...>python -u <...>arm_server.py" 등 (start_robot.sh 의 실행 형태)
PAT_LAUNCHER='^[^ ]*python[^ ]* [^ ]*run_launcher\.py'
PAT_LAUNCHER_SUDO='^sudo [^ ]*python[^ ]* [^ ]*run_launcher\.py'
PAT_ROBOT='^[^ ]*bash [^ ]*start_robot\.sh'

sudo -v || exit 1          # 비밀번호 한 번 (이후 sudo 는 묻지 않음)

# 패턴에 맞는 프로세스에 SIGTERM → 최대 $2 초 대기 → 남으면 SIGKILL
stop_procs() {
  local pattern="$1" wait_sec="$2" label="$3"
  local pids
  pids=$(pgrep -f "$pattern" 2>/dev/null)
  [ -z "$pids" ] && return 0
  echo "[launcher] 기존 $label 발견 ($(echo $pids)) — 정상 종료 요청"
  sudo kill -TERM $pids 2>/dev/null
  for ((i = 0; i < wait_sec * 2; i++)); do
    pgrep -f "$pattern" >/dev/null 2>&1 || { echo "[launcher]   $label 종료됨"; return 0; }
    sleep 0.5
  done
  echo "[launcher]   $label ${wait_sec}초 내 미종료 — SIGKILL"
  sudo pkill -9 -f "$pattern" 2>/dev/null
  sleep 0.5
}

stop_procs "$PAT_LAUNCHER" 25 "launcher"
sudo pkill -9 -f "$PAT_LAUNCHER_SUDO" 2>/dev/null   # 남은 sudo 껍데기 (보통 이미 종료)
stop_procs "$PAT_ROBOT" 15 "start_robot.sh"
for name in "${TARGETS[@]}"; do
  stop_procs "^[^ ]*python[^ ]* (-u )?[^ ]*${name}\.py" 5 "${name}.py"
done

# 포트 확인 (다른 프로그램이 80 을 쓰고 있으면 어떤 프로세스인지 보여주고 중단)
if command -v ss >/dev/null 2>&1 && ss -ltn 2>/dev/null | grep -qE ':80[[:space:]]'; then
  echo "[launcher] ⚠️ 포트 80 사용 중 — 아래 프로세스를 확인하세요 (nginx/apache 등이면 중지 필요)"
  sudo ss -ltnp 2>/dev/null | grep -E ':80[[:space:]]'
  exit 1
fi

exec sudo "$PY" "$SCRIPT_DIR/run_launcher.py"
