#!/usr/bin/env bash
# G1 미션 스택 실행 스크립트 (마커 접근 → 파지 → 후진 → 운반 → 내려놓기)
# 위치: /home/circulus/project/g1-motion-control/start_mission.sh
# 사용: ./start_mission.sh
# 종료: Ctrl+C (TERM 후 8초 안 죽으면 KILL)
#
# 구성 (기동 순서 = 의존 순서):
#   rs_stream(50001) → detect_box(50010) → arm_server(50022)
#   → mission_server(50030) → marker_nav(50050)
#   (detect_marker(50011)는 이 스택에서 제외 — 마커는 marker_nav 가 자기
#    정면 카메라로 직접 검출한다. robot_server 스택에서는 계속 사용)
#
#   역할 분담 (외부에서 순서대로 호출):
#     POST :50050/approach  접근 — 마커 앞까지 가서 정지 (state="arrived")
#     POST :50030/grab      잡기 — 박스 정렬·파지·들기·후진 한 걸음
#     POST :50030/place     놓기 — 한걸음 전진·내려놓기·후진·release
#   접근은 잡기 전에도 놓기 전에도 동일(/approach 하나).
#   pilot_server(50040)는 별도 실행: python high/pilot_server.py
#
# ※ robot_server(50000)와 동시에 쓰지 말 것 — loco 명령 소스가 둘이 됨.
set -u

ROOT="$(cd "$(dirname "$0")" && pwd)"
LOG_DIR="$ROOT/logs"
mkdir -p "$LOG_DIR"
DAY="$(date '+%Y%m%d')"

TARGETS=("rs_stream.py" "ctrl/detect_box.py" \
         "arm_server.py" "mission_server.py" "marker_nav.py")

stamp() {
  awk '{ print strftime("[%Y-%m-%d %H:%M:%S]"), $0; fflush() }'
}

# ==========================================
# 0) 기존 좀비 프로세스 청소
# ==========================================
sweep_zombies() {
  local found=0
  for name in "${TARGETS[@]}"; do
    local base
    base=$(basename "$name")
    pids=$(pgrep -f "$base" 2>/dev/null || true)
    if [ -n "$pids" ]; then
      found=1
      echo "[cleanup] 기존 $base 발견: $pids — SIGKILL"
      pkill -9 -f "$base" 2>/dev/null || true
    fi
  done
  [ $found -eq 1 ] && sleep 0.5 || true
}
sweep_zombies

# ==========================================
# venv 활성화
# ==========================================
source "$ROOT/activate_tv.sh"
cd "$ROOT/high"

# detect_box 의 검출 루프는 ROBOT_SERVER/active_mode 가 "box" 일 때만 돈다.
# 미션 스택에는 robot_server(50000)가 없으므로 mission_server(50030)가 대신한다.
# (이 값이 없으면 detect_box 가 계속 잠들어 화면이 검게 나오고 frames=0)
export ROBOT_SERVER="http://localhost:50030"

# marker_nav 의 정면 USB 카메라. 번호가 밀리면 여기만 고치면 된다.
# (/dev/v4l/by-id/ 경로를 쓰면 재부팅에도 안 바뀜)
export MARKER_CAM="${MARKER_CAM:-/dev/video6}"

PIDS=()
NAMES=()

cleanup() {
  trap '' INT TERM EXIT
  echo ""
  echo "[stop] 종료 중..."
  for i in "${!PIDS[@]}"; do
    kill -TERM "${PIDS[$i]}" 2>/dev/null || true
  done
  for i in $(seq 1 8); do
    alive=0
    for pid in "${PIDS[@]}"; do
      kill -0 "$pid" 2>/dev/null && alive=1
    done
    [ $alive -eq 0 ] && break
    sleep 1
  done
  for i in "${!PIDS[@]}"; do
    if kill -0 "${PIDS[$i]}" 2>/dev/null; then
      echo "[stop] ${NAMES[$i]} 강제 종료"
      kill -9 "${PIDS[$i]}" 2>/dev/null || true
    fi
  done
  # 최종 sweep — PID 추적을 벗어나 살아남은 놈들 이름으로 정리
  for name in "${TARGETS[@]}"; do
    base=$(basename "$name")
    if pgrep -f "$base" > /dev/null 2>&1; then
      echo "[stop] $base 잔존 — 이름 기준 SIGKILL"
      pkill -9 -f "$base" 2>/dev/null || true
    fi
  done
  echo "[stop] 완료"
  exit 0
}
trap cleanup INT TERM EXIT

run() {  # run <이름> <스크립트경로> <대기초>
  echo "[start] $1 ..."
  python -u "$2" > >(stamp >> "$LOG_DIR/$1_$DAY.log") 2>&1 &
  PIDS+=($!); NAMES+=("$1")
  sleep "$3"
}

# ==========================================
# 기동 (의존 순서)
# ==========================================
run "rs_stream"      "rs_stream.py"             4
run "detect_box"     "ctrl/detect_box.py"       2
run "arm_server"     "arm_server.py"            4
run "mission_server" "mission_server.py"        3
run "marker_nav"     "marker_nav.py"            2

echo ""
echo "  ✓ 미션 스택 실행 중 (5개 서버)"
echo "    - marker_nav(접근)     : http://localhost:50050/"
echo "    - mission(잡기/놓기)   : http://localhost:50030/"
echo "    - pilot (별도 실행)    : python pilot_server.py → http://localhost:50040/"
echo "    - arm_server          : http://localhost:50022/status"
echo "    - box                 : http://localhost:50010/"
echo "    - 로그                 : $LOG_DIR/*_$DAY.log"
echo "    - 종료                 : Ctrl+C"
echo ""

wait
