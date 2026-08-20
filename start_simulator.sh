#!/usr/bin/env bash
# G1 Motion Editor(통합 시뮬레이터) 실행 스크립트
# 위치: /home/circulus/project/g1-motion-control/start_simulator.sh
# 사용: ./start_simulator.sh
# 종료: Ctrl+C (TERM 후 8초 안 죽으면 KILL)
#
# 동작:
#   - arm_server(50022)가 이미 떠 있으면(예: start_robot.sh 실행 중) 재사용
#   - 없으면 arm_server를 직접 띄운 뒤 simulator(8000) 기동
#   - 종료 시 이 스크립트가 띄운 프로세스만 정리 (남의 arm_server는 안 건드림)
set -u

ROOT="$(cd "$(dirname "$0")" && pwd)"
LOG_DIR="$ROOT/logs"
mkdir -p "$LOG_DIR"
DAY="$(date '+%Y%m%d')"

stamp() {
  awk '{ print strftime("[%Y-%m-%d %H:%M:%S]"), $0; fflush() }'
}

# ==========================================
# 0) 기존 simulator 좀비 정리 (simulator만 — arm_server는 공용이라 안 죽임)
# ==========================================
pids=$(pgrep -f "simulator.py" 2>/dev/null || true)
if [ -n "$pids" ]; then
  echo "[cleanup] 기존 simulator.py 발견: $pids — SIGKILL"
  pkill -9 -f "simulator.py" 2>/dev/null || true
  sleep 0.5
fi

# ==========================================
# venv 활성화
# ==========================================
source "$ROOT/activate_tv.sh"
cd "$ROOT/high"

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
  echo "[stop] 완료"
  exit 0
}
trap cleanup INT TERM EXIT

# ==========================================
# 1) arm_server — 이미 떠 있으면 재사용, 없으면 기동
# ==========================================
if curl -s -m 1 http://localhost:50022/status > /dev/null 2>&1; then
  echo "[start] arm_server    (50022) 이미 실행 중 — 재사용"
else
  echo "[start] arm_server    (50022) ..."
  python -u arm_server.py > >(stamp >> "$LOG_DIR/arm_server_$DAY.log") 2>&1 &
  PIDS+=($!); NAMES+=("arm_server")
  # 준비될 때까지 대기 (최대 15초)
  for i in $(seq 1 15); do
    curl -s -m 1 http://localhost:50022/status > /dev/null 2>&1 && break
    sleep 1
  done
  if ! curl -s -m 1 http://localhost:50022/status > /dev/null 2>&1; then
    echo "[start] ⚠ arm_server 응답 없음 — 로그 확인: $LOG_DIR/arm_server_$DAY.log"
  fi
fi

# ==========================================
# 2) simulator (8000)
# ==========================================
echo "[start] simulator     (8000) ..."
python -u simulator.py > >(stamp >> "$LOG_DIR/simulator_$DAY.log") 2>&1 &
PIDS+=($!); NAMES+=("simulator")
sleep 2

echo ""
echo "  ✓ Motion Editor 실행 중"
echo "    - simulator  : http://localhost:8000/          (Joint/IK 통합 편집기)"
echo "    - arm_server : http://localhost:50022/status"
echo "    - 로그        : $LOG_DIR/{simulator,arm_server}_$DAY.log"
echo "    - 종료        : Ctrl+C"
echo ""

wait
