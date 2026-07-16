#!/usr/bin/env bash
# G1 Motion Runner 실행 스크립트
# 위치: /home/circulus/project/g1-motion-control/start_motion.sh
# 사용: ./start_motion.sh
# 종료: Ctrl+C (TERM 후 8초 안 죽으면 KILL)
set -u

ROOT="$(cd "$(dirname "$0")" && pwd)"
LOG_DIR="$ROOT/logs"
mkdir -p "$LOG_DIR"

# 관리 대상 스크립트
TARGETS=("run_motion.py")

# ==========================================
# 0) 기존 좀비 프로세스 청소 (시작 전)
# ==========================================
sweep_zombies() {
  local label="$1"
  local found=0
  for name in "${TARGETS[@]}"; do
    pids=$(pgrep -f "$name" 2>/dev/null || true)
    if [ -n "$pids" ]; then
      found=1
      echo "[$label] 기존 $name 발견: $pids — SIGKILL"
      pkill -9 -f "$name" 2>/dev/null || true
    fi
  done
  [ $found -eq 1 ] && sleep 0.5 || true
}

sweep_zombies "cleanup"

# ==========================================
# venv 활성화
# ==========================================
source "$ROOT/activate_tv.sh"
cd "$ROOT/high"

PIDS=()
NAMES=()

# ==========================================
# 종료 처리
# ==========================================
cleanup() {
  trap '' INT TERM EXIT
  echo ""
  echo "[stop] 서버 종료 중..."

  # 1단계: 자식들에 SIGTERM
  for i in "${!PIDS[@]}"; do
    pid=${PIDS[$i]}
    if kill -0 "$pid" 2>/dev/null; then
      kill -TERM "$pid" 2>/dev/null
    fi
  done

  # 최대 8초 대기
  for s in 1 2 3 4 5 6 7 8; do
    alive=0
    for pid in "${PIDS[@]}"; do
      kill -0 "$pid" 2>/dev/null && alive=1
    done
    [ $alive -eq 0 ] && break
    sleep 1
  done

  # 2단계: 추적 PID 중 살아있는 것 SIGKILL
  for i in "${!PIDS[@]}"; do
    pid=${PIDS[$i]}
    if kill -0 "$pid" 2>/dev/null; then
      echo "[stop] 강제 종료(KILL): ${NAMES[$i]} (pid=$pid)"
      kill -KILL "$pid" 2>/dev/null
    fi
  done

  # 3단계: PPID=1로 reparent된 좀비까지 이름 기준으로 sweep
  sweep_zombies "stop"

  echo "[stop] 완료"
  exit 0
}
trap cleanup INT TERM

# ==========================================
# 시작
# ==========================================
echo "[start] run_motion (50003) ..."
python -u run_motion.py > "$LOG_DIR/run_motion.log" 2>&1 &
PIDS+=($!); NAMES+=("run_motion")
sleep 1

cat <<EOF
  ✓ Motion Runner 실행 중
    - 통합 UI:    http://localhost:50003/
    - 로봇만:     http://localhost:50003/robot-only
    - API docs:   http://localhost:50003/docs

  로그: $LOG_DIR/run_motion.log
  실시간 보기: tail -f $LOG_DIR/run_motion.log
  종료: Ctrl+C
EOF

wait
