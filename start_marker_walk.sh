#!/usr/bin/env bash
# G1 마커 추종 보행 실행 스크립트
# 위치: 레포 루트 (start_grab.sh 옆). marker_walk_server.py 는 high/ 에 둘 것.
# 사용: ./start_marker.sh [iface]     (기본 enp46s0)
# 종료: Ctrl+C (TERM 후 8초 안 죽으면 KILL)
#
# 띄우는 것:
#   - high/rs_stream.py           (50001, RealSense 카메라 스트림)
#   - high/ctrl/detect_marker.py  (50011, 마커 인식 - 50001 스트림 소비)
#   - high/marker_walk_server.py  (50040, 추종 웹 콘솔)
#
# 주의: robot_server(50000) 와 동시 실행 시 추종 중 웹 방향키 사용 금지.
set -u

IFACE="${1:-enp46s0}"

ROOT="$(cd "$(dirname "$0")" && pwd)"
LOG_DIR="$ROOT/logs"
mkdir -p "$LOG_DIR"
DAY="$(date '+%Y%m%d')"

TARGETS=("rs_stream.py" "detect_marker.py" "marker_walk_server.py")

stamp() {
  awk '{ print strftime("[%Y-%m-%d %H:%M:%S]"), $0; fflush() }'
}

# ==========================================
# 0) 기존 프로세스 청소
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
  echo "[stop] 종료 시퀀스 (marker_walk_server 먼저 — 정지 명령 보장)"
  # 추종 서버부터 TERM (finally 에서 StopMove 수행)
  for i in "${!PIDS[@]}"; do
    kill -TERM "${PIDS[$i]}" 2>/dev/null || true
  done
  for t in $(seq 1 8); do
    alive=0
    for pid in "${PIDS[@]}"; do
      kill -0 "$pid" 2>/dev/null && alive=1
    done
    [ $alive -eq 0 ] && break
    sleep 1
  done
  for i in "${!PIDS[@]}"; do
    if kill -0 "${PIDS[$i]}" 2>/dev/null; then
      echo "[stop] ${NAMES[$i]} 미종료 — SIGKILL"
      kill -9 "${PIDS[$i]}" 2>/dev/null || true
    fi
  done
  sweep_zombies "final"
  echo "[stop] 완료"
  exit 0
}
trap cleanup INT TERM EXIT

# ==========================================
# 1) rs_stream (50001) - 카메라
# ==========================================
echo "[start] rs_stream (50001) ..."
python -u rs_stream.py 2>&1 | stamp >> "$LOG_DIR/rs_stream_$DAY.log" &
PIDS+=($!); NAMES+=("rs_stream")
sleep 3

# ==========================================
# 2) detect_marker (50011)
# ==========================================
echo "[start] detect_marker (50011) ..."
python ctrl/detect_marker.py 2>&1 | stamp >> "$LOG_DIR/detect_marker_$DAY.log" &
PIDS+=($!); NAMES+=("detect_marker")
sleep 2

# ==========================================
# 3) marker_walk_server (50040)
# ==========================================
echo "[start] marker_walk_server (50040, iface=$IFACE) ..."
python marker_walk_server.py "$IFACE" 2>&1 | stamp >> "$LOG_DIR/marker_walk_$DAY.log" &
PIDS+=($!); NAMES+=("marker_walk_server")
sleep 1

echo ""
echo "=========================================="
echo "  마커 추종 콘솔 : http://localhost:50040/"
echo "  마커 인식 확인 : http://localhost:50011/"
echo "  로그           : $LOG_DIR/{rs_stream,detect_marker,marker_walk}_$DAY.log"
echo "  종료           : Ctrl+C"
echo "=========================================="
echo ""
echo "체크리스트:"
echo "  1) 로봇 FSM 501 / 보행 가능 상태"
echo "  2) 마커는 낮은 위치 (카메라 42도 하향)"
echo "  3) 첫 실행: 마커 왼쪽에 두고 부호 확인 (반대면 yaw_sign=-1)"
echo ""

wait
