#!/usr/bin/env bash
# G1 통합 잡기 서비스 실행 스크립트
# 위치: /home/circulus/project/g1-motion-control/start_grab.sh
# 사용: ./start_grab.sh
# 종료: Ctrl+C (TERM 후 8초 안 죽으면 KILL)
set -u

ROOT="$(cd "$(dirname "$0")" && pwd)"
LOG_DIR="$ROOT/logs"
mkdir -p "$LOG_DIR"

# 시작 날짜 (파일명 분리용). 자정 넘겨 계속 돌면 시작일 파일에 계속 쌓임.
DAY="$(date '+%Y%m%d')"

# 관리 대상 스크립트 (이름 기준 sweep)
TARGETS=("rs_stream.py" "robot_server.py" "dashboard.py" "detect_marker.py" "detect_box.py")

# ==========================================
# 로그 타임스탬프 필터
#   - 각 줄 앞에 [YYYY-MM-DD HH:MM:SS] 부착
#   - awk 한 프로세스로 처리(줄마다 date 호출 안 함) + fflush로 즉시 기록
#   - gawk의 strftime 사용. mawk만 있으면 'sudo apt install gawk'
# ==========================================
stamp() {
  awk '{ print strftime("[%Y-%m-%d %H:%M:%S]"), $0; fflush() }'
}

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

  # 최대 8초 대기 (robot_server의 arm go_home 종료 시퀀스 + 여유)
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
#   rs_stream(50001) → robot_server(50000, arm 제어) → dashboard(50003, 뷰어)
#   → 인식기(50011/50010)
#   ※ 로그: 하루 단위 파일(_$DAY) + append(>>) + 줄별 타임스탬프
#     $!는 python PID 유지(graceful 종료 보존)
# ==========================================
echo "[start] rs_stream     (50001) ..."
python -u rs_stream.py          > >(stamp >> "$LOG_DIR/rs_stream_$DAY.log")     2>&1 &
PIDS+=($!); NAMES+=("rs_stream")
sleep 2

echo "[start] robot_server  (50000) ..."
python -u robot_server.py       > >(stamp >> "$LOG_DIR/robot_server_$DAY.log")  2>&1 &
PIDS+=($!); NAMES+=("robot_server")
sleep 3

echo "[start] dashboard     (50003) ..."
python -u dashboard.py          > >(stamp >> "$LOG_DIR/dashboard_$DAY.log")     2>&1 &
PIDS+=($!); NAMES+=("dashboard")
sleep 2

echo "[start] detect_marker (50011) ..."
python -u ctrl/detect_marker.py > >(stamp >> "$LOG_DIR/detect_marker_$DAY.log") 2>&1 &
PIDS+=($!); NAMES+=("detect_marker")
sleep 1

echo "[start] detect_box    (50010) ..."
python -u ctrl/detect_box.py    > >(stamp >> "$LOG_DIR/detect_box_$DAY.log")    2>&1 &
PIDS+=($!); NAMES+=("detect_box")
sleep 1

cat <<EOF
  ✓ 5개 서버 실행 중
    - Robot control : http://localhost:50000/          (제어 + 잡기)
    - Dashboard     : http://localhost:50003/dashboard (3D viewer + video + depth)
    - rs_stream     : http://localhost:50001/video_feed
    - detect_marker : http://localhost:50011/          (마커 인식)
    - detect_box    : http://localhost:50010/          (박스 인식)

  사용:
    1) http://localhost:50000/ 접속 (제어)
    2) Grab Mode에서 Marker / Box 선택
    3) 자동: 인식 웹(50011/50010)에서 자동 ON + 영역 설정
       수동: 50000 웹의 [수동 잡기]
    4) 로봇 시각화는 http://localhost:50003/dashboard

  로그: $LOG_DIR  (하루 단위 _$DAY + append + 타임스탬프)
  실시간: tail -f $LOG_DIR/robot_server_$DAY.log
  종료: Ctrl+C
EOF

wait
