#!/usr/bin/env bash
# G1 통합 잡기 서비스 실행 스크립트
# 위치: /home/circulus/project/g1-motion-control/start_grab.sh
# 사용: ./start_grab.sh
# 종료: Ctrl+C (TERM 후 8초 안 죽으면 KILL)
#
# 시연 2종:
#   · box      (50010) : 정지 테이블 박스 양팔 파지
#   · conveyor (50012) : 움직이는 컨베이어 박스 파지 (한 손 대기 → range 진입 시 잡기)
#   ※ marker 제거됨
set -u

ROOT="$(cd "$(dirname "$0")" && pwd)"
LOG_DIR="$ROOT/logs"
mkdir -p "$LOG_DIR"

# 시작 날짜 (파일명 분리용). 자정 넘겨 계속 돌면 시작일 파일에 계속 쌓임.
DAY="$(date '+%Y%m%d')"

# 관리 대상 스크립트 (이름 기준 sweep)
TARGETS=("rs_stream.py" "robot_server.py" "dashboard.py" "detect_box.py" "detect_box_conv.py")

# ==========================================
# 로그 타임스탬프 필터
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
#   → 인식기 box(50010) / conveyor(50012)
#   ※ box/conveyor 는 서로 배타(active_mode 따라 한쪽만 detect, 나머지는 sleep)
# ==========================================
echo "[start] rs_stream       (50001) ..."
python -u rs_stream.py               > >(stamp >> "$LOG_DIR/rs_stream_$DAY.log")        2>&1 &
PIDS+=($!); NAMES+=("rs_stream")
sleep 2

echo "[start] robot_server    (50000) ..."
python -u robot_server.py            > >(stamp >> "$LOG_DIR/robot_server_$DAY.log")     2>&1 &
PIDS+=($!); NAMES+=("robot_server")
sleep 3

echo "[start] dashboard       (50003) ..."
python -u dashboard.py               > >(stamp >> "$LOG_DIR/dashboard_$DAY.log")        2>&1 &
PIDS+=($!); NAMES+=("dashboard")
sleep 2

echo "[start] detect_box      (50010) ..."
python -u ctrl/detect_box.py         > >(stamp >> "$LOG_DIR/detect_box_$DAY.log")       2>&1 &
PIDS+=($!); NAMES+=("detect_box")
sleep 1

echo "[start] detect_box_conv (50012) ..."
python -u ctrl/detect_box_conv.py    > >(stamp >> "$LOG_DIR/detect_box_conv_$DAY.log")  2>&1 &
PIDS+=($!); NAMES+=("detect_box_conv")
sleep 1

cat <<EOF
  ✓ 5개 서버 실행 중
    - Robot control : http://localhost:50000/          (제어 + 잡기)
    - Dashboard     : http://localhost:50003/dashboard (3D viewer + video + depth)
    - rs_stream     : http://localhost:50001/video_feed
    - detect_box    : http://localhost:50010/          (정지 테이블 박스)
    - detect_box_conv: http://localhost:50012/         (컨베이어 박스)

  시연 전환 (robot control 웹 또는 curl):
    - 정지 테이블 : curl -X POST 'http://localhost:50000/set_mode?mode=box'
    - 컨베이어    : curl -X POST 'http://localhost:50000/set_mode?mode=conveyor'
    - 해제        : curl -X POST 'http://localhost:50000/set_mode?mode=none'

  컨베이어 설정 (대기손/대기위치, 단위 m):
    - 오른손 대기 : curl 'http://localhost:50000/set_conveyor?hand=right&x=0.0&y=-0.20&z=0.0'
    - 왼손 대기   : curl 'http://localhost:50000/set_conveyor?hand=left&x=0.0&y=0.20&z=0.0'
    - range(zone)/자동 ON : http://localhost:50012/ 웹에서 설정

  로그: $LOG_DIR  (하루 단위 _$DAY + append + 타임스탬프)
  실시간: tail -f $LOG_DIR/robot_server_$DAY.log
  종료: Ctrl+C
EOF

wait
