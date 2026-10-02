#!/bin/bash
# G1 Launcher 웹 (http://<robot-ip>:50080/) — start_fsm.sh / start_robot.sh 버튼 실행
# start_fsm.sh 와 같이 sudo 로 tv 환경 python 을 실행한다 (비밀번호는 여기서 한 번만).
# 종료: Ctrl+C  (Robot 서버가 떠 있으면 웹에서 먼저 [Robot 정지])
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
sudo /home/circulus/miniconda3/envs/tv/bin/python "$SCRIPT_DIR/run_launcher.py"
