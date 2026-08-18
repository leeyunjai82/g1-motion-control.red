#!/bin/bash
# nav_server 실행 (포트 50030)
# SLAM 서비스는 ROS/ROS2 환경과 충돌하므로 여기서 걷어냅니다.
cd "$(dirname "$0")"

unset ROS_DISTRO ROS_VERSION AMENT_PREFIX_PATH CMAKE_PREFIX_PATH
unset ROS_PACKAGE_PATH RMW_IMPLEMENTATION COLCON_PREFIX_PATH

IFACE="${1:-eth0}"
PCD="${NAV_PCD:-/home/unitree/test.pcd}"

echo "[start_nav] iface=$IFACE pcd=$PCD  UI=http://localhost:50030"
exec python3 nav_server.py --iface "$IFACE" --pcd "$PCD"
