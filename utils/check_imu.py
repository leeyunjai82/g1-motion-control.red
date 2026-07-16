#!/usr/bin/env python3
"""
LowState IMU 데이터 확인 스크립트
"""
import time
import threading
from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_ as hg_LowState

def main():
    ChannelFactoryInitialize(0)

    sub = ChannelSubscriber("rt/lowstate", hg_LowState)
    sub.Init()

    print("데이터 수신 대기 중...")
    while True:
        msg = sub.Read()
        if msg is not None:
            break
        time.sleep(0.1)

    print("\n=== dir(msg) ===")
    print(dir(msg))

    print("\n=== dir(msg.imu_state) ===")
    try:
        print(dir(msg.imu_state))
    except Exception as e:
        print(f"imu_state 없음: {e}")

    print("\n=== IMU 값 실시간 출력 (Ctrl+C로 종료) ===")
    while True:
        msg = sub.Read()
        if msg is None:
            continue
        try:
            rpy = msg.imu_state.rpy
            acc = msg.imu_state.accelerometer
            gyro = msg.imu_state.gyroscope
            quat = msg.imu_state.quaternion
            print(f"RPY: [{rpy[0]:.3f}, {rpy[1]:.3f}, {rpy[2]:.3f}]  "
                  f"Accel: [{acc[0]:.3f}, {acc[1]:.3f}, {acc[2]:.3f}]  "
                  f"Gyro: [{gyro[0]:.3f}, {gyro[1]:.3f}, {gyro[2]:.3f}]")
        except Exception as e:
            print(f"오류: {e}")
            break
        time.sleep(0.2)

if __name__ == "__main__":
    main()
