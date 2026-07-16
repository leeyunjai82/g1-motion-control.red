#!/usr/bin/env python3
"""
D435i IMU + LowState IMU 동시 확인
카메라 pitch 보정값 검증용
"""
import time
import threading
import numpy as np
import pyrealsense2 as rs
from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_ as hg_LowState

CAMERA_PITCH_URDF = 0.8307767239493009  # 47.6도

latest_accel = None
pelvis_pitch = 0.0
waist_pitch = 0.0

def start_robot_imu():
    ChannelFactoryInitialize(0)
    sub = ChannelSubscriber("rt/lowstate", hg_LowState)
    sub.Init()

    def _read():
        global pelvis_pitch, waist_pitch
        while True:
            msg = sub.Read()
            if msg is not None:
                pelvis_pitch = float(msg.imu_state.rpy[1])
                waist_pitch  = float(msg.motor_state[14].q)  # WaistPitch
            time.sleep(0.01)

    t = threading.Thread(target=_read, daemon=True)
    t.start()
    print("[Robot IMU] 스레드 시작")

def start_camera():
    global latest_accel
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 200)
    pipeline.start(config)
    print("[D435i IMU] 시작")
    return pipeline

def get_accel_pitch(accel):
    ax, ay, az = accel.x, accel.y, accel.z
    return float(np.arctan2(-az, np.sqrt(ax**2 + ay**2)))  # p4

def main():
    global latest_accel

    start_robot_imu()
    time.sleep(1.0)

    pipeline = start_camera()
    time.sleep(1.0)

    print("\n=== 실시간 IMU 비교 (Ctrl+C 종료) ===")
    print(f"{'pelvis_p':>10} {'waist_p':>10} {'torso_abs':>10} {'cam_abs':>10} {'cam_to_torso':>14} {'URDF':>8} {'diff':>8}")

    while True:
        frames = pipeline.wait_for_frames()
        accel_frame = frames.first_or_default(rs.stream.accel)
        if not accel_frame:
            continue

        accel = accel_frame.as_motion_frame().get_motion_data()
        cam_abs = get_accel_pitch(accel)
        torso_abs = pelvis_pitch + waist_pitch
        cam_to_torso = cam_abs - torso_abs
        diff = cam_to_torso - CAMERA_PITCH_URDF

        print(f"{np.degrees(pelvis_pitch):>10.2f} "
              f"{np.degrees(waist_pitch):>10.2f} "
              f"{np.degrees(torso_abs):>10.2f} "
              f"{np.degrees(cam_abs):>10.2f} "
              f"{np.degrees(cam_to_torso):>14.2f} "
              f"{np.degrees(CAMERA_PITCH_URDF):>8.2f} "
              f"{np.degrees(diff):>8.2f}°")
        time.sleep(0.2)

if __name__ == "__main__":
    main()
