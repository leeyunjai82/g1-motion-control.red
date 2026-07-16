#!/usr/bin/env python3
# Version: 0.2
"""
D435i IMU pitch 확인 (로봇 없이).

원본 check_rsimu.py에서 로봇 IMU 부분 제거.
카메라가 URDF의 CAMERA_PITCH (47.6도)에 맞게 기울어졌는지 확인용.

torso_abs = 0 (로봇 없으니 가정)
cam_to_torso = cam_abs - 0 = cam_abs
diff = cam_abs - URDF(47.6°)

카메라를 47.6° 아래로 정확히 기울이면 diff ≈ 0°
"""
import time
import numpy as np
import pyrealsense2 as rs


CAMERA_PITCH_URDF = 0.8307767239493009  # 47.6도


def start_camera():
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 200)
    pipeline.start(config)
    print("[D435i IMU] 시작")
    return pipeline


def get_accel_pitch(accel):
    """가속도계에서 카메라 pitch 계산.

    카메라가 평평하면 z = -g (중력이 카메라 -z 방향)
    카메라가 아래로 기울어지면 z 성분 줄고 y 성분 늘어남
    pitch = atan2(-z, sqrt(x^2 + y^2))
    """
    ax, ay, az = accel.x, accel.y, accel.z
    return float(np.arctan2(-az, np.sqrt(ax**2 + ay**2)))


def main():
    pipeline = start_camera()
    time.sleep(1.0)

    print("\n=== 실시간 D435i pitch (Ctrl+C 종료) ===")
    print("torso_abs = 0 가정 (로봇 없음)")
    print(f"목표: cam_abs ≈ URDF ({np.degrees(CAMERA_PITCH_URDF):.2f}°), diff ≈ 0°\n")
    print(f"{'cam_abs':>10} {'URDF':>8} {'diff':>8}")

    while True:
        try:
            frames = pipeline.wait_for_frames()
        except KeyboardInterrupt:
            break
        accel_frame = frames.first_or_default(rs.stream.accel)
        if not accel_frame:
            continue

        accel = accel_frame.as_motion_frame().get_motion_data()
        cam_abs = get_accel_pitch(accel)
        diff = cam_abs - CAMERA_PITCH_URDF

        print(f"{np.degrees(cam_abs):>10.2f}° "
              f"{np.degrees(CAMERA_PITCH_URDF):>8.2f}° "
              f"{np.degrees(diff):>+8.2f}°")
        time.sleep(0.2)


if __name__ == "__main__":
    main()
