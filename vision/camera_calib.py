#!/usr/bin/env python3
"""
RealSense camera intrinsics를 출력.
출력 내용을 ik_box.py 상단에 복사 붙여넣기.
"""

import pyrealsense2 as rs


def main():
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    print("[CALIB] RealSense 시작 중...")
    profile = pipeline.start(config)

    try:
        for _ in range(15):
            pipeline.wait_for_frames()

        intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()

        print()
        print("=" * 60)
        print("아래 내용을 ik_box.py 의 카메라 캘리브레이션 섹션에 복사:")
        print("=" * 60)
        print()
        print(f"CAM_WIDTH  = {intr.width}")
        print(f"CAM_HEIGHT = {intr.height}")
        print(f"CAM_FX     = {intr.fx:.6f}")
        print(f"CAM_FY     = {intr.fy:.6f}")
        print(f"CAM_PPX    = {intr.ppx:.6f}")
        print(f"CAM_PPY    = {intr.ppy:.6f}")
        print(f"CAM_DIST   = {[float(c) for c in intr.coeffs]}")
        print()
        print("=" * 60)

    finally:
        pipeline.stop()


if __name__ == "__main__":
    main()
