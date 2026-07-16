#!/usr/bin/env python3
"""
OpenVINO에서 사용 가능한 디바이스(CPU/GPU/NPU)를 나열합니다.
"""

import openvino as ov


def main():
    core = ov.Core()
    devices = core.available_devices

    print(f"OpenVINO version: {ov.__version__}")
    print(f"Available devices: {devices}\n")

    for device in devices:
        print(f"=== {device} ===")
        try:
            full_name = core.get_property(device, "FULL_DEVICE_NAME")
            print(f"  Full name      : {full_name}")
        except Exception as e:
            print(f"  Full name      : (unavailable: {e})")

        # 지원 속성 전체 보기 (선택)
        try:
            supported = core.get_property(device, "SUPPORTED_PROPERTIES")
            for prop in supported:
                if prop in ("SUPPORTED_PROPERTIES",):
                    continue
                try:
                    val = core.get_property(device, prop)
                    print(f"  {prop:30s}: {val}")
                except Exception:
                    pass
        except Exception:
            pass
        print()


if __name__ == "__main__":
    main()
