import os
import sys
import time
import json

from g1_motor_high import Custom

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("사용법: python3 run_motion.py <json_file_path> [network_interface]")
        sys.exit(1)

    motion_file_path = sys.argv[1]
    interface = sys.argv[2] if len(sys.argv) > 2 else "eth0"

    try:
        with open(motion_file_path, 'r', encoding='utf-8') as f:
            motion_data = json.load(f)
    except FileNotFoundError:
        print(f"오류: '{motion_file_path}' 파일을 찾을 수 없습니다.")
        sys.exit(1)
    except json.JSONDecodeError:
        print(f"오류: '{motion_file_path}' 파일이 올바른 JSON 형식이 아닙니다.")
        sys.exit(1)

    print("WARNING: Please ensure there are no obstacles around the robot.")
    input("Press Enter to continue...")

    #os.system('../g1_cmd --set_fsm_id=1'); time.sleep(5)
    #os.system('../g1_cmd --set_fsm_id=4'); time.sleep(5)
    #os.system('../g1_cmd --set_fsm_id=500'); time.sleep(5)

    custom = Custom(interface=interface)
    custom.Init()
    custom.Start()
    time.sleep(5)

    custom.set_motion(motion_data)

    print("\n동작 완료. 2초 후 초기 자세로 복귀합니다.")
    time.sleep(2)
    for n in custom.arm_joints:
        custom.command_new_move(n, 0, 1.5)
    time.sleep(1.6)

    print("종료하려면 Ctrl+C를 누르세요.")
    while True:
        time.sleep(1)
