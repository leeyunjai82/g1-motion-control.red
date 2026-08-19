"""
G1 "팔 든 채 보행" 단계별 실기 시험 (FSM 501 전제)

목적: hold(weight=1) 상태에서 release 없이 저속 보행이 성립하는지 확인.
각 단계는 Enter 로 진행, q+Enter 로 즉시 중단(정지+반납 후 종료).

시험 순서
  P1  기준 보행: release 상태 저속 직진 2s  (비교 기준)
  P2  hold -> CARRY 자세(가슴 앞 밀착) 보간
  P3  [핵심] release 없이 저속 직진 2s      <- 셔플/허리굽힘 관찰
  P4  (선택) 허리 yaw 스윕 +-0.4rad (다리 고정, 몸통만 회전)
  P5  HOME 보간 -> release -> 기준 보행 재확인

관찰 항목 (P3)
  - 전진하는가 / 제자리 셔플인가
  - 허리(pitch)가 굽는가                  -> 굽으면 즉시 q
  - 발 구름이 P1 대비 얼마나 무거운가

주의
  - 로봇은 FSM 501, 록스탠딩/보행 가능 상태에서 시작
  - rt/arm_sdk 로 쏘는 다른 프로세스(robot_server 등) 종료 필수
  - CARRY 자세 각도는 실기 전 시뮬레이터(simulator.py)로 반드시 확인.
    (G1 팔 0도 = '앞으로 나란히' 기준. 부호/각도가 기체와 다르면 수정)
  - 전방 2m 확보, 유인 감시. 가능하면 하네스.
"""

import sys
import time

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient

from arm_gate import ArmGate, WAIST_YAW, WAIST_ROLL, WAIST_PITCH

D = 3.14159265 / 180.0

# ---- 자세 정의 (deg 기준 -> rad). 실기 전 시뮬레이터로 확인할 것 ----
# HOME: calm_down 마지막 프레임
HOME = {
    WAIST_YAW: 0.0, WAIST_ROLL: 0.0, WAIST_PITCH: 0.0,
    15: 10 * D, 16:  15 * D, 17: 0.0, 18: 50 * D, 19: 0, 20: 0, 21: 0,
    22: 10 * D, 23: -15 * D, 24: 0.0, 25: 50 * D, 26: 0, 27: 0, 28: 0,
}
# CARRY: 박스를 가슴에 안는 자세(팔꿈치 크게 굽혀 손을 몸 쪽으로).
# 실기 확인: shoulder pitch 는 +가 뒤, -가 앞. (v1 의 +20 은 팔이 뒤로 감)
# 앞으로 들어올리려면 음수. 아래는 수정안 — P2 에서 자세 확인 후 미세조정.
CARRY = {
    WAIST_YAW: 0.0, WAIST_ROLL: 0.0, WAIST_PITCH: 0.0,
    15: -20 * D, 16:  10 * D, 17: 0.0, 18: 50 * D, 19: 0, 20: 0, 21: 0,
    22: -20 * D, 23: -10 * D, 24: 0.0, 25: 50 * D, 26: 0, 27: 0, 28: 0,
}

VX = 0.2          # 저속 직진 (m/s)
WALK_S = 5.0      # 보행 지속 시간


def gate_step(msg: str) -> bool:
    """Enter=진행, q=중단. 중단이면 False."""
    ans = input(f"\n[다음] {msg}  (Enter=진행 / q=중단) > ").strip().lower()
    return ans != "q"


def walk(loco, vx, vy, vyaw, dur):
    t_end = time.time() + dur
    while time.time() < t_end:
        loco.Move(vx, vy, vyaw)   # 20Hz 재전송
        time.sleep(0.05)
    loco.StopMove()
    time.sleep(1.5)

def main():
    iface = sys.argv[1] if len(sys.argv) > 1 else "eth0"
    ChannelFactoryInitialize(0, iface)

    loco = LocoClient()
    loco.SetTimeout(10.0)
    loco.Init()

    gate = ArmGate()
    held = False

    try:
        # P1 기준 보행
        if not gate_step(f"P1 기준 보행: release 상태 직진 {VX}m/s x {WALK_S}s"):
            return
        walk(loco, VX, 0, 0, WALK_S)
        print("P1 완료. 이 보행 품질이 비교 기준.")

        # P2 hold + CARRY
        if not gate_step("P2 hold(weight 0->1) 후 CARRY 자세 보간"):
            return
        gate.hold(ramp_s=2.0)
        held = True
        gate.set_joints(CARRY, move_s=3.0)
        print("P2 완료. 허리 굽힘 없는지, 팔이 가슴 앞인지 확인.")

        # P3 핵심 시험
        if not gate_step(f"P3 [핵심] release 없이 직진 {VX}m/s x {WALK_S}s"):
            return
        walk(loco, VX, 0, 0, WALK_S)
        print("P3 완료. 관찰: 전진 여부 / 셔플 / 허리 / 발구름.")

        # P4 선택: 허리 yaw 스윕 (arm_gate 직접 지령 - 몸통만 좌우 회전)
        if gate_step("P4 (선택) 허리 yaw 스윕 +-0.4rad (다리 고정, 몸통만 회전)"):
            gate.set_waist_yaw(0.4, move_s=1.5)
            time.sleep(0.5)
            gate.set_waist_yaw(-0.4, move_s=2.0)
            time.sleep(0.5)
            gate.set_waist_yaw(0.0, move_s=1.5)
            print("P4 완료.")

        # P5 복귀
        if not gate_step("P5 HOME 보간 -> release -> 기준 보행 재확인"):
            return
        gate.release(home=HOME, move_s=2.0, ramp_s=2.0)
        held = False
        time.sleep(0.5)
        walk(loco, VX, 0, 0, WALK_S)
        print("P5 완료. 전 과정 종료.")

    finally:
        # 어떤 경로로 빠져도: 정지 -> (점유 중이면) 안전 반납
        try:
            loco.StopMove()
        except Exception:
            pass
        time.sleep(1.0)
        if held:
            print("[안전] 점유 중 종료 -> HOME 복귀 후 반납")
            try:
                gate.release(home=HOME, move_s=2.0, ramp_s=2.0)
            except Exception as e:
                print(f"[경고] 반납 실패: {e} - 리모컨으로 댐핑 전환 후 회수")


if __name__ == "__main__":
    main()
