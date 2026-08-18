#!/usr/bin/env python3
"""
허리 Roll/Pitch를 loco에 남기고 걸을 수 있는지 확인하는 시험 스크립트.

가설
  rt/arm_sdk 로 팔 14축 + 허리 3축을 전부 kp>0 으로 잡으면 loco 컨트롤러가
  상체로 각운동량을 흘리지 못해 제자리걸음이 난다.
  균형에 쓰이는 축은 허리 Roll(13) / Pitch(14) 이므로, 이 둘만 kp=kd=0 으로
  **기동 시점부터** 비워두면 loco가 계속 소유하고, 팔과 허리 Yaw(12)는
  우리가 계속 제어할 수 있다.

중요
  런타임에 kp를 150 -> 0 으로 떨어뜨리는 것과 다르다. 그건 이미 실기에서
  로봇이 접혔다. 이 스크립트는 처음부터 0으로 시작한다.

사용
  sudo <python> test_waist_yaw_only.py [--iface eth0] [--mode A|B]
    A (기본) : Roll/Pitch 를 loco 에 남김  ← 검증 대상
    B        : 기존과 동일하게 3축 전부 잡음  ← 비교용 기준선

  실행 후 키보드:
    w/s  전진/후진      a/d  좌/우 이동      q/e  좌/우 선회
    space 정지          p    팔 내림(park)   o    팔 0도
    z/x  허리 Yaw -/+   c    허리 Yaw 0
    i    현재 상태 출력  Ctrl-C  종료(제어권 반납)

주의
  * 반드시 FSM 501(stand + arm sdk) 상태에서 실행할 것. start_fsm.sh stand
  * 첫 시험은 로봇을 붙잡을 수 있는 상태에서. 서 있기부터 확인하고 걷기로.
  * robot_server.py 가 떠 있으면 먼저 종료할 것 (arm_sdk 토픽 충돌).
"""

import argparse
import sys
import termios
import threading
import time
import tty

import numpy as np

from unitree_sdk2py.core.channel import (ChannelFactoryInitialize,
                                         ChannelPublisher, ChannelSubscriber)
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_ as hg_LowCmd
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_ as hg_LowState
from unitree_sdk2py.utils.crc import CRC

try:
    from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
    LOCO_OK = True
except ImportError as e:
    print(f"⚠️ LocoClient 로드 실패: {e}")
    LOCO_OK = False

TOPIC_ARM_SDK = "rt/arm_sdk"
TOPIC_LOWSTATE = "rt/lowstate"

NUM_MOTORS = 35
WEIGHT_JOINT = 29           # kNotUsedJoint0

WAIST_YAW, WAIST_ROLL, WAIST_PITCH = 12, 13, 14
ARM_IDS = list(range(15, 29))          # 15~28, 좌7 + 우7
WRIST_IDS = {19, 20, 21, 26, 27, 28}

KP_ARM, KD_ARM = 80.0, 3.0
KP_WRIST, KD_WRIST = 40.0, 1.5
KP_WAIST, KD_WAIST = 150.0, 3.0

# calm_down.json 마지막 프레임 (팔 내림)
PARK_DEG = [10.0, 15.0, 0.0, 50.0, 0.0, 0.05, 0.05,
            10.0, -15.0, 0.0, 50.0, 0.0, 0.05, 0.05]

VX, VY, VYAW = 0.3, 0.2, 0.4


class Tester:
    def __init__(self, mode="A"):
        self.mode = mode
        self.free_ids = ([WAIST_ROLL, WAIST_PITCH] if mode == "A" else [])

        self.crc = CRC()
        self.state = None
        self.lock = threading.Lock()
        self.running = True

        self.sub = ChannelSubscriber(TOPIC_LOWSTATE, hg_LowState)
        self.sub.Init(self._on_state, 10)
        print("lowstate 대기...")
        while self.state is None:
            time.sleep(0.1)
        print("lowstate 수신 OK")

        self.pub = ChannelPublisher(TOPIC_ARM_SDK, hg_LowCmd)
        self.pub.Init()

        self.msg = unitree_hg_msg_dds__LowCmd_()
        self.msg.mode_pr = 0
        self.msg.mode_machine = 0

        q0 = self.current_q()
        for i in range(NUM_MOTORS):
            self.msg.motor_cmd[i].mode = 1
            self.msg.motor_cmd[i].q = float(q0[i]) if i < len(q0) else 0.0
            self.msg.motor_cmd[i].dq = 0.0
            self.msg.motor_cmd[i].tau = 0.0
            self.msg.motor_cmd[i].kp = 0.0
            self.msg.motor_cmd[i].kd = 0.0

        for i in ARM_IDS:
            self.msg.motor_cmd[i].kp = KP_WRIST if i in WRIST_IDS else KP_ARM
            self.msg.motor_cmd[i].kd = KD_WRIST if i in WRIST_IDS else KD_ARM

        # 핵심: 여기서 Roll/Pitch 는 kp=kd=0 인 채로 남는다
        for i in (WAIST_YAW, WAIST_ROLL, WAIST_PITCH):
            if i in self.free_ids:
                continue
            self.msg.motor_cmd[i].kp = KP_WAIST
            self.msg.motor_cmd[i].kd = KD_WAIST

        held = [i for i in (WAIST_YAW, WAIST_ROLL, WAIST_PITCH)
                if i not in self.free_ids]
        print(f"\n[모드 {mode}] 허리 점유={held} / loco에 남김={self.free_ids}")

        # 지령 타겟 (rad)
        self.arm_target = np.array(q0[15:29], dtype=float)
        self.waist_yaw_target = float(q0[WAIST_YAW])

        self.weight = 0.0
        self.pub_thread = threading.Thread(target=self._publish_loop, daemon=True)
        self.pub_thread.start()

        self.loco = None
        if LOCO_OK:
            self.loco = LocoClient()
            self.loco.SetTimeout(0.0001)
            self.loco.Init()

    # ---------------- DDS ----------------
    def _on_state(self, msg):
        self.state = msg

    def current_q(self):
        return np.array([self.state.motor_state[i].q for i in range(NUM_MOTORS)])

    def _publish_loop(self):
        while self.running:
            t0 = time.time()
            with self.lock:
                self.msg.motor_cmd[WEIGHT_JOINT].q = float(self.weight)
                for n, i in enumerate(ARM_IDS):
                    self.msg.motor_cmd[i].q = float(self.arm_target[n])
                if WAIST_YAW not in self.free_ids:
                    self.msg.motor_cmd[WAIST_YAW].q = float(self.waist_yaw_target)
                # Roll/Pitch: 모드 A 에서는 kp=0 이라 q 값이 무의미. 건드리지 않는다.
                self.msg.crc = self.crc.Crc(self.msg)
            self.pub.Write(self.msg)
            time.sleep(max(0.0, 0.002 - (time.time() - t0)))

    # ---------------- 제어권 ----------------
    def ramp_weight(self, to, sec=1.5):
        frm = self.weight
        n = max(2, int(sec / 0.02))
        for w in np.linspace(frm, to, num=n):
            self.weight = float(w)
            time.sleep(sec / n)
        self.weight = float(to)
        print(f"weight {frm:.1f} -> {to:.1f}")

    # ---------------- 동작 ----------------
    def move_arm(self, target_deg, sec=2.0):
        start = self.arm_target.copy()
        goal = np.radians(np.array(target_deg, dtype=float))
        n = max(2, int(sec * 100))
        for k in range(n + 1):
            r = 0.5 * (1 - np.cos(np.pi * k / n))     # ease in-out
            with self.lock:
                self.arm_target = start + (goal - start) * r
            time.sleep(sec / n)

    def move_yaw(self, deg, sec=1.0):
        if WAIST_YAW in self.free_ids:
            print("허리 Yaw는 loco 소유 — 지령 무시")
            return
        start = self.waist_yaw_target
        goal = np.radians(deg)
        n = max(2, int(sec * 100))
        for k in range(n + 1):
            r = 0.5 * (1 - np.cos(np.pi * k / n))
            with self.lock:
                self.waist_yaw_target = start + (goal - start) * r
            time.sleep(sec / n)

    def walk(self, vx, vy, vyaw):
        if self.loco:
            self.loco.Move(vx, vy, vyaw, continous_move=False)

    def report(self):
        q = self.current_q()
        print(f"\n  weight={self.weight:.2f}  모드={self.mode}")
        print(f"  허리 실측(deg) Yaw={np.degrees(q[12]):7.2f} "
              f"Roll={np.degrees(q[13]):7.2f} Pitch={np.degrees(q[14]):7.2f}")
        print(f"  허리 kp        Yaw={self.msg.motor_cmd[12].kp:5.0f} "
              f"Roll={self.msg.motor_cmd[13].kp:5.0f} "
              f"Pitch={self.msg.motor_cmd[14].kp:5.0f}")
        print(f"  어깨 Pitch(deg) L={np.degrees(q[15]):7.2f} R={np.degrees(q[22]):7.2f}\n")

    def shutdown(self):
        print("\n제어권 반납 중...")
        self.walk(0, 0, 0)
        self.ramp_weight(0.0, 1.5)
        self.running = False
        time.sleep(0.2)
        print("종료")


def getkey():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


HELP = """
  w/s 전진/후진   a/d 좌/우   q/e 선회   space 정지
  p 팔내림   o 팔0도   z/x 허리Yaw-/+   c 허리Yaw0
  i 상태   Ctrl-C 종료
"""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--iface", default="eth0")
    ap.add_argument("--mode", default="A", choices=["A", "B"])
    args = ap.parse_args()

    ChannelFactoryInitialize(0, args.iface)
    t = Tester(mode=args.mode)

    print("\n" + "=" * 58)
    print("1) 지금은 weight=0. 로봇에 아무 지령도 안 나가는 상태다.")
    print("2) Enter 를 누르면 weight 를 1 로 올린다.")
    print("3) 올린 직후 '걷지 말고' 10초간 서 있는지부터 볼 것.")
    print("   허리가 접히면 Ctrl-C. 모드 A 가설이 틀린 것이다.")
    print("=" * 58)
    input("\nEnter 로 시작: ")

    t.ramp_weight(1.0, 2.0)
    t.report()
    print(HELP)

    try:
        while True:
            k = getkey()
            if k == "\x03":
                break
            elif k == "w":
                t.walk(VX, 0, 0)
            elif k == "s":
                t.walk(-VX, 0, 0)
            elif k == "a":
                t.walk(0, VY, 0)
            elif k == "d":
                t.walk(0, -VY, 0)
            elif k == "q":
                t.walk(0, 0, VYAW)
            elif k == "e":
                t.walk(0, 0, -VYAW)
            elif k == " ":
                t.walk(0, 0, 0)
            elif k == "p":
                t.move_arm(PARK_DEG)
            elif k == "o":
                t.move_arm([0.0] * 14)
            elif k == "z":
                t.move_yaw(np.degrees(t.waist_yaw_target) - 15)
            elif k == "x":
                t.move_yaw(np.degrees(t.waist_yaw_target) + 15)
            elif k == "c":
                t.move_yaw(0.0)
            elif k == "i":
                t.report()
    except KeyboardInterrupt:
        pass
    finally:
        t.shutdown()


if __name__ == "__main__":
    main()
