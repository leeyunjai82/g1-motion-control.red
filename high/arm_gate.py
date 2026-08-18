"""
G1 loco(보행) <-> arm_sdk(팔 14축 + 허리) 전환 게이트  (v2)

v2 변경: 허리 3축(12 yaw, 13 roll, 14 pitch) 전부 kp 걸어 고정.
  - arm_sdk weight=1 이면 허리 3축이 모두 arm_sdk 소유가 됨.
  - v1 은 yaw(12)만 지령해서 roll/pitch 가 kp=0 무강성 상태
    -> 상체가 중력에 앞으로 굽는 증상. (xr_teleoperate 도 3축을 kp300 으로 lock)

원리
----
- rt/arm_sdk LowCmd 의 motor_cmd[29].q = arm_sdk 가중치(0.0~1.0)
- 1.0: arm_sdk 가 허리+팔 점유(보행 금지, 제자리) / 0.0: loco 가 되찾음(정상 보행)
- 걷기 전 반드시 release 로 weight 0 반납

안전
----
- hold() 는 실측각 동기화 후 램프 -> 튐 없음
- 허리 타겟을 매 주기 실측각으로 추종시키는 방식 금지(복원토크 0 -> 굽음).
  hold 시점에 1회 동기화 후 고정 목표각 유지.
- release 직후 loco 가 팔을 기본자세로 가져갈 수 있음 -> 첫 시험은 유인 감시
"""

import time
import threading

from unitree_sdk2py.core.channel import (
    ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize,
)
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC

# ---------------- 관절 인덱스 ----------------
WAIST_YAW, WAIST_ROLL, WAIST_PITCH = 12, 13, 14
WAIST_JOINTS = [WAIST_YAW, WAIST_ROLL, WAIST_PITCH]   # 3축 모두 고정 지령
ARM_JOINTS = list(range(15, 29))                      # L 15~21, R 22~28
CTRL_JOINTS = WAIST_JOINTS + ARM_JOINTS
NOT_USED_JOINT = 29                                   # motor_cmd[29].q = weight

KP_ARM, KD_ARM = 60.0, 1.5
KP_WAIST, KD_WAIST = 300.0, 3.0

RATE_HZ = 250.0
DT = 1.0 / RATE_HZ


class ArmGate:
    """rt/arm_sdk 점유/반납 게이트. loco 클라이언트와 함께 사용."""

    def __init__(self):
        self._crc = CRC()
        self._lock = threading.Lock()

        self._weight = 0.0
        self._targets = {j: 0.0 for j in CTRL_JOINTS}
        self._meas = {j: 0.0 for j in CTRL_JOINTS}
        self._got_state = False
        self._publishing = False

        self._pub = ChannelPublisher("rt/arm_sdk", LowCmd_)
        self._pub.Init()
        self._sub = ChannelSubscriber("rt/lowstate", LowState_)
        self._sub.Init(self._on_state, 10)

        self._th = threading.Thread(target=self._loop, daemon=True)
        self._th.start()

    # ---------------- 내부 ----------------
    def _on_state(self, msg: LowState_):
        for j in CTRL_JOINTS:
            self._meas[j] = msg.motor_state[j].q
        self._got_state = True

    def _make_cmd(self) -> LowCmd_:
        cmd = unitree_hg_msg_dds__LowCmd_()
        cmd.mode_pr = 0
        cmd.mode_machine = 0          # 실기 lowstate 는 5지만 arm_sdk 에는 0 유지
        with self._lock:
            w = self._weight
            tg = dict(self._targets)
        cmd.motor_cmd[NOT_USED_JOINT].q = w
        for j in CTRL_JOINTS:
            m = cmd.motor_cmd[j]
            m.q = tg[j]
            m.dq = 0.0
            m.tau = 0.0
            if j in WAIST_JOINTS:
                m.kp, m.kd = KP_WAIST, KD_WAIST
            else:
                m.kp, m.kd = KP_ARM, KD_ARM
        cmd.crc = self._crc.Crc(cmd)
        return cmd

    def _loop(self):
        while True:
            if self._publishing:
                self._pub.Write(self._make_cmd())
            time.sleep(DT)

    # ---------------- 공개 API ----------------
    def hold(self, ramp_s: float = 2.0):
        """실측각 동기화 -> weight 0->1 램프. 완료 후 팔/허리 지령 가능."""
        t0 = time.time()
        while not self._got_state:
            if time.time() - t0 > 3.0:
                raise RuntimeError("rt/lowstate 미수신 - 네트워크/인터페이스 확인")
            time.sleep(0.05)
        with self._lock:
            self._targets = dict(self._meas)   # 허리 3축 포함 전부 현재각 고정
            self._weight = 0.0
        self._publishing = True
        self._ramp_weight(1.0, ramp_s)

    def release(self, home: dict | None = None,
                move_s: float = 2.0, ramp_s: float = 2.0):
        """(선택) home 자세로 보간 -> weight 1->0 램프 -> 송신 중단.
        release 완료 후에만 loco 보행 재개할 것."""
        if home:
            self._interp_targets(home, move_s)
        self._ramp_weight(0.0, ramp_s)
        time.sleep(0.1)
        self._publishing = False

    def set_joints(self, targets: dict, move_s: float = 0.0):
        """관절 절대각 지령(rad). move_s>0 이면 보간. 예: {WAIST_YAW: 0.3}"""
        bad = set(targets) - set(CTRL_JOINTS)
        if bad:
            raise ValueError(f"제어 불가 관절: {bad}")
        if move_s > 0:
            self._interp_targets(targets, move_s)
        else:
            with self._lock:
                self._targets.update(targets)

    def set_waist_yaw(self, rad: float, move_s: float = 0.5):
        self.set_joints({WAIST_YAW: rad}, move_s)

    @property
    def active(self) -> bool:
        return self._publishing and self._weight >= 0.999

    # ---------------- 보조 ----------------
    def _ramp_weight(self, dst: float, dur: float):
        src = self._weight
        n = max(1, int(dur * RATE_HZ))
        for i in range(1, n + 1):
            with self._lock:
                self._weight = src + (dst - src) * i / n
            time.sleep(DT)

    def _interp_targets(self, dst: dict, dur: float):
        with self._lock:
            src = dict(self._targets)
        full = dict(src)
        full.update(dst)
        n = max(1, int(dur * RATE_HZ))
        for i in range(1, n + 1):
            a = i / n
            with self._lock:
                for j in CTRL_JOINTS:
                    self._targets[j] = src[j] + (full[j] - src[j]) * a
            time.sleep(DT)


# ---------------- 데모: 걷기 -> 팔+허리 -> 걷기 ----------------
if __name__ == "__main__":
    import sys
    from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient

    iface = sys.argv[1] if len(sys.argv) > 1 else "eth0"
    ChannelFactoryInitialize(0, iface)

    loco = LocoClient()
    loco.SetTimeout(10.0)
    loco.Init()

    gate = ArmGate()

    # Home: calm_down 마지막 프레임 (deg -> rad). 허리 3축 0.
    d = 3.14159265 / 180.0
    HOME = {
        WAIST_YAW: 0.0, WAIST_ROLL: 0.0, WAIST_PITCH: 0.0,
        15: 10 * d, 16:  15 * d, 17: 0.0, 18: 50 * d, 19: 0, 20: 0, 21: 0,
        22: 10 * d, 23: -15 * d, 24: 0.0, 25: 50 * d, 26: 0, 27: 0, 28: 0,
    }

    # 1) 걷기 (weight 0 상태)
    loco.Move(0.3, 0.0, 0.0)
    time.sleep(3.0)
    loco.StopMove()
    time.sleep(1.0)

    # 2) 팔+허리 모드
    gate.hold(ramp_s=2.0)
    gate.set_waist_yaw(0.4, move_s=1.0)
    gate.set_joints({18: 90 * d}, move_s=1.0)
    time.sleep(2.0)

    # 3) 반납 후 다시 걷기
    gate.release(home=HOME, move_s=2.0, ramp_s=2.0)
    time.sleep(0.5)
    loco.Move(0.3, 0.0, 0.0)
    time.sleep(3.0)
    loco.StopMove()
