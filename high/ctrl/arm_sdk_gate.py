"""
arm_sdk 제어권 수동 토글 (걷기 모드 <-> 잡기 모드)

배경
  ArmControllerWrapper(motion_mode=True)가 살아있는 동안 rt/arm_sdk가 250Hz로
  팔 14축 + 허리 3축(kp=150)을 붙잡는다. 이때 loco 컨트롤러는 상체로 각운동량을
  흘리지 못해 발로 상쇄하려 하고, 결과적으로 제자리걸음 / 균형 불안정이 생긴다.

동작
  release : q_target을 실측각에 계속 동기화(follow) 하면서 weight 1 -> 0 램프.
            arm_sdk 지령이 무효화되어 loco가 상체를 자유롭게 쓴다. (걷기 모드)
  hold    : follow 중단 -> 실측각으로 q_target / waist_q_target 최종 동기화 ->
            weight 0 -> 1 램프. 팔이 튀지 않는다. (잡기 모드)

주의
  자동 전환은 넣지 않는다. 잡기 시퀀스 도중 release가 걸리면 데모가 깨진다.
"""

import threading
import time

import numpy as np

from ctrl.robot_arm import G1_29_JointIndex

# arm_sdk 제어권 가중치가 실리는 가상 관절 (29번)
WEIGHT_JOINT = G1_29_JointIndex.kNotUsedJoint0

STATE_HOLD = "hold"        # arm_sdk 점유 중 (잡기 가능)
STATE_RELEASE = "release"  # 제어권 반납 (걷기용)
STATE_PARTIAL = "partial"  # 부분 점유 (박스 들고 걷기)
STATE_RAMPING = "ramping"  # 램프 진행 중


class ArmSdkGate:
    """arm_sdk weight를 수동으로 0/1 토글한다."""

    def __init__(self, arm, ramp_sec=1.0, follow_hz=50.0):
        self.arm = arm
        self.ramp_sec = float(ramp_sec)
        self.follow_hz = float(follow_hz)

        self._lock = threading.Lock()
        self._weight = 1.0            # robot_arm._ctrl_motor_state가 1.0으로 시작
        self._state = STATE_HOLD
        self._follow_stop = threading.Event()
        self._follow_thread = None

    # ------------------------------------------------------------------
    # 상태
    # ------------------------------------------------------------------
    @property
    def available(self):
        ctrl = getattr(self.arm, "arm_ctrl", None) if self.arm else None
        return bool(ctrl is not None and getattr(ctrl, "motion_mode", False))

    @property
    def state(self):
        return self._state if self.available else "n/a"

    def status(self):
        return {
            "ok": self.available,
            "state": self.state,
            "weight": round(float(self._weight), 3),
            "available": self.available,
        }

    # ------------------------------------------------------------------
    # 내부 유틸
    # ------------------------------------------------------------------
    def _set_weight(self, w):
        # publish 스레드가 매 주기 msg를 그대로 쏘므로 필드만 바꾸면 즉시 반영된다.
        w = float(np.clip(w, 0.0, 1.0))
        self.arm.arm_ctrl.msg.motor_cmd[WEIGHT_JOINT].q = w
        self._weight = w

    def _ramp(self, w_from, w_to, sec):
        sec = max(0.05, float(sec))
        steps = max(2, int(sec / 0.02))
        dt = sec / steps
        for w in np.linspace(w_from, w_to, num=steps):
            self._set_weight(w)
            time.sleep(dt)
        self._set_weight(w_to)

    def sync_targets(self):
        """실측 관절각 -> q_target / waist_q_target (bumpless 복귀용)."""
        ctrl = self.arm.arm_ctrl
        arm_q = np.asarray(ctrl.get_current_dual_arm_q(), dtype=float)
        all_q = np.asarray(ctrl.get_current_motor_q(), dtype=float)
        with ctrl.ctrl_lock:
            ctrl.q_target = arm_q
            ctrl.tauff_target = np.zeros(14)
            ctrl.waist_q_target = all_q[12:15].copy()

    def _follow_loop(self):
        dt = 1.0 / self.follow_hz
        while not self._follow_stop.is_set():
            try:
                self.sync_targets()
            except Exception:
                pass
            time.sleep(dt)

    def _start_follow(self):
        if self._follow_thread and self._follow_thread.is_alive():
            return
        self._follow_stop.clear()
        self._follow_thread = threading.Thread(target=self._follow_loop, daemon=True)
        self._follow_thread.start()

    def _stop_follow(self):
        self._follow_stop.set()
        t = self._follow_thread
        if t and t.is_alive():
            t.join(timeout=0.5)
        self._follow_thread = None

    # ------------------------------------------------------------------
    # 공개 API
    # ------------------------------------------------------------------
    def release(self, ramp_sec=None):
        """걷기 모드: arm_sdk 제어권 반납 (weight 1 -> 0)."""
        if not self.available:
            return {"ok": False, "reason": "arm_sdk 사용 불가 (arm 미초기화 또는 motion_mode=False)"}
        with self._lock:
            if self._state == STATE_RELEASE:
                return self.status()
            self._state = STATE_RAMPING
            try:
                self.arm.stop_motion()      # 진행 중 보간 중단
            except Exception:
                pass
            time.sleep(0.05)
            try:
                self.sync_targets()         # 램프 구간에서 팔이 끌리지 않게
                self._start_follow()
            except Exception as e:
                print(f"[ARM_SDK] release sync 실패: {e}")
            self._ramp(self._weight, 0.0, ramp_sec or self.ramp_sec)
            self._state = STATE_RELEASE
        print("[ARM_SDK] release — 걷기 모드 (weight=0)")
        return self.status()

    def hold(self, ramp_sec=None):
        """잡기 모드: 실측각 동기화 후 arm_sdk 제어권 확보 (weight 0 -> 1)."""
        if not self.available:
            return {"ok": False, "reason": "arm_sdk 사용 불가 (arm 미초기화 또는 motion_mode=False)"}
        with self._lock:
            if self._state == STATE_HOLD:
                return self.status()
            self._state = STATE_RAMPING
            self._stop_follow()
            try:
                self.sync_targets()         # 이게 빠지면 복귀 순간 허리가 튄다
            except Exception as e:
                print(f"[ARM_SDK] hold sync 실패: {e}")
            self._ramp(self._weight, 1.0, ramp_sec or self.ramp_sec)
            self._state = STATE_HOLD
        print("[ARM_SDK] hold — 잡기 모드 (weight=1)")
        return self.status()

    # ------------------------------------------------------------------
    # 부분 제어 (박스 들고 걷기용)
    # ------------------------------------------------------------------
    def set_weight(self, weight, ramp_sec=None):
        """weight를 임의 값으로 램프. 0<w<1 이면 loco와 arm_sdk가 섞인다.

        0.3~0.5 부근이 "팔 자세는 대충 유지 + loco 스윙 일부 허용" 구간.
        정확한 값은 실측으로 찾아야 한다.
        """
        if not self.available:
            return {"ok": False, "reason": "arm_sdk 사용 불가"}
        w = float(np.clip(weight, 0.0, 1.0))
        with self._lock:
            self._state = STATE_RAMPING
            if w >= 0.999:
                self._stop_follow()
                try:
                    self.sync_targets()
                except Exception as e:
                    print(f"[ARM_SDK] set_weight sync 실패: {e}")
            elif w <= 0.001:
                try:
                    self.sync_targets()
                    self._start_follow()
                except Exception as e:
                    print(f"[ARM_SDK] set_weight sync 실패: {e}")
            else:
                # 부분 구간: follow를 끈다. 팔이 목표를 어느 정도 붙들어야 한다.
                self._stop_follow()
            self._ramp(self._weight, w, ramp_sec or self.ramp_sec)
            self._state = (STATE_HOLD if w >= 0.999
                           else STATE_RELEASE if w <= 0.001
                           else STATE_PARTIAL)
        print(f"[ARM_SDK] weight={w:.2f} ({self._state})")
        return self.status()

    def shutdown(self):
        """종료 시 follow 스레드만 정리한다. weight는 건드리지 않는다."""
        self._stop_follow()
