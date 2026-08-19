import numpy as np
import threading
import time
from enum import IntEnum

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize # dds
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import ( LowCmd_ as hg_LowCmd, LowState_ as hg_LowState)
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.utils.crc import CRC

import logging
logging.basicConfig(level=logging.INFO)
logger_mp = logging.getLogger(__name__)

kTopicLowCommand_Debug  = "rt/lowcmd"
kTopicLowCommand_Motion = "rt/arm_sdk"
kTopicLowState = "rt/lowstate"

G1_29_Num_Motors = 35

class MotorState:
    def __init__(self):
        self.q = 0.0
        self.dq = 0.0

class G1_29_LowState:
    def __init__(self):
        self.motor_state = [MotorState() for _ in range(G1_29_Num_Motors)]

class DataBuffer:
    def __init__(self):
        self.data = None
        self.lock = threading.Lock()

    def GetData(self):
        with self.lock:
            return self.data

    def SetData(self, data):
        with self.lock:
            self.data = data

class G1_29_ArmController:
    def __init__(self, motion_mode = False, simulation_mode = False):
        logger_mp.info("G1_29_ArmController 초기화 중...")

        # 제어 타겟 초기화
        self.q_target = np.zeros(14)        # 양팔 14축
        self.tauff_target = np.zeros(14)    # 양팔 토크 피드포워드
        self.waist_q_target = np.zeros(3)   # 허리 3축 (Yaw, Roll, Pitch)

        self.motion_mode = motion_mode
        self.simulation_mode = simulation_mode

        # 게인 설정
        self.kp_high = 300.0
        self.kd_high = 3.0
        self.kp_low = 80.0
        self.kd_low = 3.0
        self.kp_wrist = 40.0
        self.kd_wrist = 1.5
        self.kp_waist = 150.0
        self.kd_waist = 3.0

        self.arm_velocity_limit = 20.0
        self.control_dt = 1.0 / 250.0 # 250Hz

        # arm_sdk 가중치 (0.0=loco 소유 / 1.0=arm_sdk 소유). motion_mode 에서만 의미.
        # FSM 501(Regular)에서는 weight=1 유지 상태로도 보행 가능(실기 검증).
        self.arm_weight = 1.0 if motion_mode else 0.0

        self._speed_gradual_max = False
        self._gradual_start_time = None

        # IMU 데이터 (subscribe 루프에서 갱신)
        self.imu_rpy   = np.zeros(3)  # [roll, pitch, yaw] rad
        self.imu_accel = np.zeros(3)  # [x, y, z] m/s²
        self.imu_gyro  = np.zeros(3)  # [x, y, z] rad/s

        # DDS 초기화
        if self.simulation_mode:
            ChannelFactoryInitialize(1)
        else:
            ChannelFactoryInitialize(0)

        # Publisher / Subscriber 설정
        if self.motion_mode:
            self.lowcmd_publisher = ChannelPublisher(kTopicLowCommand_Motion, hg_LowCmd)
        else:
            self.lowcmd_publisher = ChannelPublisher(kTopicLowCommand_Debug, hg_LowCmd)

        self.lowcmd_publisher.Init()
        self.lowstate_subscriber = ChannelSubscriber(kTopicLowState, hg_LowState)
        self.lowstate_subscriber.Init()
        self.lowstate_buffer = DataBuffer()

        # 수신 스레드 시작
        self.subscribe_thread = threading.Thread(target=self._subscribe_motor_state)
        self.subscribe_thread.daemon = True
        self.subscribe_thread.start()

        # 데이터 수신 대기
        while not self.lowstate_buffer.GetData():
            time.sleep(0.1)
            logger_mp.warning("DDS 데이터 수신 대기 중...")
        logger_mp.info("DDS 연결 성공.")

        # 메시지 객체 생성
        self.crc = CRC()
        self.msg = unitree_hg_msg_dds__LowCmd_()
        self.msg.mode_pr = 0
        self.msg.mode_machine = 0

        # 현재 상태 읽기 및 초기 타겟 설정
        current_all_q = self.get_current_motor_q()
        self.q_target = self.get_current_dual_arm_q()
        self.waist_q_target = current_all_q[12:15] # 12, 13, 14번

        logger_mp.info("모든 관절 고정 설정 중 (팔/허리 제외)...")
        arm_indices = set(member.value for member in G1_29_JointArmIndex)
        waist_indices = {12, 13, 14}

        for id in G1_29_JointIndex:
            self.msg.motor_cmd[id].mode = 1
            if id.value in arm_indices:
                if self._Is_wrist_motor(id):
                    self.msg.motor_cmd[id].kp = self.kp_wrist
                    self.msg.motor_cmd[id].kd = self.kd_wrist
                else:
                    self.msg.motor_cmd[id].kp = self.kp_low
                    self.msg.motor_cmd[id].kd = self.kd_low
            elif id.value in waist_indices:
                self.msg.motor_cmd[id].kp = self.kp_waist
                self.msg.motor_cmd[id].kd = self.kd_waist
            else:
                if self._Is_weak_motor(id):
                    self.msg.motor_cmd[id].kp = self.kp_low
                    self.msg.motor_cmd[id].kd = self.kd_low
                else:
                    self.msg.motor_cmd[id].kp = self.kp_high
                    self.msg.motor_cmd[id].kd = self.kd_high

            self.msg.motor_cmd[id].q = current_all_q[id]

        logger_mp.info("관절 고정 완료.")

        # 송신 스레드 시작
        self.publish_thread = threading.Thread(target=self._ctrl_motor_state)
        self.ctrl_lock = threading.Lock()
        self.publish_thread.daemon = True
        self.publish_thread.start()

        logger_mp.info("G1_29_ArmController 초기화 완료!")

    def _subscribe_motor_state(self):
        while True:
            msg = self.lowstate_subscriber.Read()
            if msg is not None:
                # 관절 상태 저장
                lowstate = G1_29_LowState()
                for id in range(G1_29_Num_Motors):
                    lowstate.motor_state[id].q  = msg.motor_state[id].q
                    lowstate.motor_state[id].dq = msg.motor_state[id].dq
                self.lowstate_buffer.SetData(lowstate)

                # IMU 데이터 저장 (pelvis IMU)
                self.imu_rpy   = np.array(msg.imu_state.rpy)
                self.imu_accel = np.array(msg.imu_state.accelerometer)
                self.imu_gyro  = np.array(msg.imu_state.gyroscope)

            time.sleep(0.002)

    def clip_arm_q_target(self, target_q, velocity_limit):
        current_q = self.get_current_dual_arm_q()
        delta = target_q - current_q
        motion_scale = np.max(np.abs(delta)) / (velocity_limit * self.control_dt)
        cliped_arm_q_target = current_q + delta / max(motion_scale, 1.0)
        return cliped_arm_q_target

    def _ctrl_motor_state(self):
        err_streak = 0
        while True:
            start_time = time.time()

            # weight 매 주기 반영 (hold/release 램프가 이 값을 바꾼다)
            if self.motion_mode:
                self.msg.motor_cmd[G1_29_JointIndex.kNotUsedJoint0].q = self.arm_weight

            with self.ctrl_lock:
                arm_q_target     = self.q_target
                arm_tauff_target = self.tauff_target
                waist_q_target   = self.waist_q_target

            # 1. 팔 제어 업데이트
            if self.simulation_mode:
                cliped_arm_q_target = arm_q_target
            else:
                cliped_arm_q_target = self.clip_arm_q_target(arm_q_target, velocity_limit=self.arm_velocity_limit)

            for idx, id in enumerate(G1_29_JointArmIndex):
                self.msg.motor_cmd[id].q   = cliped_arm_q_target[idx]
                self.msg.motor_cmd[id].dq  = 0
                self.msg.motor_cmd[id].tau = arm_tauff_target[idx]

            # 2. 허리 제어 업데이트 (12, 13, 14번)
            for i, joint_idx in enumerate([12, 13, 14]):
                self.msg.motor_cmd[joint_idx].q   = waist_q_target[i]
                self.msg.motor_cmd[joint_idx].dq  = 0
                self.msg.motor_cmd[joint_idx].tau = 0

            # 3. CRC 계산 및 전송 (예외 시에도 루프 유지 - 송신 중단은 낙상 위험)
            try:
                self.msg.crc = self.crc.Crc(self.msg)
                self.lowcmd_publisher.Write(self.msg)
                err_streak = 0
            except Exception as e:
                err_streak += 1
                logger_mp.error(f"arm_sdk 송신 실패({err_streak}): {e}")
                if self.motion_mode and err_streak >= 250:  # 약 1초 연속 실패
                    logger_mp.error("송신 연속 실패 - weight 비상 반납 시도")
                    self.arm_weight = max(0.0, self.arm_weight - 0.02)

            # 속도 점진적 증가 처리
            if self._speed_gradual_max:
                t_elapsed = start_time - self._gradual_start_time
                self.arm_velocity_limit = 20.0 + (10.0 * min(1.0, t_elapsed / 5.0))

            current_time = time.time()
            sleep_time = max(0, (self.control_dt - (current_time - start_time)))
            time.sleep(sleep_time)

    # ==================== 제어 메서드 ====================

    def ctrl_dual_arm(self, q_target, tauff_target):
        with self.ctrl_lock:
            self.q_target = q_target
            self.tauff_target = tauff_target

    def ctrl_waist(self, q_target):
        """허리 관절(Yaw, Roll, Pitch) 목표 각도 설정"""
        if len(q_target) != 3:
            return
        with self.ctrl_lock:
            self.waist_q_target = np.array(q_target)

    # ==================== arm_sdk 제어권 (hold/release) ====================

    def get_weight(self):
        return float(self.arm_weight)

    def sync_targets_to_current(self):
        """팔/허리 타겟을 현재 실측각으로 1회 동기화 (hold 진입 시 튐 방지).
        주의: 매 주기 추종은 금지 - 복원토크가 사라져 굽는다. 1회만."""
        all_q = self.get_current_motor_q()
        with self.ctrl_lock:
            self.q_target = self.get_current_dual_arm_q()
            self.tauff_target = np.zeros(14)
            self.waist_q_target = all_q[12:15].copy()

    def ramp_weight(self, dst, duration=2.0):
        """weight 를 현재값에서 dst 까지 duration 초 동안 선형 램프 (블로킹)."""
        dst = float(np.clip(dst, 0.0, 1.0))
        src_w = float(self.arm_weight)
        n = max(1, int(duration / 0.02))
        for i in range(1, n + 1):
            self.arm_weight = src_w + (dst - src_w) * i / n
            time.sleep(0.02)
        self.arm_weight = dst

    # ==================== 상태 조회 메서드 ====================

    def get_current_motor_q(self):
        return np.array([self.lowstate_buffer.GetData().motor_state[id].q for id in range(G1_29_Num_Motors)])

    def get_current_dual_arm_q(self):
        return np.array([self.lowstate_buffer.GetData().motor_state[id].q for id in G1_29_JointArmIndex])

    def get_current_dual_arm_dq(self):
        return np.array([self.lowstate_buffer.GetData().motor_state[id].dq for id in G1_29_JointArmIndex])

    # ==================== IMU 조회 메서드 ====================

    def get_imu_rpy(self):
        """pelvis IMU RPY [roll, pitch, yaw] (rad)"""
        return self.imu_rpy.copy()

    def get_imu_roll(self):
        """pelvis IMU roll (rad)"""
        return float(self.imu_rpy[0])

    def get_imu_pitch(self):
        """pelvis IMU pitch (rad)"""
        return float(self.imu_rpy[1])

    def get_imu_yaw(self):
        """pelvis IMU yaw (rad)"""
        return float(self.imu_rpy[2])

    def get_imu_accel(self):
        """pelvis IMU accelerometer [x, y, z] (m/s²)"""
        return self.imu_accel.copy()

    def get_imu_gyro(self):
        """pelvis IMU gyroscope [x, y, z] (rad/s)"""
        return self.imu_gyro.copy()

    def get_waist_q(self):
        """현재 허리 관절각 [yaw, roll, pitch] (rad)"""
        q = self.get_current_motor_q()
        return q[12:15].copy()

    # ==================== 유틸리티 ====================

    def ctrl_dual_arm_go_home(self):
        logger_mp.info("양팔 홈 포지션 이동 시작...")
        with self.ctrl_lock:
            self.q_target = np.zeros(14)
            self.waist_q_target = np.zeros(3)

        time.sleep(2.0)

        if self.motion_mode:
            self.ramp_weight(0.0, 1.0)
        logger_mp.info("홈 이동 완료 및 제어권 반납.")

    def speed_gradual_max(self, t=5.0):
        self._gradual_start_time = time.time()
        self._speed_gradual_max = True

    def _Is_weak_motor(self, motor_index):
        weak_motors = [4, 10, 15, 16, 17, 18, 22, 23, 24, 25]
        return motor_index.value in weak_motors

    def _Is_wrist_motor(self, motor_index):
        wrist_motors = [19, 20, 21, 26, 27, 28]
        return motor_index.value in wrist_motors


class G1_29_JointArmIndex(IntEnum):
    kLeftShoulderPitch = 15
    kLeftShoulderRoll  = 16
    kLeftShoulderYaw   = 17
    kLeftElbow         = 18
    kLeftWristRoll     = 19
    kLeftWristPitch    = 20
    kLeftWristyaw      = 21
    kRightShoulderPitch = 22
    kRightShoulderRoll  = 23
    kRightShoulderYaw   = 24
    kRightElbow         = 25
    kRightWristRoll     = 26
    kRightWristPitch    = 27
    kRightWristYaw      = 28

class G1_29_JointIndex(IntEnum):
    kLeftHipPitch   = 0
    kLeftHipRoll    = 1
    kLeftHipYaw     = 2
    kLeftKnee       = 3
    kLeftAnklePitch = 4
    kLeftAnkleRoll  = 5
    kRightHipPitch  = 6
    kRightHipRoll   = 7
    kRightHipYaw    = 8
    kRightKnee      = 9
    kRightAnklePitch = 10
    kRightAnkleRoll  = 11
    kWaistYaw        = 12
    kWaistRoll       = 13
    kWaistPitch      = 14
    kLeftShoulderPitch = 15
    kLeftShoulderRoll  = 16
    kLeftShoulderYaw   = 17
    kLeftElbow         = 18
    kLeftWristRoll     = 19
    kLeftWristPitch    = 20
    kLeftWristyaw      = 21
    kRightShoulderPitch = 22
    kRightShoulderRoll  = 23
    kRightShoulderYaw   = 24
    kRightElbow         = 25
    kRightWristRoll     = 26
    kRightWristPitch    = 27
    kRightWristYaw      = 28
    kNotUsedJoint0      = 29


