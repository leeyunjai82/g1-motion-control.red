import time
import sys
import numpy as np

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowState_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.utils.thread import RecurrentThread
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient

Kp = [
    60, 60, 60, 100, 40, 40,
    60, 60, 60, 100, 40, 40,
    60, 100, 100,
    40, 40, 40, 40, 40, 40, 40,
    40, 40, 40, 40, 40, 40, 40,
]

Kd = [
    1, 1, 1, 2, 1, 1,
    1, 1, 1, 2, 1, 1,
    1, 1, 1,
    1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1,
]

LOCO_DIRECTION_MAP = {
    "forward":    ( 1.0,  0.0,  0.0),
    "backward":   (-1.0,  0.0,  0.0),
    "left":       ( 0.0,  1.0,  0.0),
    "right":      ( 0.0, -1.0,  0.0),
    "turn_left":  ( 0.0,  0.0,  1.0),
    "turn_right": ( 0.0,  0.0, -1.0),
    "stop":       ( 0.0,  0.0,  0.0),
}

LOCO_LINEAR_SPEED  = 0.3  # m/s
LOCO_ANGULAR_SPEED = 0.4  # rad/s


class G1JointIndex:
    LeftHipPitch, LeftHipRoll, LeftHipYaw, LeftKnee, LeftAnklePitch, LeftAnkleRoll = 0, 1, 2, 3, 4, 5
    RightHipPitch, RightHipRoll, RightHipYaw, RightKnee, RightAnklePitch, RightAnkleRoll = 6, 7, 8, 9, 10, 11
    WaistYaw, WaistRoll, WaistPitch = 12, 13, 14
    LeftShoulderPitch, LeftShoulderRoll, LeftShoulderYaw, LeftElbow = 15, 16, 17, 18
    LeftWristRoll, LeftWristPitch, LeftWristYaw = 19, 20, 21
    RightShoulderPitch, RightShoulderRoll, RightShoulderYaw, RightElbow = 22, 23, 24, 25
    RightWristRoll, RightWristPitch, RightWristYaw = 26, 27, 28
    kNotUsedJoint = 29


class Custom:
    def __init__(self, interface=None):
        if interface:
            ChannelFactoryInitialize(0, interface)
        else:
            ChannelFactoryInitialize(0)

        self.time_ = 0.0
        self.control_dt_ = 0.02
        self.duration_ = 3.0
        self.low_cmd = unitree_hg_msg_dds__LowCmd_()
        self.low_state = None
        self.first_update_low_state = False
        self.crc = CRC()
        self.motion_commands = {}

        self.loco_client = LocoClient()
        self.loco_client.SetTimeout(10.0)
        self.loco_client.Init()

        self.arm_joints = [
            G1JointIndex.WaistYaw, G1JointIndex.WaistRoll, G1JointIndex.WaistPitch,
            G1JointIndex.LeftShoulderPitch, G1JointIndex.LeftShoulderRoll, G1JointIndex.LeftShoulderYaw, G1JointIndex.LeftElbow,
            G1JointIndex.LeftWristRoll, G1JointIndex.LeftWristPitch, G1JointIndex.LeftWristYaw,
            G1JointIndex.RightShoulderPitch, G1JointIndex.RightShoulderRoll, G1JointIndex.RightShoulderYaw, G1JointIndex.RightElbow,
            G1JointIndex.RightWristRoll, G1JointIndex.RightWristPitch, G1JointIndex.RightWristYaw,
        ]

    def Init(self):
        self.arm_sdk_publisher = ChannelPublisher("rt/arm_sdk", LowCmd_)
        self.arm_sdk_publisher.Init()
        self.lowstate_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.lowstate_subscriber.Init(self.LowStateHandler, 10)

    def Start(self):
        self.lowCmdWriteThreadPtr = RecurrentThread(
            interval=self.control_dt_, target=self.LowCmdWrite, name="control"
        )
        while not self.first_update_low_state:
            print("Waiting for robot state...")
            time.sleep(1)
        self.lowCmdWriteThreadPtr.Start()

    def LowStateHandler(self, msg: LowState_):
        self.low_state = msg
        if not self.first_update_low_state:
            self.first_update_low_state = True

    def _set_motor_cmd(self, motor_index, q, dq, kp, kd):
        cmd = self.low_cmd.motor_cmd[motor_index]
        cmd.mode = 1
        cmd.q = q
        cmd.dq = dq
        cmd.kp = kp
        cmd.kd = kd
        cmd.tau = 0.0

    def command_new_move(self, motor_index, target_degree, duration=None):
        if motor_index not in self.arm_joints:
            print(f"경고: 모터 {motor_index}는 arm_sdk로 제어할 수 없습니다.")
            return
        if self.time_ < self.duration_:
            print("아직 초기화 중입니다. 잠시 후 다시 시도해주세요.")
            return

        target_q = target_degree * np.pi / 180.0
        current_q = self.low_state.motor_state[motor_index].q

        if duration is None or duration <= 0:
            duration = abs(target_q - current_q) * 2
            duration = max(duration, 0.1)

        self.motion_commands[motor_index] = {
            "start_q": current_q,
            "target_q": target_q,
            "start_time": self.time_,
            "duration": duration,
        }

    def execute_loco_command(self, command_name: str, *args, **kwargs):
        try:
            if hasattr(self.loco_client, command_name):
                getattr(self.loco_client, command_name)(*args, **kwargs)
            else:
                print(f"에러: '{command_name}'은(는) 유효한 로코모션 명령이 아닙니다.")
        except Exception as e:
            print(f"로코모션 명령 '{command_name}' 실행 중 에러: {e}")

    def LowCmdWrite(self):
        self.time_ += self.control_dt_
        if self.low_state is None:
            return

        self.low_cmd.motor_cmd[G1JointIndex.kNotUsedJoint].q = 1.0

        if self.time_ < self.duration_:
            ratio = np.clip(self.time_ / self.duration_, 0.0, 1.0)
            for i in self.arm_joints:
                initial_q = self.low_state.motor_state[i].q if self.first_update_low_state else 0.0
                self._set_motor_cmd(i, (1.0 - ratio) * initial_q, 0., Kp[i], Kd[i])
        else:
            q_des_map = {}
            for motor_idx, cmd in self.motion_commands.items():
                elapsed = self.time_ - cmd["start_time"]
                if elapsed < cmd["duration"]:
                    ratio = elapsed / cmd["duration"]
                    q_des_map[motor_idx] = (1.0 - ratio) * cmd["start_q"] + ratio * cmd["target_q"]
                else:
                    q_des_map[motor_idx] = cmd["target_q"]

            for i in self.arm_joints:
                if i in q_des_map:
                    target_q = q_des_map[i]
                elif i in self.motion_commands:
                    target_q = self.motion_commands[i]["target_q"]
                else:
                    target_q = self.low_state.motor_state[i].q
                self._set_motor_cmd(i, target_q, 0., Kp[i], Kd[i])

        self.low_cmd.crc = self.crc.Crc(self.low_cmd)
        self.arm_sdk_publisher.Write(self.low_cmd)

    def set_motion(self, motion_data):
        print(f"모션 시퀀스 시작 ({len(motion_data)}개 동작)")

        for i, action in enumerate(motion_data):
            duration = float(action.get("duration", 1.0))
            print(f"[{i+1}/{len(motion_data)}] duration={duration}s")

            if "pose" in action and "targets" in action["pose"]:
                for target in action["pose"]["targets"]:
                    motor_id = target.get("motor_index")
                    target_deg = float(target.get("target_degree", 0.0))
                    if motor_id in self.arm_joints:
                        self.command_new_move(motor_id, target_deg, duration)
                    else:
                        print(f"  ⚠️ 모터 ID {motor_id} 매핑 없음, 건너뜀")
                time.sleep(duration + 0.1)

            elif "locomotion" in action:
                direction = action["locomotion"].get("direction", "stop")
                if direction in LOCO_DIRECTION_MAP:
                    vx, vy, vyaw = LOCO_DIRECTION_MAP[direction]
                    print(f"  이동: '{direction}' {duration}s")
                    self.execute_loco_command("Move", vx, vy, vyaw)
                    time.sleep(duration)
                    self.execute_loco_command("Move", 0.0, 0.0, 0.0)
                    time.sleep(0.5)
                else:
                    print(f"  ⚠️ 알 수 없는 방향: '{direction}'")

        print("모션 시퀀스 완료")
