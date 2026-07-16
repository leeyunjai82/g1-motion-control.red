import time
import sys

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize
from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowState_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.utils.thread import RecurrentThread
from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
import subprocess
import numpy as np

G1_NUM_MOTOR = 29

Kp = [
    60, 60, 60, 100, 40, 40,      # legs
    60, 60, 60, 100, 40, 40,      # legs
    60, 40, 40,                   # waist
    40, 40, 40, 40,  40, 40, 40,  # arms
    40, 40, 40, 40,  40, 40, 40   # arms
]

Kd = [
    1, 1, 1, 2, 1, 1,     # legs
    1, 1, 1, 2, 1, 1,     # legs
    1, 1, 1,              # waist
    1, 1, 1, 1, 1, 1, 1,  # arms
    1, 1, 1, 1, 1, 1, 1   # arms
]

class G1JointIndex:
    LeftHipPitch = 0
    LeftHipRoll = 1
    LeftHipYaw = 2
    LeftKnee = 3
    LeftAnklePitch = 4
    LeftAnkleB = 4
    LeftAnkleRoll = 5
    LeftAnkleA = 5
    RightHipPitch = 6
    RightHipRoll = 7
    RightHipYaw = 8
    RightKnee = 9
    RightAnklePitch = 10
    RightAnkleB = 10
    RightAnkleRoll = 11
    RightAnkleA = 11
    WaistYaw = 12
    WaistRoll = 13        # NOTE: INVALID for g1 23dof/29dof with waist locked
    WaistA = 13           # NOTE: INVALID for g1 23dof/29dof with waist locked
    WaistPitch = 14       # NOTE: INVALID for g1 23dof/29dof with waist locked
    WaistB = 14           # NOTE: INVALID for g1 23dof/29dof with waist locked
    LeftShoulderPitch = 15
    LeftShoulderRoll = 16
    LeftShoulderYaw = 17
    LeftElbow = 18
    LeftWristRoll = 19
    LeftWristPitch = 20   # NOTE: INVALID for g1 23dof
    LeftWristYaw = 21     # NOTE: INVALID for g1 23dof
    RightShoulderPitch = 22
    RightShoulderRoll = 23
    RightShoulderYaw = 24
    RightElbow = 25
    RightWristRoll = 26
    RightWristPitch = 27  # NOTE: INVALID for g1 23dof
    RightWristYaw = 28    # NOTE: INVALID for g1 23dof


class Mode:
    PR = 0  # Series Control for Pitch/Roll Joints
    AB = 1  # Parallel Control for A/B Joints

class Custom:
    def __init__(self):
        self.time_ = 0.0
        self.control_dt_ = 0.002  # [2ms]
        self.duration_ = 3.0    # [3 s]
        self.counter_ = 0
        self.mode_pr_ = Mode.PR
        self.mode_machine_ = 0
        self.low_cmd = unitree_hg_msg_dds__LowCmd_()
        self.low_state = None
        self.update_mode_machine_ = False
        self.crc = CRC()
        # ▼▼▼ [수정] 여러 명령을 저장할 딕셔너리로 변경 ▼▼▼
        self.motion_commands = {}

    def Init(self):
        self.msc = MotionSwitcherClient()
        self.msc.SetTimeout(5.0)
        self.msc.Init()

        status, result = self.msc.CheckMode()
        while result['name']:
            self.msc.ReleaseMode()
            status, result = self.msc.CheckMode()
            time.sleep(1)

        self.lowcmd_publisher_ = ChannelPublisher("rt/lowcmd", LowCmd_)
        self.lowcmd_publisher_.Init()

        self.lowstate_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.lowstate_subscriber.Init(self.LowStateHandler, 10)

    def Start(self):
        self.lowCmdWriteThreadPtr = RecurrentThread(
            interval=self.control_dt_, target=self.LowCmdWrite, name="control"
        )
        while self.update_mode_machine_ == False:
            time.sleep(1)

        if self.update_mode_machine_ == True:
            self.lowCmdWriteThreadPtr.Start()

    def LowStateHandler(self, msg: LowState_):
        self.low_state = msg

        if self.update_mode_machine_ == False:
            self.mode_machine_ = self.low_state.mode_machine
            self.update_mode_machine_ = True

        self.counter_ +=1
        if (self.counter_ % 500 == 0) :
            self.counter_ = 0

    def _set_motor_cmd(self, motor_index, q, dq, kp, kd):
        cmd = self.low_cmd.motor_cmd[motor_index]
        cmd.mode = 1
        cmd.q = q
        cmd.dq = dq
        cmd.kp = kp
        cmd.kd = kd
        cmd.tau = 0.

    # ▼▼▼ [수정] 여러 명령을 처리하도록 로직 변경 ▼▼▼
    def command_new_move(self, motor_index, target_degree, duration):
        if self.time_ < self.duration_:
            print("아직 초기화 중입니다. 잠시 후 다시 시도해주세요.")
            return

        print(f"[{self.time_:.2f}s] 새로운 명령: {motor_index}번 모터를 {target_degree}도로 {duration}초 안에 이동")

        # 해당 모터 인덱스에 대한 새로운 지시서 생성
        self.motion_commands[motor_index] = {
            "start_q": self.low_state.motor_state[motor_index].q,
            "target_q": target_degree * np.pi / 180.0,
            "start_time": self.time_,
            "duration": duration,
        }

    # ▼▼▼ [수정] 여러 명령을 동시에 계산하도록 로직 변경 ▼▼▼
    def LowCmdWrite(self):
        self.time_ += self.control_dt_
        if self.low_state is None: return

        if self.time_ < self.duration_:
            ratio = np.clip(self.time_ / self.duration_, 0.0, 1.0)
            for i in range(G1_NUM_MOTOR):
                self._set_motor_cmd(
                    i,
                    q=(1.0 - ratio) * self.low_state.motor_state[i].q,
                    dq=0.,
                    kp=Kp[i],
                    kd=Kd[i]
                )
            self.low_cmd.mode_pr = Mode.PR
            self.low_cmd.mode_machine = self.mode_machine_
        else:
            self.low_cmd.mode_pr = Mode.PR
            self.low_cmd.mode_machine = self.mode_machine_

            q_des_map = {}

            # 1. 활성화된 모든 명령에 대해 목표 위치 계산
            for motor_idx, cmd in self.motion_commands.items():
                elapsed_time = self.time_ - cmd["start_time"]

                if elapsed_time < cmd["duration"]:
                    ratio = elapsed_time / cmd["duration"]
                    q_des = (1.0 - ratio) * cmd["start_q"] + ratio * cmd["target_q"]
                else:
                    q_des = cmd["target_q"]
                
                q_des_map[motor_idx] = q_des

            # 2. 모든 모터에 대해 최종 명령 설정
            for i in range(G1_NUM_MOTOR):
                if i in q_des_map:
                    target_q = q_des_map[i]
                else:
                    target_q = self.low_state.motor_state[i].q

                self._set_motor_cmd(i, target_q, 0., Kp[i], Kd[i])

        self.low_cmd.crc = self.crc.Crc(self.low_cmd)
        self.lowcmd_publisher_.Write(self.low_cmd)

if __name__ == '__main__':

    print("WARNING: Please ensure there are no obstacles around the robot while running this example.")
    input("Press Enter to continue...")

    if len(sys.argv)>1:
        ChannelFactoryInitialize(0, sys.argv[1])
    else:
        ChannelFactoryInitialize(0)

    custom = Custom()
    custom.Init()
    custom.Start()

    # 초기화(3초)가 끝난 후 명령 시작
    time.sleep(5) 

    for i in range(29):
        custom.command_new_move(i, 0.0, 2.0) # waist yaw

    time.sleep(2.2) 
    print("action1")
    subprocess.Popen(['./g1_audio', 's.wav'],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
    subprocess.Popen(['./g1_vui', '255', '0','0'],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)

    custom.command_new_move(15, -10.0, 2.0)
    custom.command_new_move(16, 30.0, 1.0)
    custom.command_new_move(18, 20.0, 2.0)

    custom.command_new_move(22, -10.0, 2.0)
    custom.command_new_move(23, -30.0, 2.0)
    custom.command_new_move(25, 20.0, 1.0)
    time.sleep(2.2)

    print("action2")
    custom.command_new_move(16, -10.0, 1.0)
    custom.command_new_move(23, 10.0, 1.0)

    time.sleep(2.2)

    print("action3")
    custom.command_new_move(15, -30.0, 2.0)
    custom.command_new_move(16, 0.0, 2.0)

    custom.command_new_move(22, -30.0, 2.0)
    custom.command_new_move(23, 0.0, 2.0)

    time.sleep(2.2)


    print("action4")
    custom.command_new_move(12, 80.0, 4.0)

    time.sleep(4.4)
    
    print("action5")
    custom.command_new_move(16, 20.0, 1.0)
    custom.command_new_move(23, -20.0, 1.0)

    subprocess.Popen(['./g1_audio', 'e.wav'],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
    subprocess.Popen(['./g1_vui', '0', '255','255'],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
    time.sleep(2.2)


    for i in range(29):
        custom.command_new_move(i, 0.0, 2.0) # waist yaw


    while True:
        time.sleep(1)
