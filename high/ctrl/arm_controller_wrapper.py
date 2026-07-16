"""
G1 로봇 통합 제어 Wrapper 클래스
- 원본(.bak)의 모든 기능(양팔 IK, 단일/다중 관절 보간) 유지
- 허리(Waist) 3축 보간 제어 기능 추가
"""

import time
import threading
import numpy as np
import pinocchio as pin

import os
import sys
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(parent_dir)

from ctrl.robot_arm import G1_29_ArmController, G1_29_JointArmIndex
from ctrl.robot_arm_ik import G1_29_ArmIK

# Locomotion 관련 임포트
try:
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize # dds
    from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
    LOCO_AVAILABLE = True
except ImportError as e:
    print(f"⚠️ Locomotion 라이브러리 로드 실패: {e}")
    LOCO_AVAILABLE = False


# ==================== 관절 정보 (허리 + 팔 통합 매핑) ====================

JOINT_INFO = [
    # Waist (허리 전역 0~2)
    (14, 0, "WaistYaw"),
    (15, 1, "WaistRoll"),
    (16, 2, "WaistPitch"),
    # Arms (팔 0~13 -> 전역 15~28)
    (0,  15, "LeftShoulderPitch"),
    (1,  16, "LeftShoulderRoll"),
    (2,  17, "LeftShoulderYaw"),
    (3,  18, "LeftElbow"),
    (4,  19, "LeftWristRoll"),
    (5,  20, "LeftWristPitch"),
    (6,  21, "LeftWristYaw"),
    (7,  22, "RightShoulderPitch"),
    (8,  23, "RightShoulderRoll"),
    (9,  24, "RightShoulderYaw"),
    (10, 25, "RightElbow"),
    (11, 26, "RightWristRoll"),
    (12, 27, "RightWristPitch"),
    (13, 28, "RightWristYaw"),
]

GLOBAL_TO_INTERNAL = {info[1]: info[0] for info in JOINT_INFO}
INTERNAL_TO_GLOBAL = {info[0]: info[1] for info in JOINT_INFO}
JOINT_NAMES = [info[2] for info in (sorted(JOINT_INFO, key=lambda x: x[0]) if isinstance(JOINT_INFO[0], tuple) else [])] # 이름 리스트


# ==================== Locomotion Client ====================

class LocoClientWrapper:
    """원본과 동일한 걷기 제어 클래스"""
    def __init__(self):
        if not LOCO_AVAILABLE:
            raise RuntimeError("Locomotion library not available")
        ChannelFactoryInitialize(0)
        self.client = LocoClient()
        self.client.SetTimeout(0.0001)
        self.client.Init()
        print(dir(self.client))

    def move(self, vx, vy, vyaw):
        self.client.Move(vx, vy, vyaw, continous_move=False)

    def stop(self):
        self.client.Move(0, 0, 0, continous_move=False)

    def damp(self):
        self.client.Damp()

    def forward(self, speed=0.3): self.move(speed, 0, 0)
    def backward(self, speed=0.3): self.move(-speed, 0, 0)
    def left(self, speed=0.3): self.move(0, speed, 0)
    def right(self, speed=0.3): self.move(0, -speed, 0)
    def turn_left(self, speed=0.3): self.move(0, 0, speed)
    def turn_right(self, speed=0.3): self.move(0, 0, -speed)
    def set_height(self, height): 
        print(height)
        self.client.SetStandHeight(height)


# ==================== Arm & Waist Controller ====================

class ArmControllerWrapper:
    """G1 로봇 양팔 및 허리 통합 제어 (원본 기능 전체 포함)"""

    GROUND_TO_PELVIS = 0.782
    DEFAULT_X_RANGE = (0.1, 0.6)
    DEFAULT_Y_RANGE = (0.0, 0.4)
    DEFAULT_Z_RANGE = (-0.3, 0.5)

    def __init__(self, motion_mode=True, simulation_mode=False, visualization=False, use_motor_control=True):
        self.use_motor_control = use_motor_control
        self.visualization = visualization
        self.arm_ik = G1_29_ArmIK()

        if use_motor_control:
            self.arm_ctrl = G1_29_ArmController(motion_mode=motion_mode, simulation_mode=simulation_mode)
        else:
            self.arm_ctrl = None

        self._started = False
        self._current_q = np.zeros(14)
        self._current_dq = np.zeros(14)
        self._stop_interpolation = False
        self._interpolation_lock = threading.Lock()

    def start(self):
        if not self._started and self.arm_ctrl:
            self.arm_ctrl.speed_gradual_max()
            self._started = True

    def _update_current_state(self):
        if self.arm_ctrl:
            self._current_q = self.arm_ctrl.get_current_dual_arm_q()
            self._current_dq = self.arm_ctrl.get_current_dual_arm_dq()

    def _reset_interpolation(self):
        with self._interpolation_lock:
            self._stop_interpolation = True
        time.sleep(0.02)
        with self._interpolation_lock:
            self._stop_interpolation = False

    # -------------------- 원본 상태 조회 메서드들 --------------------

    def get_current_position(self):
        self._update_current_state()
        pin.forwardKinematics(self.arm_ik.reduced_robot.model, self.arm_ik.reduced_robot.data, self._current_q)
        pin.updateFramePlacements(self.arm_ik.reduced_robot.model, self.arm_ik.reduced_robot.data)
        left_pos = self.arm_ik.reduced_robot.data.oMf[self.arm_ik.L_hand_id].translation.copy()
        right_pos = self.arm_ik.reduced_robot.data.oMf[self.arm_ik.R_hand_id].translation.copy()
        return {'left': left_pos.tolist(), 'right': right_pos.tolist()}

    def get_current_joints_rad(self):
        self._update_current_state()
        return self._current_q.copy()

    def get_current_joints_deg(self):
        return np.degrees(self.get_current_joints_rad())

    def get_height_from_ground(self, z): return self.GROUND_TO_PELVIS + z
    def get_z_from_height(self, h): return h - self.GROUND_TO_PELVIS

    # -------------------- 허리(Waist) 제어 (신규 추가) --------------------

    def move_waist_smooth(self, yaw=0.0, roll=0.0, pitch=0.0, duration=1.5, frequency=100):
        """허리 3축을 부드럽게 동시 제어"""
        if not self.arm_ctrl: return
        if not self._started: self.start()
        self._reset_interpolation()

        with self.arm_ctrl.ctrl_lock:
            start_waist_rad = getattr(self.arm_ctrl, 'waist_q_target', np.zeros(3)).copy()

        target_waist_rad = np.radians([yaw, roll, pitch])
        steps = int(duration * frequency)
        dt = 1.0 / frequency

        for i in range(steps + 1):
            if self._stop_interpolation: return
            start_time = time.time()
            t_smooth = (i/steps)**2 * (3 - 2*(i/steps))
            interp_waist = start_waist_rad + t_smooth * (target_waist_rad - start_waist_rad)

            if hasattr(self.arm_ctrl, 'ctrl_waist'):
                self.arm_ctrl.ctrl_waist(interp_waist)
            time.sleep(max(0, dt - (time.time() - start_time)))

    # -------------------- 양팔(Arm) IK 이동 메서드들 --------------------

    def solve_ik_only(self, left_pos, right_pos, left_rot=None, right_rot=None):
        if left_rot is None: left_rot = pin.Quaternion(1, 0, 0, 0)
        if right_rot is None: right_rot = pin.Quaternion(1, 0, 0, 0)
        self._update_current_state()
        sol_q, sol_tau = self.arm_ik.solve_ik(pin.SE3(left_rot, np.array(left_pos)).homogeneous,
                                            pin.SE3(right_rot, np.array(right_pos)).homogeneous,
                                            self._current_q, self._current_dq)
        return {'joints_rad': sol_q, 'joints_deg': np.degrees(sol_q), 'tau': sol_tau}

    def solve_ik_symmetric(self, pos):
        return self.solve_ik_only(pos, [pos[0], -pos[1], pos[2]])

    def move_to(self, pos, left_rot=None, right_rot=None, duration=3.0, frequency=100):
        self.move_hands(pos, [pos[0], -pos[1], pos[2]], left_rot, right_rot, duration, frequency)

    def move_hands(self, left_pos, right_pos, left_rot=None, right_rot=None, duration=3.0, frequency=100):
        if not self.arm_ctrl: raise RuntimeError("Motor control is disabled")
        if not self._started: self.start()
        self._reset_interpolation()

        if left_rot is None: left_rot = pin.Quaternion(1, 0, 0, 0)
        if right_rot is None: right_rot = pin.Quaternion(1, 0, 0, 0)

        self._update_current_state()
        pin.forwardKinematics(self.arm_ik.reduced_robot.model, self.arm_ik.reduced_robot.data, self._current_q)
        pin.updateFramePlacements(self.arm_ik.reduced_robot.model, self.arm_ik.reduced_robot.data)

        s_L_p = self.arm_ik.reduced_robot.data.oMf[self.arm_ik.L_hand_id].translation.copy()
        s_R_p = self.arm_ik.reduced_robot.data.oMf[self.arm_ik.R_hand_id].translation.copy()
        s_L_r = pin.Quaternion(self.arm_ik.reduced_robot.data.oMf[self.arm_ik.L_hand_id].rotation)
        s_R_r = pin.Quaternion(self.arm_ik.reduced_robot.data.oMf[self.arm_ik.R_hand_id].rotation)

        target_L_p, target_R_p = np.array(left_pos), np.array(right_pos)
        steps = int(duration * frequency)
        dt = 1.0 / frequency

        for i in range(steps + 1):
            if self._stop_interpolation: return
            start_time = time.time()
            t_smooth = (i/steps)**2 * (3 - 2*(i/steps))
            
            interp_L_p = s_L_p + t_smooth * (target_L_p - s_L_p)
            interp_R_p = s_R_p + t_smooth * (target_R_p - s_R_p)
            interp_L_r = s_L_r.slerp(t_smooth, left_rot)
            interp_R_r = s_R_r.slerp(t_smooth, right_rot)

            sol_q, sol_tau = self.arm_ik.solve_ik(pin.SE3(interp_L_r, interp_L_p).homogeneous,
                                                pin.SE3(interp_R_r, interp_R_p).homogeneous,
                                                self.arm_ctrl.get_current_dual_arm_q(), self.arm_ctrl.get_current_dual_arm_dq())
            self.arm_ctrl.ctrl_dual_arm(sol_q, sol_tau)
            time.sleep(max(0, dt - (time.time() - start_time)))

    def move_to_height(self, x, y, height, duration=3.0):
        self.move_to([x, y, self.get_z_from_height(height)], duration=duration)

    def move_left_hand(self, left_pos, left_rot=None, duration=2.0, frequency=100):
        curr = self.get_current_position()
        self.move_hands(left_pos, curr['right'], left_rot, None, duration, frequency)

    def move_right_hand(self, right_pos, right_rot=None, duration=2.0, frequency=100):
        curr = self.get_current_position()
        self.move_hands(curr['left'], right_pos, None, right_rot, duration, frequency)

    # -------------------- 관절 직접 제어 (원본 기능 복구) --------------------

    def move_joint_smooth(self, motor_index, target_deg, duration=1.0, frequency=100):
        """단일 관절 제어 (허리 인덱스 지원 포함)"""
        if not self.arm_ctrl: return
        if not self._started: self.start()

        is_waist = False
        if 0 <= motor_index <= 2: # 허리 전역
            internal_idx = GLOBAL_TO_INTERNAL[motor_index] - 14
            is_waist = True
        elif 15 <= motor_index <= 28: # 팔 전역
            internal_idx = GLOBAL_TO_INTERNAL[motor_index]
        elif 0 <= motor_index <= 13: # 팔 내부
            internal_idx = motor_index
        else: raise ValueError(f"Invalid motor index: {motor_index}")

        self._reset_interpolation()

        with self.arm_ctrl.ctrl_lock:
            base_q = getattr(self.arm_ctrl, 'waist_q_target' if is_waist else 'q_target', np.zeros(14 if not is_waist else 3)).copy()

        start_rad = base_q[internal_idx]
        target_rad = np.radians(target_deg)
        steps, dt = int(duration * frequency), 1.0 / frequency

        for i in range(steps + 1):
            if self._stop_interpolation: return
            start_time = time.time()
            t_smooth = (i/steps)**2 * (3 - 2*(i/steps))
            target_q = base_q.copy()
            target_q[internal_idx] = start_rad + t_smooth * (target_rad - start_rad)

            if is_waist: self.arm_ctrl.ctrl_waist(target_q)
            else: self.arm_ctrl.ctrl_dual_arm(target_q, np.zeros(14))
            time.sleep(max(0, dt - (time.time() - start_time)))

    def move_joints_smooth(self, target_degrees, duration=1.0, frequency=100):
        """[중요] 원본의 모든 팔 관절(14개) 동시 제어 기능 복구"""
        if not self.arm_ctrl: raise RuntimeError("Motor control is disabled")
        if not self._started: self.start()
        if len(target_degrees) != 14: raise ValueError("target_degrees must have 14 elements")

        self._reset_interpolation()

        with self.arm_ctrl.ctrl_lock:
            start_rad = self.arm_ctrl.q_target.copy()

        target_rad = np.radians(target_degrees)
        steps, dt = int(duration * frequency), 1.0 / frequency

        for i in range(steps + 1):
            if self._stop_interpolation: return
            start_time = time.time()
            t_smooth = (i/steps)**2 * (3 - 2*(i/steps))
            interp_rad = start_rad + t_smooth * (target_rad - start_rad)
            self.arm_ctrl.ctrl_dual_arm(interp_rad, np.zeros(14))
            time.sleep(max(0, dt - (time.time() - start_time)))

    # -------------------- 유틸리티 --------------------

    def go_home(self):
        if self.arm_ctrl:
            self.stop_motion()
            self.arm_ctrl.ctrl_dual_arm_go_home()
            if hasattr(self.arm_ctrl, 'ctrl_waist'): self.arm_ctrl.ctrl_waist(np.zeros(3))

    def stop_motion(self):
        with self._interpolation_lock: self._stop_interpolation = True

    def get_joint_name(self, motor_index):
        if 15 <= motor_index <= 28: idx = GLOBAL_TO_INTERNAL[motor_index]
        elif 0 <= motor_index <= 13: idx = motor_index
        elif 0 <= motor_index <= 2: idx = GLOBAL_TO_INTERNAL[motor_index]
        else: return "Unknown"
        return JOINT_NAMES[idx]

    @staticmethod
    def create_quaternion_from_euler(r, p, y):
        return pin.Quaternion(pin.utils.rpyToMatrix(r, p, y))

    @staticmethod
    def validate_position(x, y, z, xr=DEFAULT_X_RANGE, yr=DEFAULT_Y_RANGE, zr=DEFAULT_Z_RANGE):
        if not (xr[0] <= x <= xr[1]): return False, f"x out of range"
        if not (yr[0] <= y <= yr[1]): return False, f"y out of range"
        if not (zr[0] <= z <= zr[1]): return False, f"z out of range"
        return True, ""

    def verify_current_position(self):
        """현재 관절각에서 실제 손 위치 확인 (FK)"""
        self._update_current_state()
        
        # FK 계산
        pin.forwardKinematics(
            self.arm_ik.reduced_robot.model,
            self.arm_ik.reduced_robot.data,
            self._current_q
        )
        pin.updateFramePlacements(
            self.arm_ik.reduced_robot.model,
            self.arm_ik.reduced_robot.data
        )
        
        # 실제 위치 추출
        left_pos = self.arm_ik.reduced_robot.data.oMf[self.arm_ik.L_hand_id].translation
        right_pos = self.arm_ik.reduced_robot.data.oMf[self.arm_ik.R_hand_id].translation
        left_rot = self.arm_ik.reduced_robot.data.oMf[self.arm_ik.L_hand_id].rotation
        right_rot = self.arm_ik.reduced_robot.data.oMf[self.arm_ik.R_hand_id].rotation
        
        return {
            'left_pos': left_pos.copy(),
            'right_pos': right_pos.copy(),
            'left_rot': left_rot.copy(),
            'right_rot': right_rot.copy(),
            'joints_rad': self._current_q.copy(),
            'joints_deg': np.degrees(self._current_q).tolist()
        }
    
    def solve_and_verify_ik(self, left_pos, right_pos, left_rot=None, right_rot=None):
        """IK 솔루션을 계산하고 FK로 즉시 검증"""
        if left_rot is None:
            left_rot = pin.Quaternion(1, 0, 0, 0)
        if right_rot is None:
            right_rot = pin.Quaternion(1, 0, 0, 0)
        
        self._update_current_state()
        
        # IK 솔루션 계산
        sol_q, sol_tau = self.arm_ik.solve_ik(
            pin.SE3(left_rot, np.array(left_pos)).homogeneous,
            pin.SE3(right_rot, np.array(right_pos)).homogeneous,
            self._current_q,
            self._current_dq
        )
        
        # FK로 검증
        pin.forwardKinematics(
            self.arm_ik.reduced_robot.model,
            self.arm_ik.reduced_robot.data,
            sol_q
        )
        pin.updateFramePlacements(
            self.arm_ik.reduced_robot.model,
            self.arm_ik.reduced_robot.data
        )
        
        actual_left = self.arm_ik.reduced_robot.data.oMf[self.arm_ik.L_hand_id].translation
        actual_right = self.arm_ik.reduced_robot.data.oMf[self.arm_ik.R_hand_id].translation
        
        error_left = np.linalg.norm(np.array(left_pos) - actual_left)
        error_right = np.linalg.norm(np.array(right_pos) - actual_right)
        
        return {
            'solution_q': sol_q.tolist(),
            'solution_q_deg': np.degrees(sol_q).tolist(),
            'solution_tau': sol_tau.tolist(),
            'target_left': left_pos,
            'target_right': right_pos,
            'actual_left': actual_left.tolist(),
            'actual_right': actual_right.tolist(),
            'error_left_mm': error_left * 1000,
            'error_right_mm': error_right * 1000,
            'is_accurate': (error_left < 0.001 and error_right < 0.001)  # 1mm 이내
        }
    

# ==================== ArmIKOnly (원본 복구) ====================

class ArmIKOnly:
    """FastAPI 서버 등에서 사용하는 경량 IK 클래스"""
    def __init__(self, visualization=False):
        self.arm_ik = G1_29_ArmIK(Unit_Test=True, Visualization=visualization)
        self._init_q = np.zeros(14)

    def solve(self, left_xyz, right_xyz, left_rpy=None, right_rpy=None):
        L_rot = self._rpy_to_quat(*(left_rpy if left_rpy else [0,0,0]))
        R_rot = self._rpy_to_quat(*(right_rpy if right_rpy else [0,0,0]))
        sol_q, sol_tau = self.arm_ik.solve_ik(pin.SE3(L_rot, np.array(left_xyz)).homogeneous,
                                            pin.SE3(R_rot, np.array(right_xyz)).homogeneous,
                                            self._init_q, np.zeros(14))
        self._init_q = sol_q.copy()
        return {'joints_deg': np.degrees(sol_q).tolist(), 'joints_rad': sol_q.tolist(), 'tau': sol_tau.tolist()}

    def solve_symmetric(self, xyz): return self.solve(xyz, [xyz[0], -xyz[1], xyz[2]])
    def _rpy_to_quat(self, r, p, y): return pin.Quaternion(pin.utils.rpyToMatrix(r, p, y))
    def reset_init_state(self): self._init_q = np.zeros(14)


# ==================== 편의 함수 (원본 복구) ====================

def parse_xyz_input(user_input):
    try:
        parts = user_input.strip().split(',')
        return [float(p.strip()) for p in parts] if len(parts) == 3 else None
    except: return None

def parse_motor_index(input_str):
    try:
        idx = int(input_str)
        if 0 <= idx <= 13: return idx
        if 15 <= idx <= 28: return GLOBAL_TO_INTERNAL[idx]
        if 0 <= idx <= 2: return GLOBAL_TO_INTERNAL[idx]
        return None
    except: return None
