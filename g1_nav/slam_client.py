"""
slam_client.py — Unitree SLAM 서비스(slam_operate) Python 클라이언트

공식 C++ 예제 keyDemo.cpp 를 그대로 옮긴 것입니다. payload 스키마와 토픽은
전부 그 소스에서 확인한 값이므로 추측이 없습니다.

RPC (service: slam_operate, version 1.0.0.1)
    1801 start mapping     {"data": {"slam_type": "indoor"}}
    1802 end mapping       {"data": {"address": "<pcd 저장 경로>"}}
    1804 start relocation  {"data": {x,y,z,q_x,q_y,q_z,q_w, "address": "<pcd 경로>"}}
    1102 pose navigation   {"data": {"targetPose": {...}, "mode": 1}}
    1201 pause / 1202 resume / 1901 stop node   {"data": {}}

토픽 (std_msgs/String 에 JSON)
    rt/slam_info      type=="pos_info"    -> data.currentPose  (현재 위치, 실시간)
    rt/slam_key_info  type=="task_result" -> data.is_arrived   (도착 판정)

주의: ROS/ROS2 환경을 source 하지 않은 쉘에서 실행하세요.
"""

import json
import math
import threading
import time

from unitree_sdk2py.core.channel import ChannelSubscriber
from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_
from unitree_sdk2py.rpc.client import Client

SERVICE_NAME = "slam_operate"
SERVICE_VERSION = "1.0.0.1"

TOPIC_INFO = "rt/slam_info"
TOPIC_KEY_INFO = "rt/slam_key_info"

API_START_MAPPING = 1801
API_END_MAPPING = 1802
API_START_RELOCATION = 1804
API_POSE_NAV = 1102
API_PAUSE_NAV = 1201
API_RESUME_NAV = 1202
API_STOP_NODE = 1901

ALL_APIS = (API_POSE_NAV, API_PAUSE_NAV, API_RESUME_NAV, API_STOP_NODE,
            API_START_MAPPING, API_END_MAPPING, API_START_RELOCATION)

DEFAULT_PCD = "/home/unitree/test.pcd"


# ---------------------------------------------------------------------------
# 포즈 (yaw <-> quaternion)
# ---------------------------------------------------------------------------
def yaw_to_quat(yaw):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def quat_to_yaw(q_x, q_y, q_z, q_w):
    return math.atan2(2.0 * (q_w * q_z + q_x * q_y),
                      1.0 - 2.0 * (q_y * q_y + q_z * q_z))


class Pose:
    __slots__ = ("x", "y", "z", "q_x", "q_y", "q_z", "q_w")

    def __init__(self, x=0.0, y=0.0, z=0.0, q_x=0.0, q_y=0.0, q_z=0.0, q_w=1.0):
        self.x, self.y, self.z = float(x), float(y), float(z)
        self.q_x, self.q_y = float(q_x), float(q_y)
        self.q_z, self.q_w = float(q_z), float(q_w)

    @classmethod
    def from_yaw(cls, x, y, yaw, z=0.0):
        qx, qy, qz, qw = yaw_to_quat(yaw)
        return cls(x, y, z, qx, qy, qz, qw)

    @classmethod
    def from_dict(cls, d):
        if "yaw" in d and "q_w" not in d:
            return cls.from_yaw(d["x"], d["y"], d["yaw"], d.get("z", 0.0))
        return cls(**{k: d[k] for k in cls.__slots__ if k in d})

    @property
    def yaw(self):
        return quat_to_yaw(self.q_x, self.q_y, self.q_z, self.q_w)

    def to_dict(self):
        d = {k: getattr(self, k) for k in self.__slots__}
        d["yaw"] = round(self.yaw, 4)
        return d

    def distance_to(self, other):
        return math.hypot(self.x - other.x, self.y - other.y)

    def __repr__(self):
        return f"Pose(x={self.x:.3f}, y={self.y:.3f}, yaw={math.degrees(self.yaw):.1f}deg)"


class SlamClient(Client):
    """RPC + 토픽 구독을 함께 들고 있는 클라이언트."""

    def __init__(self, timeout=10.0, pcd_path=DEFAULT_PCD):
        super().__init__(SERVICE_NAME, False)
        self.SetTimeout(timeout)
        self.pcd_path = pcd_path

        self._lock = threading.Lock()
        self.current_pose = Pose()
        self.pose_stamp = 0.0
        self.is_arrived = False
        self.last_target_node = None
        self.last_info = None      # errorCode != 0 인 메시지
        self._sub_info = None
        self._sub_key = None

    # -- 초기화 ------------------------------------------------------------
    def Init(self):
        setter = getattr(self, "_SetApiVersion", None) or getattr(self, "_SetApiVerson")
        setter(SERVICE_VERSION)
        for api_id in ALL_APIS:
            self._RegistApi(api_id, 0)

        self._sub_info = ChannelSubscriber(TOPIC_INFO, String_)
        self._sub_info.Init(self._on_info, 1)
        self._sub_key = ChannelSubscriber(TOPIC_KEY_INFO, String_)
        self._sub_key.Init(self._on_key_info, 1)

    # -- 토픽 핸들러 -------------------------------------------------------
    def _on_info(self, msg):
        try:
            d = json.loads(msg.data)
        except (json.JSONDecodeError, AttributeError):
            return
        if d.get("errorCode") != 0:
            with self._lock:
                self.last_info = d.get("info")
            return
        if d.get("type") == "pos_info":
            cp = d["data"]["currentPose"]
            with self._lock:
                self.current_pose = Pose(cp["x"], cp["y"], cp["z"],
                                         cp["q_x"], cp["q_y"], cp["q_z"], cp["q_w"])
                self.pose_stamp = time.time()

    def _on_key_info(self, msg):
        try:
            d = json.loads(msg.data)
        except (json.JSONDecodeError, AttributeError):
            return
        if d.get("errorCode") != 0:
            with self._lock:
                self.last_info = d.get("info")
            return
        if d.get("type") == "task_result":
            with self._lock:
                self.is_arrived = bool(d["data"].get("is_arrived"))
                self.last_target_node = d["data"].get("targetNodeName")

    # -- 상태 --------------------------------------------------------------
    def get_pose(self):
        with self._lock:
            return self.current_pose, self.pose_stamp

    def pose_is_fresh(self, max_age=2.0):
        with self._lock:
            return self.pose_stamp > 0 and (time.time() - self.pose_stamp) < max_age

    def clear_arrival(self):
        with self._lock:
            self.is_arrived = False
            self.last_target_node = None

    def arrival(self):
        with self._lock:
            return self.is_arrived, self.last_target_node

    # -- RPC ---------------------------------------------------------------
    def _call(self, api_id, payload_data: dict):
        parameter = json.dumps({"data": payload_data})
        code, data = self._Call(api_id, parameter)
        return code, data

    def start_mapping(self, slam_type="indoor"):
        return self._call(API_START_MAPPING, {"slam_type": slam_type})

    def end_mapping(self, address=None):
        return self._call(API_END_MAPPING, {"address": address or self.pcd_path})

    def start_relocation(self, pose: Pose, address=None):
        d = {k: getattr(pose, k) for k in Pose.__slots__}
        d["address"] = address or self.pcd_path
        return self._call(API_START_RELOCATION, d)

    def nav_to_pose(self, pose: Pose, mode=1):
        self.clear_arrival()
        return self._call(API_POSE_NAV, {
            "targetPose": {k: getattr(pose, k) for k in Pose.__slots__},
            "mode": mode,
        })

    def pause_nav(self):
        return self._call(API_PAUSE_NAV, {})

    def resume_nav(self):
        return self._call(API_RESUME_NAV, {})

    def stop_node(self):
        return self._call(API_STOP_NODE, {})

    # -- 편의 --------------------------------------------------------------
    def goto_and_wait(self, pose: Pose, timeout=180.0, poll=0.05):
        """도착 토픽이 올 때까지 대기. (성공여부, 메시지)"""
        code, data = self.nav_to_pose(pose)
        if code != 0:
            return False, f"nav_to_pose 실패 code={code} data={data!r}"
        t0 = time.time()
        while time.time() - t0 < timeout:
            arrived, node = self.arrival()
            if arrived:
                return True, f"도착 {node}"
            time.sleep(poll)
        self.pause_nav()
        return False, f"{timeout}s 초과"
