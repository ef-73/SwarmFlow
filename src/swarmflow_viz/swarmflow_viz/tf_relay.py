"""Merge the per-robot TF trees into one global tree for Foxglove (T021, design §7.6).

Each robot publishes ``/<robot>/tf`` and ``/<robot>/tf_static`` with unprefixed frame names (``odom``,
``base_footprint``, ``lidar_link``, …), the Nav2 multi-robot convention. Foxglove reads only ``/tf`` and
``/tf_static`` and cannot tell three ``base_link`` frames apart. This node republishes every transform with the
robot's name as a frame prefix (``robot_1/base_link``); ``map`` stays shared. It also republishes each robot's lidar
scan with the prefixed frame on ``/viz/<robot>/scan``. Viz only: Nav2 and the robot agents are untouched.
"""

from __future__ import annotations

import copy
import math
from typing import Dict, List, Sequence, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage

SHARED_FRAMES = frozenset({"map"})
#: high-rate frames the Foxglove view does not need (wheel joints): dropped to save CPU on the relay and the bridge
SKIP_CHILD_SUFFIXES = ("wheel_link",)

_STATIC_QOS = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
# Each robot has several latched static publishers (robot_state_publisher, map→odom): a depth-1 reader would keep
# only one of their messages, so the reader queue is deep.
_STATIC_SUB_QOS = QoSProfile(depth=100, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
_TF_QOS = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)


def prefixed(frame: str, robot: str) -> str:
    frame = frame.lstrip("/")
    if not frame or frame in SHARED_FRAMES or frame.startswith(robot + "/"):
        return frame
    return f"{robot}/{frame}"


def prefix_transforms(transforms: Sequence, robot: str, in_place: bool = False) -> List:
    """Prefixed copies (``in_place``: rename the given messages, cheaper for messages the caller owns)."""
    out = []
    for tr in transforms:
        t = tr if in_place else copy.deepcopy(tr)
        t.header.frame_id = prefixed(t.header.frame_id, robot)
        t.child_frame_id = prefixed(t.child_frame_id, robot)
        out.append(t)
    return out


def finite_ranges(ranges) -> List[float]:
    """Beams without a return are ``inf``; Foxglove flags those as invalid. 0.0 (below ``range_min``) = no point."""
    return [r if math.isfinite(r) else 0.0 for r in ranges]


class TfRelay(Node):
    def __init__(self, **kwargs):
        super().__init__("swarmflow_tf_relay", **kwargs)
        self.declare_parameter("robots", ["robot_1", "robot_2", "robot_3"])
        robots = [r for r in self.get_parameter("robots").value if r]
        self._tf_pub = self.create_publisher(TFMessage, "/tf", _TF_QOS)
        self._static_pub = self.create_publisher(TFMessage, "/tf_static", _STATIC_QOS)
        self._static: Dict[Tuple[str, str], object] = {}
        self._scan_pubs = {}
        for r in robots:
            self.create_subscription(TFMessage, f"/{r}/tf", lambda m, r=r: self._on_tf(r, m), _TF_QOS)
            self.create_subscription(TFMessage, f"/{r}/tf_static", lambda m, r=r: self._on_static(r, m),
                                     _STATIC_SUB_QOS)
            self._scan_pubs[r] = self.create_publisher(LaserScan, f"/viz/{r}/scan", qos_profile_sensor_data)
            self.create_subscription(LaserScan, f"/{r}/scan", lambda m, r=r: self._on_scan(r, m),
                                     qos_profile_sensor_data)

    def _on_tf(self, robot: str, msg: TFMessage) -> None:
        keep = [t for t in msg.transforms if not t.child_frame_id.endswith(SKIP_CHILD_SUFFIXES)]
        if keep:
            self._tf_pub.publish(TFMessage(transforms=prefix_transforms(keep, robot, in_place=True)))

    def _on_static(self, robot: str, msg: TFMessage) -> None:
        # /tf_static is latched with depth 1: always republish the union of every robot's static transforms
        for t in prefix_transforms(msg.transforms, robot):
            self._static[(t.header.frame_id, t.child_frame_id)] = t
        self._static_pub.publish(TFMessage(transforms=list(self._static.values())))

    def _on_scan(self, robot: str, msg: LaserScan) -> None:
        msg.header.frame_id = prefixed(msg.header.frame_id, robot)
        msg.ranges = finite_ranges(msg.ranges)
        self._scan_pubs[robot].publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TfRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
