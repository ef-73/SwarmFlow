"""rclpy wrapper around :class:`PayloadCore` (design §7.4): feeds it robot states and ground-truth odometry, moves the
Gazebo package models, publishes ``/<robot>/payload_state``.

The gz-transport calls are marked **[U]** (names taken from the installed gz Python modules, not yet exercised);
the lead verifies them in sim. gz is imported lazily, so the node runs in the dev image with a fake ``PoseSetter``.
"""

from __future__ import annotations

import math
import threading
from typing import Dict, Mapping, Optional, Sequence

import rclpy
import yaml
from geometry_msgs.msg import Point32
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from swarmflow_core import api
from swarmflow_core.graph import load_layout
from swarmflow_interfaces.msg import PayloadState, RobotState

from .payload_core import PayloadCore

_MODES = list(api.RobotMode)
_QOS = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)  # best-effort subscriber matches either publisher


class PoseSetter:
    """Moves Gazebo models via gz-transport. [U] Everything in here is unverified against a running sim."""

    def __init__(self, world: str, timeout_ms: int = 300):
        self.world = world
        self.timeout_ms = timeout_ms
        self._node = None

    def set_poses(self, poses: Mapping[str, Sequence[float]]) -> bool:
        # [U] gz-transport 13 / gz-msgs 10 Python bindings, as shipped in the sim image
        from gz.msgs10.boolean_pb2 import Boolean
        from gz.msgs10.pose_v_pb2 import Pose_V
        from gz.transport13 import Node as GzNode

        if self._node is None:
            self._node = GzNode()
        req = Pose_V()
        for name, (x, y, z, yaw) in poses.items():
            p = req.pose.add()
            p.name = name
            p.position.x, p.position.y, p.position.z = float(x), float(y), float(z)
            p.orientation.x = 0.0
            p.orientation.y = 0.0
            p.orientation.z = math.sin(yaw / 2.0)
            p.orientation.w = math.cos(yaw / 2.0)
        # [U] service name and request/response types
        result, rep = self._node.request(f"/world/{self.world}/set_pose_vector", req, Pose_V, Boolean, self.timeout_ms)
        return bool(result) and bool(getattr(rep, "data", False))


def _yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _bounds_and_offsets(layout_file: str):
    data = yaml.safe_load(open(layout_file, encoding="utf-8"))
    b = data["bounds"]
    bounds = (float(b["x_min"]), float(b["y_min"]), float(b["x_max"]), float(b["y_max"]))
    offsets = {s["name"]: float(s["drop_offset_m"]) for s in data.get("stations", []) if "drop_offset_m" in s}
    return bounds, offsets


class PayloadNode(Node):
    def __init__(self, pose_setter: Optional[object] = None, core: Optional[PayloadCore] = None, **kwargs):
        super().__init__("payload", **kwargs)
        self.declare_parameter("layout_dir", "")
        self.declare_parameter("layout_file", "")
        self.declare_parameter("world", "standard")
        self.declare_parameter("robots", ["robot_1", "robot_2", "robot_3"])
        self.declare_parameter("rate_hz", 20.0)
        robots = list(self.get_parameter("robots").value)
        world = self.get_parameter("world").value
        if core is None:
            bounds, offsets = _bounds_and_offsets(self.get_parameter("layout_file").value)
            core = PayloadCore(load_layout(self.get_parameter("layout_dir").value), bounds, offsets)
        self.core = core
        self.setter = pose_setter if pose_setter is not None else PoseSetter(world)
        self._lock = threading.RLock()
        self._robots = robots
        self._warned = 0
        self._pubs: Dict[str, object] = {}
        for r in robots:
            self._pubs[r] = self.create_publisher(PayloadState, f"/{r}/payload_state", 10)
            self.create_subscription(Odometry, f"/{r}/odom", lambda m, r=r: self._on_odom(r, m), _QOS)
        self.create_subscription(RobotState, "/fleet/robot_states", self._on_state, _QOS)
        self.create_timer(1.0 / float(self.get_parameter("rate_hz").value), self._on_tick)
        self.create_timer(0.2, self._publish_states)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_odom(self, robot: str, msg: Odometry) -> None:
        p = msg.pose.pose
        with self._lock:
            self.core.on_robot_pose(robot, p.position.x, p.position.y, _yaw_of(p.orientation), self._now())

    def _on_state(self, msg: RobotState) -> None:
        mode = _MODES[msg.mode].value if 0 <= msg.mode < len(_MODES) else "FAULT"
        with self._lock:
            self.core.on_robot_state({"robot_id": msg.robot_id, "mode": mode, "order_id": msg.order_id}, self._now())

    def _on_tick(self) -> None:
        with self._lock:
            poses = self.core.tick(self._now())
            warnings = list(self.core.warnings)
        for w in warnings[self._warned:]:
            self.get_logger().warning(w)
        self._warned = len(warnings)
        if not poses:
            return
        try:
            ok = self.setter.set_poses(poses)
        except Exception as exc:  # noqa: BLE001 - a missing/failed gz call must not kill the node
            self.get_logger().error(f"set_pose_vector failed: {exc}", throttle_duration_sec=5.0)
            return
        if not ok:
            self.get_logger().warning("set_pose_vector returned failure", throttle_duration_sec=5.0)

    def _publish_states(self) -> None:
        stamp = self.get_clock().now().to_msg()
        for r in self._robots:
            with self._lock:
                d = self.core.payload_state(r)
            m = PayloadState()
            m.header.stamp = stamp
            m.header.frame_id = "base_footprint"
            m.robot_id = r
            m.loaded = d["loaded"]
            m.order_id = d["order_id"]
            m.payload_type = d["payload_type"]
            m.size_x = d["size_x"]
            m.size_y = d["size_y"]
            m.footprint.points = [Point32(x=float(x), y=float(y), z=0.0) for x, y in d["footprint"]]
            self._pubs[r].publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = PayloadNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
