"""rclpy wrapper around :class:`PayloadCore` (design §7.4): feeds it robot states and ground-truth odometry, moves the
Gazebo package models, publishes ``/<robot>/payload_state``.

The gz-transport calls below were verified in sim in M4 and have moved the packages in every run since.
gz is imported lazily, so the node runs in the dev image with a fake ``PoseSetter``.
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
from swarmflow_interfaces.msg import Order, PayloadState, RobotState

from .pallet_core import PalletCore
from .payload_core import PayloadCore

_MODES = list(api.RobotMode)
_QOS = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)  # best-effort subscriber matches either publisher


class PoseSetter:
    """Moves Gazebo models via gz-transport (``/world/<world>/set_pose_vector``; verified in sim since M4)."""

    def __init__(self, world: str, timeout_ms: int = 300):
        self.world = world
        self.timeout_ms = timeout_ms
        self._node = None

    def set_poses(self, poses: Mapping[str, Sequence[float]]) -> bool:
        # gz-transport 13 / gz-msgs 10 Python bindings, as shipped in the sim image
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


def _pallet_spots(layout_file: str) -> Dict[str, tuple]:
    """Pallet stands of a v2 layout: model name -> (x, y, 0, yaw); empty for v1 layouts."""
    if not layout_file:
        return {}
    data = yaml.safe_load(open(layout_file, encoding="utf-8"))
    return {f"pallet_{s['name']}": (float(s["pose"]["x"]), float(s["pose"]["y"]), 0.0, float(s["pose"]["yaw"]))
            for s in data.get("stations", []) if s.get("type") == "pallet"}


PALLET_SIZE = (1.20, 1.00)


class PayloadNode(Node):
    def __init__(self, pose_setter: Optional[object] = None, core: Optional[PayloadCore] = None,
                 pallet_core: Optional[PalletCore] = None, **kwargs):
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
        if pallet_core is None:
            spots = _pallet_spots(self.get_parameter("layout_file").value)
            pool = None
            if spots:
                b = _bounds_and_offsets(self.get_parameter("layout_file").value)[0]
                pool = (b[0] - 3.0, b[1])
            pallet_core = PalletCore(spots, pool_origin=pool) if spots else None
        self.pallets = pallet_core
        self._odom: Dict[str, tuple] = {}            # robot -> latest ground-truth (x, y, yaw)
        self._last_state: Dict[str, dict] = {}       # robot -> latest {robot_id, mode, order_id}
        self.setter = pose_setter if pose_setter is not None else PoseSetter(world)
        self._lock = threading.RLock()
        self._robots = robots
        self._warned = 0
        self._pubs: Dict[str, object] = {}
        for r in robots:
            self._pubs[r] = self.create_publisher(PayloadState, f"/{r}/payload_state", 10)
            self.create_subscription(Odometry, f"/{r}/odom", lambda m, r=r: self._on_odom(r, m), _QOS)
        self.create_subscription(RobotState, "/fleet/robot_states", self._on_state, _QOS)
        if self.pallets is not None:
            self.create_subscription(Order, "/fleet/orders", self._on_order,
                                     QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE))
        self.create_timer(1.0 / float(self.get_parameter("rate_hz").value), self._on_tick)
        self.create_timer(0.2, self._publish_states)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_odom(self, robot: str, msg: Odometry) -> None:
        p = msg.pose.pose
        with self._lock:
            self.core.on_robot_pose(robot, p.position.x, p.position.y, _yaw_of(p.orientation), self._now())
            if self.pallets is not None:
                self._odom[robot] = (p.position.x, p.position.y, _yaw_of(p.orientation))
                self._feed_pallets(robot)

    def _feed_pallets(self, robot: str) -> None:
        """Give the pallet core the robot's latest mode / order together with its latest ground-truth pose."""
        st = self._last_state.get(robot)
        if st is None or robot not in self._odom:
            return
        x, y, yaw = self._odom[robot]
        self.pallets.on_robot_state(dict(st, x=x, y=y, yaw=yaw), self._now())

    def _on_order(self, msg: Order) -> None:
        otype = next((k for k, v in api.ORDER_TYPE_CODES.items() if v == msg.order_type), api.ORDER_TRANSPORT)
        with self._lock:
            self.pallets.on_order(msg.order_id, otype, msg.pallet_id)

    def _on_state(self, msg: RobotState) -> None:
        mode = _MODES[msg.mode].value if 0 <= msg.mode < len(_MODES) else "FAULT"
        with self._lock:
            st = {"robot_id": msg.robot_id, "mode": mode, "order_id": msg.order_id}
            self.core.on_robot_state(st, self._now())
            if self.pallets is not None:
                self._last_state[msg.robot_id] = st
                self._feed_pallets(msg.robot_id)

    def _on_tick(self) -> None:
        with self._lock:
            poses = self.core.tick(self._now())
            if self.pallets is not None:
                poses.update(self.pallets.tick(self._now()))
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
                pd = self.pallets.payload_state(r) if self.pallets is not None else None
            if pd is not None and pd["loaded"]:
                d = dict(pd, size_x=PALLET_SIZE[0], size_y=PALLET_SIZE[1])
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
