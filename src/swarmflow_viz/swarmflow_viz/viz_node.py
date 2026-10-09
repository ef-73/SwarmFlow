"""Marker publisher node (design 11.1): /fleet/robot_states + /fleet/reservations -> MarkerArrays for Foxglove."""

from __future__ import annotations

from typing import Dict

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import MarkerArray

from swarmflow_core.api import LEASE_STATE_CODES, LeaseState
from swarmflow_core.graph import load_layout
from swarmflow_interfaces.msg import RobotState, ZoneReservation

from .markers import robot_markers, station_markers, zone_markers

_MODE_NAMES = {RobotState.MODE_IDLE: "IDLE", RobotState.MODE_NAVIGATING: "NAVIGATING",
               RobotState.MODE_WAITING_RESERVATION: "WAITING_RESERVATION", RobotState.MODE_LOADING: "LOADING",
               RobotState.MODE_UNLOADING: "UNLOADING", RobotState.MODE_STUCK: "STUCK",
               RobotState.MODE_FAULT: "FAULT"}
_CODE_TO_LEASE = {code: st for st, code in LEASE_STATE_CODES.items()}
_QOS = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
_STATE_QOS = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)


class VizNode(Node):
    def __init__(self, **kwargs):
        super().__init__("swarmflow_viz", **kwargs)
        self.declare_parameter("layout_dir", "")
        layout_dir = self.get_parameter("layout_dir").value
        self._graph = load_layout(layout_dir) if layout_dir else None
        self._states: Dict[str, dict] = {}
        self._leases: Dict[str, LeaseState] = {}
        self.create_subscription(RobotState, "/fleet/robot_states", self._on_state, _STATE_QOS)
        self.create_subscription(ZoneReservation, "/fleet/reservations", self._on_lease, _QOS)
        self._robot_pub = self.create_publisher(MarkerArray, "/fleet/robot_markers", 10)
        self._zone_pub = self.create_publisher(MarkerArray, "/fleet/zone_markers", 10)
        self.create_timer(0.2, self._pub_robots)
        self.create_timer(1.0, self._pub_zones)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_state(self, msg: RobotState) -> None:
        self._states[msg.robot_id] = {
            "robot_id": msg.robot_id, "x": msg.x, "y": msg.y, "yaw": msg.yaw,
            "mode": _MODE_NAMES.get(msg.mode, str(msg.mode)), "order_id": msg.order_id}

    def _on_lease(self, msg: ZoneReservation) -> None:
        st = _CODE_TO_LEASE.get(msg.state)
        if st is not None:
            self._leases[msg.zone_id] = st

    def _pub_robots(self) -> None:
        self._robot_pub.publish(robot_markers(list(self._states.values()), self._now()))

    def _pub_zones(self) -> None:
        if self._graph is None:
            return
        now = self._now()
        ma = zone_markers(self._graph, self._leases, now)
        ma.markers.extend(station_markers(self._graph, now).markers)
        self._zone_pub.publish(ma)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VizNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
