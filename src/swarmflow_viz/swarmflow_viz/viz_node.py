"""Viz node (design 11.1, T021): turns fleet topics into a Foxglove scene that matches Gazebo.

Publishes
- ``/viz/scene`` (latched): the Gazebo world read from ``generated/world.sdf``, floor, place symbols and labels;
- ``/viz/zones``: reservation zones coloured by lease state;
- ``/viz/packages``: packages at their Gazebo poses (read from gz ``/world/<w>/dynamic_pose/info`` with the
  gz-transport Python bindings of the sim image; without them packages stay at their initial poses);
- ``/viz/robots``: per robot body, footprint outline and label (locked to ``<robot>/base_footprint``, see
  ``tf_relay``), path on the floor and current goal, all in the robot's Gazebo colour;
- ``/viz/fleet_dashboard``: ``diagnostic_msgs/DiagnosticArray``, one status per robot;
- inside the Gazebo window, when one is open (``gz_overlay``): status rings, paths and goals, and the status
  board for the "SwarmFlow robots" panel;
- ``/fleet/robot_markers`` and ``/fleet/zone_markers``: the v1 marker topics, unchanged.
"""

from __future__ import annotations

import math
import os
import threading
from typing import Dict, List, Optional, Tuple

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import MarkerArray

from swarmflow_core.api import LEASE_STATE_CODES, ORDER_STATE_CODES, LeaseState
from swarmflow_core.graph import load_layout
from swarmflow_interfaces.msg import Order, OrderStatus, PayloadState, RobotState, ZoneReservation

from . import fleet_view as fv
from .gz_overlay import GzOverlay, board_lines, build_overlay
from .markers import robot_markers, station_markers, zone_markers
from .world_scene import Shape, load_world

_MODE_NAMES = {RobotState.MODE_IDLE: "IDLE", RobotState.MODE_NAVIGATING: "NAVIGATING",
               RobotState.MODE_WAITING_RESERVATION: "WAITING_RESERVATION", RobotState.MODE_LOADING: "LOADING",
               RobotState.MODE_UNLOADING: "UNLOADING", RobotState.MODE_STUCK: "STUCK",
               RobotState.MODE_FAULT: "FAULT"}
_CODE_TO_LEASE = {code: st for st, code in LEASE_STATE_CODES.items()}
_CODE_TO_ORDER_STATE = {code: st.value for st, code in ORDER_STATE_CODES.items()}
_QOS = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
_STATE_QOS = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
_STATE_QOS_100 = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
_LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
_PLACE_KINDS = ("loading", "delivery", "park", "hold")


def _yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class VizNode(Node):
    def __init__(self, **kwargs):
        super().__init__("swarmflow_viz", **kwargs)
        self.declare_parameter("layout_dir", "")
        self.declare_parameter("robots", ["robot_1", "robot_2", "robot_3"])
        self.declare_parameter("world_file", "")
        layout_dir = self.get_parameter("layout_dir").value
        self._robots: List[str] = [r for r in self.get_parameter("robots").value if r]
        self._graph = load_layout(layout_dir) if layout_dir else None
        world_file = self.get_parameter("world_file").value or (os.path.join(layout_dir, "world.sdf")
                                                                 if layout_dir else "")
        self._world = load_world(world_file) if world_file and os.path.exists(world_file) else None
        self._pkg_shapes: Dict[str, Shape] = {s.model: s for s in self._world.dynamic} if self._world else {}
        self._pkg_poses: Dict[str, Tuple[float, float, float, float]] = {
            s.model: s.pose for s in self._world.dynamic} if self._world else {}
        self._pkg_dirty = True
        self._pkg_lock = threading.Lock()
        self._gz = None

        self._states: Dict[str, dict] = {}
        self._leases: Dict[str, LeaseState] = {}
        self._orders: Dict[str, dict] = {}
        self._order_states: Dict[str, str] = {}
        self._payload: Dict[str, PayloadState] = {}
        self._paths: Dict[str, List[Tuple[float, float]]] = {}

        self.create_subscription(RobotState, "/fleet/robot_states", self._on_state, _STATE_QOS)
        self.create_subscription(ZoneReservation, "/fleet/reservations", self._on_lease, _QOS)
        self.create_subscription(Order, "/fleet/orders", self._on_order, _STATE_QOS_100)
        self.create_subscription(OrderStatus, "/fleet/order_status", self._on_order_status, _QOS)
        if self._world is not None and self._pkg_shapes:
            self._subscribe_gz(f"/world/{self._world.name}/dynamic_pose/info")
        for r in self._robots:
            self.create_subscription(PayloadState, f"/{r}/payload_state", self._on_payload, 10)
            self.create_subscription(Path, f"/{r}/plan", lambda m, r=r: self._on_plan(r, m), 10)

        self._robot_pub = self.create_publisher(MarkerArray, "/fleet/robot_markers", 10)
        self._zone_pub = self.create_publisher(MarkerArray, "/fleet/zone_markers", 10)
        self._scene_pub = self.create_publisher(MarkerArray, "/viz/scene", _LATCHED)
        self._zones_pub = self.create_publisher(MarkerArray, "/viz/zones", _LATCHED)
        self._pkg_pub = self.create_publisher(MarkerArray, "/viz/packages", 10)
        self._robots_pub = self.create_publisher(MarkerArray, "/viz/robots", 10)
        self._dash_pub = self.create_publisher(DiagnosticArray, "/viz/fleet_dashboard", 10)
        self.create_timer(0.2, self._pub_robots)
        self.create_timer(1.0, self._pub_zones)
        self.create_timer(0.5, self._pub_fleet)      # robot shapes are frame-locked: TF moves them smoothly
        self.create_timer(0.1, self._pub_packages)
        self.create_timer(0.5, self._pub_dashboard)
        self.create_timer(10.0, self._pub_scene)
        self._overlay = GzOverlay(logger=self.get_logger())
        if not self._overlay.start():
            self._overlay = None
        self._pub_scene()

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # ---- inputs -------------------------------------------------------------------------------------------------
    def _on_state(self, msg: RobotState) -> None:
        self._states[msg.robot_id] = {
            "robot_id": msg.robot_id, "x": msg.x, "y": msg.y, "yaw": msg.yaw,
            "mode": _MODE_NAMES.get(msg.mode, str(msg.mode)), "order_id": msg.order_id, "task_id": msg.task_id,
            "next_vertex": msg.next_vertex, "remaining_route": tuple(msg.remaining_route),
            "speed": msg.linear_speed, "leases": tuple(msg.held_lease_ids), "fault_reason": msg.fault_reason,
            "stamp": msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9}

    def _on_lease(self, msg: ZoneReservation) -> None:
        st = _CODE_TO_LEASE.get(msg.state)
        if st is not None:
            self._leases[msg.zone_id] = st

    def _on_order(self, msg: Order) -> None:
        self._orders[msg.order_id] = {"pickup_vertex": msg.pickup_vertex, "dropoff_vertex": msg.dropoff_vertex}

    def _on_order_status(self, msg: OrderStatus) -> None:
        self._order_states[msg.order_id] = _CODE_TO_ORDER_STATE.get(msg.state, str(msg.state))

    def _on_payload(self, msg: PayloadState) -> None:
        self._payload[msg.robot_id] = msg

    def _on_plan(self, robot: str, msg: Path) -> None:
        self._paths[robot] = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]

    def _subscribe_gz(self, topic: str) -> None:
        try:
            from gz.msgs10.pose_v_pb2 import Pose_V
            from gz.transport13 import Node as GzNode, SubscribeOptions
        except ImportError:
            self.get_logger().warn("gz-transport Python bindings not found: packages are drawn at their start poses")
            return
        self._gz = GzNode()
        opts = SubscribeOptions()
        opts.msgs_per_sec = 10          # Gazebo sends ~50 Hz; packages need far less (saves Python decoding)
        if not self._gz.subscribe(Pose_V, topic, self._on_gz_poses, opts):
            self.get_logger().warn(f"could not subscribe to gz topic {topic}")

    def _on_gz_poses(self, msg) -> None:          # gz-transport thread
        with self._pkg_lock:
            for p in msg.pose:
                if p.name in self._pkg_shapes:
                    pose = (p.position.x, p.position.y, p.position.z, _yaw(p.orientation))
                    if self._pkg_poses.get(p.name) != pose:
                        self._pkg_poses[p.name] = pose
                        self._pkg_dirty = True

    # ---- outputs ------------------------------------------------------------------------------------------------
    def _places(self):
        if self._graph is None:
            return []
        return [(n, v.kind, v.x, v.y) for n, v in self._graph.vertices.items() if v.kind in _PLACE_KINDS]

    def _bounds(self) -> Optional[Tuple[float, float, float, float]]:
        if self._world is None:
            return None
        walls = [s for s in self._world.static if s.model.startswith("wall_")]
        if not walls:
            return None
        xs = [s.pose[0] for s in walls]
        ys = [s.pose[1] for s in walls]
        return (min(xs), min(ys), max(xs), max(ys))

    def _pub_scene(self) -> None:
        shapes = self._world.static if self._world else ()
        self._scene_pub.publish(fv.scene_markers(shapes, self._bounds(), self._places(), self._now()))

    def _pub_packages(self) -> None:
        with self._pkg_lock:
            if not (self._pkg_dirty and self._pkg_shapes):
                return
            self._pkg_dirty = False
            poses = dict(self._pkg_poses)
        self._pkg_pub.publish(fv.package_markers(poses, self._pkg_shapes, self._now()))

    def _goal(self, rid: str, s: dict):
        oid = s.get("order_id", "")
        kinds = {n: v.kind for n, v in self._graph.vertices.items()} if self._graph is not None else {}
        g = fv.current_goal(s, self._orders.get(oid), self._order_states.get(oid, ""), kinds)
        if g is None or self._graph is None or g[0] not in self._graph.vertices:
            return None, ""
        v = self._graph.vertices[g[0]]
        return (v.x, v.y), f"{g[0]} {g[1]}"

    def _pub_fleet(self) -> None:
        t = self._now()
        ma = MarkerArray()
        for idx, rid in enumerate(self._robots):
            s = self._states.get(rid)
            if s is None:
                continue
            p = self._payload.get(rid)
            fp = [(q.x, q.y) for q in p.footprint.points] if p is not None and p.loaded else []
            ma.markers.extend(fv.robot_markers(rid, idx, t, s["mode"], fp or fv.DEFAULT_FOOTPRINT))
            active = bool(s.get("task_id"))
            ma.markers.append(fv.path_marker(rid, idx, t, self._paths.get(rid, []) if active else []))
            goal, label = self._goal(rid, s) if active else (None, "")
            ma.markers.extend(fv.goal_markers(rid, idx, t, goal, label))
        self._robots_pub.publish(ma)

    def _pub_dashboard(self) -> None:
        t = self._now()
        rows = {}
        for rid, s in self._states.items():
            oid = s.get("order_id", "")
            p = self._payload.get(rid)
            goal = self._goal(rid, s)[1] if s.get("task_id") else ""
            rows[rid] = dict(s, order_state=self._order_states.get(oid, ""), goal=goal,
                             loaded=bool(p is not None and p.loaded))
        dash = fv.dashboard(t, rows, self._robots)
        self._dash_pub.publish(dash)
        if self._overlay is not None:
            goals = {rid: self._goal(rid, s)[0] for rid, s in self._states.items() if s.get("task_id")}
            levels = {st.name: fv.level_of(st) for st in dash.status}
            board = board_lines([f"{st.name}: {st.message}" for st in dash.status])
            self._overlay.update(build_overlay(self._robots, rows, levels, self._paths, goals), board)

    def destroy_node(self):
        if self._overlay is not None:
            self._overlay.stop()
        return super().destroy_node()

    def _pub_robots(self) -> None:
        self._robot_pub.publish(robot_markers(list(self._states.values()), self._now()))

    def _pub_zones(self) -> None:
        if self._graph is None:
            return
        now = self._now()
        zones = zone_markers(self._graph, self._leases, now)
        self._zones_pub.publish(zones)
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
