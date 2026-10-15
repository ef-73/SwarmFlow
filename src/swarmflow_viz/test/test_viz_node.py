"""swarmflow_viz node test (wall clock): 2 RobotStates -> MarkerArray with 2 robots."""

from __future__ import annotations

import pathlib
import threading
import time

import pytest

rclpy = pytest.importorskip("rclpy")
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402
from visualization_msgs.msg import MarkerArray  # noqa: E402

from swarmflow_interfaces.msg import RobotState  # noqa: E402
from swarmflow_viz.viz_node import VizNode  # noqa: E402

FIX = pathlib.Path(__file__).resolve().parents[3] / "tests" / "fixtures" / "standard"


def _two_robots(g):
    return len([m for m in g.markers if m.ns == "robot_body"]) == 2


def test_two_states_give_two_robot_markers():
    rclpy.init()
    try:
        viz = VizNode(parameter_overrides=[Parameter("layout_dir", value=str(FIX)),
                                           Parameter("use_sim_time", value=False)])
        t = rclpy.create_node("viz_test")
        qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        pub = t.create_publisher(RobotState, "/fleet/robot_states", qos)
        got, zones = [], []
        t.create_subscription(MarkerArray, "/viz/robots", got.append, 10)
        t.create_subscription(MarkerArray, "/viz/zones", zones.append, qos)
        for rid, x in (("robot_1", 1.0), ("robot_2", 2.0)):
            pub.publish(RobotState(robot_id=rid, x=x, y=1.0, mode=RobotState.MODE_IDLE))
        ex = MultiThreadedExecutor(num_threads=2)
        ex.add_node(viz)
        ex.add_node(t)
        th = threading.Thread(target=ex.spin, daemon=True)
        th.start()
        deadline = time.time() + 10.0
        while time.time() < deadline and not (any(_two_robots(g) for g in got) and zones):
            time.sleep(0.1)
        ex.shutdown()
        th.join(timeout=2.0)
        assert any(_two_robots(g) for g in got)
        assert zones and any(m.ns == "zones" for m in zones[0].markers)
        viz.destroy_node()
        t.destroy_node()
    finally:
        rclpy.try_shutdown()


def test_plan_is_read_raw_and_scene_is_cached():
    """X-01b: a published Path reaches the node's paths through the raw subscription; the scene is built once."""
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Path
    rclpy.init()
    try:
        viz = VizNode(parameter_overrides=[Parameter("layout_dir", value=str(FIX)),
                                           Parameter("use_sim_time", value=False)])
        t = rclpy.create_node("viz_plan_test")
        pub = t.create_publisher(Path, "/robot_1/plan", 10)
        msg = Path()
        msg.header.frame_id = "map"
        for x, y in ((1.0, 2.0), (1.5, 2.25), (2.0, 2.5)):
            ps = PoseStamped()
            ps.header.frame_id = "map"
            ps.pose.position.x, ps.pose.position.y = x, y
            msg.poses.append(ps)
        ex = MultiThreadedExecutor(num_threads=2)
        ex.add_node(viz)
        ex.add_node(t)
        th = threading.Thread(target=ex.spin, daemon=True)
        th.start()
        deadline = time.time() + 10.0
        while time.time() < deadline and viz._paths.get("robot_1") != [(1.0, 2.0), (1.5, 2.25), (2.0, 2.5)]:
            pub.publish(msg)
            time.sleep(0.2)
        ex.shutdown()
        th.join(timeout=2.0)
        assert viz._paths.get("robot_1") == [(1.0, 2.0), (1.5, 2.25), (2.0, 2.5)]
        first = viz._scene
        assert first is not None
        viz._pub_scene()
        assert viz._scene is first
        viz.destroy_node()
        t.destroy_node()
    finally:
        rclpy.try_shutdown()
