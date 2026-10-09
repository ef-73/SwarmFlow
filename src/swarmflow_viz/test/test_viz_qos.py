"""swarmflow_viz QoS test (wall clock): a default-QoS (volatile) publisher on /fleet/robot_states must match the viz node.

Robot agents publish VOLATILE; the viz subscription used to request TRANSIENT_LOCAL, so DDS refused the match.
Volatile publishers only deliver to subscriptions already matched, so we wait for the match before publishing.
"""

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


def _robot_markers(ma):
    return [m for m in ma.markers if m.ns == "robots"]


def test_default_qos_robot_states_give_two_robot_markers():
    rclpy.init()
    try:
        viz = VizNode(parameter_overrides=[Parameter("layout_dir", value=str(FIX)),
                                           Parameter("use_sim_time", value=False)])
        t = rclpy.create_node("viz_qos_test")
        # Default publisher QoS: reliable, volatile, depth 10.
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        pub = t.create_publisher(RobotState, "/fleet/robot_states", qos)
        got = []
        t.create_subscription(MarkerArray, "/fleet/robot_markers", got.append, 10)
        ex = MultiThreadedExecutor(num_threads=2)
        ex.add_node(viz)
        ex.add_node(t)
        th = threading.Thread(target=ex.spin, daemon=True)
        th.start()
        states = (("robot_1", 1.0), ("robot_2", 2.0))
        deadline = time.time() + 15.0
        # Wait for the viz subscription to match our volatile publisher, then publish (republish is idempotent).
        while time.time() < deadline and pub.get_subscription_count() == 0:
            time.sleep(0.1)
        ok = False
        while time.time() < deadline and not ok:
            for rid, x in states:
                pub.publish(RobotState(robot_id=rid, x=x, y=1.0, mode=RobotState.MODE_IDLE))
            t_end = time.time() + 1.0
            while time.time() < t_end and not ok:
                ok = any(len(_robot_markers(g)) == 2 for g in got)
                time.sleep(0.1)
        ex.shutdown()
        th.join(timeout=2.0)
        assert pub.get_subscription_count() > 0
        assert any(len(_robot_markers(g)) == 2 for g in got)
        viz.destroy_node()
        t.destroy_node()
    finally:
        rclpy.try_shutdown()
