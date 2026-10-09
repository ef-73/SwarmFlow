"""PayloadNode with a fake PoseSetter (wall clock, no Gazebo)."""

from __future__ import annotations

import pathlib
import time

import pytest

rclpy = pytest.importorskip("rclpy")
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402

from swarmflow_interfaces.msg import PayloadState, RobotState  # noqa: E402
from swarmflow_payload.payload_node import PayloadNode  # noqa: E402

ROOT = pathlib.Path(__file__).resolve().parents[3]
LAYOUT = ROOT / "layouts" / "standard"


class FakeSetter:
    def __init__(self):
        self.calls = []

    def set_poses(self, poses):
        self.calls.append(dict(poses))
        return True


def test_loading_robot_gets_package_and_payload_state():
    rclpy.init()
    try:
        fake = FakeSetter()
        node = PayloadNode(pose_setter=fake, parameter_overrides=[
            Parameter("layout_dir", value=str(LAYOUT / "generated")),
            Parameter("layout_file", value=str(LAYOUT / "layout.yaml")),
            Parameter("robots", value=["robot_1"])])
        helper = rclpy.create_node("payload_test_helper")
        got = []
        helper.create_subscription(PayloadState, "/robot_1/payload_state", got.append, 10)
        odom_pub = helper.create_publisher(Odometry, "/robot_1/odom", 10)
        st_pub = helper.create_publisher(RobotState, "/fleet/robot_states", 10)
        ex = SingleThreadedExecutor()
        ex.add_node(node)
        ex.add_node(helper)
        deadline = time.time() + 10.0
        while time.time() < deadline:
            od = Odometry()
            od.pose.pose.position.x, od.pose.pose.position.y, od.pose.pose.orientation.w = 1.3, 3.85, 1.0
            odom_pub.publish(od)
            st = RobotState()
            st.robot_id, st.mode, st.order_id = "robot_1", RobotState.MODE_LOADING, "o1"
            st_pub.publish(st)
            ex.spin_once(timeout_sec=0.05)
            if any(m.loaded for m in got) and any("pkg_00" in c for c in fake.calls):
                break
        assert any(m.loaded and m.order_id == "o1" and m.payload_type == "small" for m in got)
        assert len(got[-1].footprint.points) == 4
        pose = [c["pkg_00"] for c in fake.calls if "pkg_00" in c][-1]
        assert pose[:3] == pytest.approx((1.3, 3.85, 0.45))
        node.destroy_node()
        helper.destroy_node()
    finally:
        rclpy.shutdown()
