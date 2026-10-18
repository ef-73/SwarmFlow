"""T054: C++ TF / scan relay (X-01) behaves like the Python tf_relay it replaces.
Starts the installed `tf_relay_cpp` executable for robot_1 and checks /tf, /tf_static and /viz/robot_1/scan."""

from __future__ import annotations

import math
import os
import subprocess
import threading
import time

import pytest

rclpy = pytest.importorskip("rclpy")
from ament_index_python.packages import get_package_prefix  # noqa: E402
from geometry_msgs.msg import TransformStamped  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data  # noqa: E402
from sensor_msgs.msg import LaserScan  # noqa: E402
from tf2_msgs.msg import TFMessage  # noqa: E402

STATIC = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
RELIABLE = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)


def tf(parent, child, x=0.0):
    t = TransformStamped()
    t.header.frame_id, t.child_frame_id = parent, child
    t.transform.translation.x = x
    t.transform.rotation.w = 1.0
    return t


def wait(pred, timeout=10.0):
    end = time.time() + timeout
    while time.time() < end:
        if pred():
            return True
        time.sleep(0.05)
    return False


def test_cpp_relay_prefixes_frames_filters_wheels_and_fixes_scans():
    exe = os.path.join(get_package_prefix("swarmflow_viz"), "lib", "swarmflow_viz", "tf_relay_cpp")
    assert os.path.isfile(exe), exe
    env = dict(os.environ, ROS_DOMAIN_ID="83")
    os.environ["ROS_DOMAIN_ID"] = "83"
    proc = subprocess.Popen([exe, "--ros-args", "-p", "robots:=[robot_1]", "-p", "use_sim_time:=false"], env=env,
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    rclpy.init()
    try:
        n = rclpy.create_node("relay_test")
        got_tf, got_static, got_scan = [], [], []
        n.create_subscription(TFMessage, "/tf", got_tf.append, RELIABLE)
        n.create_subscription(TFMessage, "/tf_static", got_static.append, STATIC)
        n.create_subscription(LaserScan, "/viz/robot_1/scan", got_scan.append, qos_profile_sensor_data)
        pub_tf = n.create_publisher(TFMessage, "/robot_1/tf", RELIABLE)
        pub_static = n.create_publisher(TFMessage, "/robot_1/tf_static", STATIC)
        pub_scan = n.create_publisher(LaserScan, "/robot_1/scan", qos_profile_sensor_data)
        ex = SingleThreadedExecutor()
        ex.add_node(n)
        threading.Thread(target=ex.spin, daemon=True).start()
        assert wait(lambda: pub_tf.get_subscription_count() > 0 and pub_scan.get_subscription_count() > 0, 15.0)
        pub_static.publish(TFMessage(transforms=[tf("map", "odom"), tf("base_link", "lidar_link", 0.2)]))
        scan = LaserScan(range_min=0.12, range_max=8.0, angle_increment=math.pi / 2, ranges=[1.0, math.inf, 2.0])
        scan.header.frame_id = "lidar_link"
        for _ in range(20):
            pub_tf.publish(TFMessage(transforms=[tf("odom", "base_footprint", 1.5), tf("base_link", "left_wheel_link")]))
            pub_scan.publish(scan)
            time.sleep(0.1)
        assert wait(lambda: got_tf and got_scan and got_static, 10.0)
        frames = {(t.header.frame_id, t.child_frame_id) for m in got_tf for t in m.transforms}
        assert ("robot_1/odom", "robot_1/base_footprint") in frames
        assert not any("wheel" in c for _, c in frames)
        x = [t.transform.translation.x for m in got_tf for t in m.transforms if t.child_frame_id == "robot_1/base_footprint"]
        assert x and x[0] == pytest.approx(1.5)
        statics = {(t.header.frame_id, t.child_frame_id) for t in got_static[-1].transforms}
        assert statics == {("map", "robot_1/odom"), ("robot_1/base_link", "robot_1/lidar_link")}
        s = got_scan[-1]
        assert s.header.frame_id == "robot_1/lidar_link" and list(s.ranges) == pytest.approx([1.0, 0.0, 2.0])
        ex.shutdown()
        n.destroy_node()
    finally:
        rclpy.try_shutdown()
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()
