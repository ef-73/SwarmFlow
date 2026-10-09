"""Fake ``NavigateThroughPoses`` action server for tests (no Nav2, no Gazebo).

Moves a simulated pose through the goal poses at a constant speed and publishes TF ``map`` -> ``base_footprint`` on
the (namespace-relative) ``tf`` topic at 20 Hz. Wall-clock based, so it also works with ``use_sim_time`` false.
"""

from __future__ import annotations

import math
import threading
import time
from typing import Tuple

import rclpy
from geometry_msgs.msg import TransformStamped
from nav2_msgs.action import NavigateThroughPoses
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from tf2_msgs.msg import TFMessage

STEP_S = 0.05
ARRIVE_EPS_M = 1e-6


class FakeNavigateThroughPoses(Node):
    def __init__(self, start_xy: Tuple[float, float] = (0.0, 0.0), speed_mps: float = 1.0, **kwargs):
        super().__init__("fake_nav2", **kwargs)
        self._lock = threading.Lock()
        self._x, self._y = float(start_xy[0]), float(start_xy[1])
        self._yaw = 0.0
        self._speed = float(speed_mps)
        self._fail = 0
        self._gen = 0
        self.goals_received = 0
        cb = ReentrantCallbackGroup()
        self._tf_pub = self.create_publisher(TFMessage, "tf", 100)
        self.create_timer(0.05, self._publish_tf, callback_group=cb)
        self._server = ActionServer(
            self, NavigateThroughPoses, "navigate_through_poses", self._execute,
            goal_callback=lambda _g: GoalResponse.ACCEPT, cancel_callback=lambda _h: CancelResponse.ACCEPT,
            callback_group=cb)

    def fail_next(self, n: int = 1) -> None:
        """The next ``n`` goals abort immediately."""
        with self._lock:
            self._fail = int(n)

    def pose(self) -> Tuple[float, float, float]:
        with self._lock:
            return self._x, self._y, self._yaw

    def _publish_tf(self) -> None:
        x, y, yaw = self.pose()
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = "map"
        tf.child_frame_id = "base_footprint"
        tf.transform.translation.x = x
        tf.transform.translation.y = y
        tf.transform.rotation.z = math.sin(yaw / 2.0)
        tf.transform.rotation.w = math.cos(yaw / 2.0)
        self._tf_pub.publish(TFMessage(transforms=[tf]))

    def _execute(self, goal_handle):
        with self._lock:
            self._gen += 1
            gen = self._gen
            self.goals_received += 1
            fail = self._fail > 0
            if fail:
                self._fail -= 1
        result = NavigateThroughPoses.Result()
        poses = list(goal_handle.request.poses)
        if fail or not poses:
            time.sleep(STEP_S)
            goal_handle.abort()
            return result
        total = 0.0
        px, py = self.pose()[:2]
        for p in poses:
            total += math.hypot(p.pose.position.x - px, p.pose.position.y - py)
            px, py = p.pose.position.x, p.pose.position.y
        travelled = 0.0
        t0 = time.monotonic()
        for k, p in enumerate(poses):
            gx, gy = p.pose.position.x, p.pose.position.y
            q = p.pose.orientation
            gyaw = 2.0 * math.atan2(q.z, q.w)
            while True:
                with self._lock:
                    if self._gen != gen:
                        goal_handle.abort()
                        return result
                    dx, dy = gx - self._x, gy - self._y
                    dist = math.hypot(dx, dy)
                    step = self._speed * STEP_S
                    if dist <= max(step, ARRIVE_EPS_M):
                        travelled += dist
                        self._x, self._y = gx, gy
                        self._yaw = gyaw
                        break
                    self._x += dx / dist * step
                    self._y += dy / dist * step
                    self._yaw = math.atan2(dy, dx)
                    travelled += step
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    return result
                fb = NavigateThroughPoses.Feedback()
                fb.current_pose.header.frame_id = "map"
                fb.current_pose.pose.position.x, fb.current_pose.pose.position.y = self._x, self._y
                fb.distance_remaining = float(max(0.0, total - travelled))
                fb.number_of_poses_remaining = len(poses) - k
                el = time.monotonic() - t0
                fb.navigation_time.sec, fb.navigation_time.nanosec = int(el), int((el % 1.0) * 1e9)
                goal_handle.publish_feedback(fb)
                time.sleep(STEP_S)
        goal_handle.succeed()
        return result


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FakeNavigateThroughPoses()
    try:
        rclpy.spin(node, executor=rclpy.executors.MultiThreadedExecutor())
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
