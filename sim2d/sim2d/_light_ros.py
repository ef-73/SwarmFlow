"""ROS side of light mode: one node per robot serving NavigateThroughPoses from a LightWorld (imports rclpy)."""

from __future__ import annotations

import math
import threading
import time
from typing import Tuple

from geometry_msgs.msg import PoseStamped, TransformStamped
from nav2_msgs.action import NavigateThroughPoses
from nav_msgs.msg import Path
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage

from .light_world import LightWorld

POLL_S = 0.02
PLAN_EVERY = 10          # publish the plan every N sim steps
_STATIC_QOS = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def _stamp(msg_stamp, t: float) -> None:
    msg_stamp.sec, msg_stamp.nanosec = int(t), int((t % 1.0) * 1e9)


class RobotNav(Node):
    def __init__(self, rid: str, world: LightWorld, lock: threading.Lock, spawn: Tuple[float, float, float]):
        super().__init__("light_nav", namespace=rid)
        self.rid, self._world, self._lock = rid, world, lock
        self._gen = 0
        self.stopping = False
        self._ticks = 0
        self._tf_pub = self.create_publisher(TFMessage, "tf", 100)
        self._plan_pub = self.create_publisher(Path, "plan", 10)
        static_pub = self.create_publisher(TFMessage, "tf_static", _STATIC_QOS)
        s = TransformStamped()
        s.header.frame_id, s.child_frame_id = "map", "odom"
        s.transform.rotation.w = 1.0
        static_pub.publish(TFMessage(transforms=[s]))
        self._static_pub = static_pub          # keep alive: the latched sample must stay available
        self._server = ActionServer(
            self, NavigateThroughPoses, "navigate_through_poses", self._execute,
            goal_callback=lambda _g: GoalResponse.ACCEPT, cancel_callback=lambda _h: CancelResponse.ACCEPT,
            callback_group=ReentrantCallbackGroup())

    # called from the sim loop after every world step
    def publish(self, sim_t: float) -> None:
        with self._lock:
            x, y, yaw = self._world.pose(self.rid)
            goals = self._world.remaining_goals(self.rid) if self._ticks % PLAN_EVERY == 0 else None
        tf = TransformStamped()
        _stamp(tf.header.stamp, sim_t)
        tf.header.frame_id, tf.child_frame_id = "odom", "base_footprint"
        tf.transform.translation.x, tf.transform.translation.y = x, y
        tf.transform.rotation.z, tf.transform.rotation.w = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
        self._tf_pub.publish(TFMessage(transforms=[tf]))
        if goals is not None:
            path = Path()
            _stamp(path.header.stamp, sim_t)
            path.header.frame_id = "map"
            for gx, gy, gyaw in [(x, y, yaw)] + goals if goals else []:
                p = PoseStamped()
                p.header = path.header
                p.pose.position.x, p.pose.position.y = gx, gy
                gyaw = yaw if gyaw is None else gyaw
                p.pose.orientation.z, p.pose.orientation.w = math.sin(gyaw / 2.0), math.cos(gyaw / 2.0)
                path.poses.append(p)
            self._plan_pub.publish(path)
        self._ticks += 1

    def _execute(self, goal_handle):
        result = NavigateThroughPoses.Result()
        poses = list(goal_handle.request.poses)
        with self._lock:
            self._gen += 1
            gen = self._gen
            if not poses:
                self._world.cancel(self.rid)
            else:
                goals = []
                for k, p in enumerate(poses):
                    last = k == len(poses) - 1
                    q = p.pose.orientation
                    goals.append((p.pose.position.x, p.pose.position.y, 2.0 * math.atan2(q.z, q.w) if last else None))
                self._world.set_goals(self.rid, goals)
        if not poses:
            goal_handle.abort()
            return result
        last_fb = 0.0
        while True:
            time.sleep(POLL_S)
            if self.stopping:
                goal_handle.abort()
                return result
            with self._lock:
                if self._gen != gen:                      # preempted by a newer goal
                    goal_handle.abort()
                    return result
                if goal_handle.is_cancel_requested:
                    self._world.cancel(self.rid)
                    goal_handle.canceled()
                    return result
                state = self._world.state(self.rid)
                remaining = self._world.distance_remaining(self.rid)
                n_left = len(self._world.remaining_goals(self.rid))
                x, y, _ = self._world.pose(self.rid)
            if state == "arrived":
                goal_handle.succeed()
                return result
            now = time.monotonic()
            if now - last_fb >= 0.5:
                last_fb = now
                fb = NavigateThroughPoses.Feedback()
                fb.current_pose.header.frame_id = "map"
                fb.current_pose.pose.position.x, fb.current_pose.pose.position.y = x, y
                fb.distance_remaining = float(remaining)
                fb.number_of_poses_remaining = n_left
                goal_handle.publish_feedback(fb)
