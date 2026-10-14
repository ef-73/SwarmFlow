#!/usr/bin/env python3
"""Blocked-robot check on warehouse_v2 (improvements_v2 N-03/N-04, M14). Lead-only, Gazebo, agents on.

robot_2 is parked on a pick point; robot_1 gets a v2 goal task (DispatchTask, ROUTE_GOALS) to that same pick point
through its robot agent. The goal can never be reached. Pass when, within --limit sim seconds of the dispatch,
robot_1's RobotState shows MODE_STUCK with fault_reason "BLOCKED" (api.FAULT_BLOCKED), the task is still running
(no failure result), and robot_1 never touched robot_2 (padded footprints may not overlap).
    python3 /tmp/blocked_check.py [--limit 30]
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateThroughPoses
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from swarmflow_core.metrics.clearance import footprint_distance
from swarmflow_interfaces.action import DispatchTask
from swarmflow_interfaces.msg import Order, RobotState

PICK = "PK_W2_S_02"
PICK_XY = (5.0, 12.65)
START_1 = (10.0, 13.3)          # robot_1 waits in the central aisle at the mouth of aisle W2


class Check(Node):
    def __init__(self):
        super().__init__("blocked_check", parameter_overrides=[rclpy.parameter.Parameter("use_sim_time", value=True)])
        self.states, self.lock = {}, threading.Lock()
        self.log = []
        self.create_subscription(RobotState, "/fleet/robot_states", self._state, 50)
        self.nav = {r: ActionClient(self, NavigateThroughPoses, f"/{r}/navigate_through_poses")
                    for r in ("robot_1", "robot_2")}
        self.task = ActionClient(self, DispatchTask, "/robot_1/dispatch_task")

    def _state(self, m):
        with self.lock:
            self.states[m.robot_id] = m
            if m.robot_id == "robot_1":
                self.log.append((self.now(), m.mode, m.fault_reason, m.task_id))

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def go(self, robot, x, y, yaw):
        g = NavigateThroughPoses.Goal()
        p = PoseStamped()
        p.header.frame_id = "map"
        p.pose.position.x, p.pose.position.y = x, y
        p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        g.poses = [p]
        f = self.nav[robot].send_goal_async(g)
        while not f.done():
            time.sleep(0.05)
        r = f.result().get_result_async()
        while not r.done():
            time.sleep(0.1)
        return r.result().status == GoalStatus.STATUS_SUCCEEDED


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--limit", type=float, default=30.0)
    args = ap.parse_args()
    rclpy.init()
    n = Check()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(n)
    threading.Thread(target=ex.spin, daemon=True).start()
    for c in list(n.nav.values()) + [n.task]:
        if not c.wait_for_server(timeout_sec=90.0):
            print("action server missing"); sys.exit(2)
    ok2 = n.go("robot_2", *PICK_XY, math.pi)
    ok1 = n.go("robot_1", *START_1, math.pi)
    print(f"staging robot_2: {'ok' if ok2 else 'FAILED'}; robot_1: {'ok' if ok1 else 'FAILED'}", flush=True)
    if not (ok1 and ok2):
        sys.exit(3)
    goal = DispatchTask.Goal(task_id="blk_1", robot_id="robot_1", route=[PICK],
                             route_kind=DispatchTask.Goal.ROUTE_GOALS,
                             order=Order(order_id="blk", dropoff_vertex="DK1", order_type=Order.TYPE_SHELF,
                                         pick_vertices=[PICK]))
    f = n.task.send_goal_async(goal)
    while not f.done():
        time.sleep(0.05)
    gh = f.result()
    t0 = n.now()
    res = gh.get_result_async()
    blocked_at, min_d = None, math.inf
    while n.now() - t0 < args.limit + 5.0:
        with n.lock:
            s1, s2 = n.states.get("robot_1"), n.states.get("robot_2")
        if s1 and s2:
            min_d = min(min_d, footprint_distance((s1.x, s1.y, s1.yaw), (s2.x, s2.y, s2.yaw)))
            if blocked_at is None and s1.mode == RobotState.MODE_STUCK and s1.fault_reason == "BLOCKED" \
                    and s1.task_id == "blk_1":
                blocked_at = n.now() - t0
        if res.done():
            break
        time.sleep(0.1)
    task_alive = not res.done()
    if task_alive:
        gh.cancel_goal_async()
        time.sleep(1.0)
    modes = sorted({(m, r) for _, m, r, tid in n.log if tid == "blk_1"})
    out = {"blocked_after_s": None if blocked_at is None else round(blocked_at, 1), "task_alive": task_alive,
           "min_separation_m": round(min_d, 3), "modes_seen": modes}
    out["pass"] = blocked_at is not None and blocked_at <= args.limit and task_alive and min_d >= 0.0
    print(json.dumps(out), flush=True)
    ex.shutdown()
    rclpy.try_shutdown()
    sys.exit(0 if out["pass"] else 1)


if __name__ == "__main__":
    main()
