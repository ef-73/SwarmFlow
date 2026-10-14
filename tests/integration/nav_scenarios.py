#!/usr/bin/env python3
"""Multi-robot local-navigation scenarios on warehouse_v2 (improvements_v2 M14: N-01 ... N-09). Lead-only, Gazebo.

Run inside a robot container while gazebo + robot_N (agent:=false) are up:
    python3 /ws/tests/integration/nav_scenarios.py <scenario> [--out runs/nav/<id>.json]
Scenarios: head_on, pass_parked, crossing, dead_end (see SCENARIOS). Each robot is first driven to its start pose
(staging, one robot at a time), then all robots get their goal at once (NavigateThroughPoses with one pose, as the
v2 robot agent sends). Poses come from /<robot>/odom (ground truth); the result reports per robot success and time,
the minimum footprint-to-footprint distance (swarmflow_core.metrics.clearance) and near misses (< 0.30 m).
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
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from swarmflow_core.metrics.clearance import footprint_distance

N, S, E, W = math.pi / 2, -math.pi / 2, 0.0, math.pi
# name -> {robot: (start_xyyaw, goal_xyyaw)}; robots not listed stay where they are (parked = start only)
SCENARIOS = {
    # N-01/02/07: two robots meet head-on in the 4.0 m central aisle
    "head_on": {"robot_1": ((10.0, 6.0, N), (10.0, 19.0, N)), "robot_2": ((10.0, 19.0, S), (10.0, 6.0, S))},
    # N-01/02/07: robot_2 is parked at a pick point in storage aisle W2; robot_1 drives past it and back out
    "pass_parked": {"robot_2": ((5.0, 12.65, W), None),
                    "robot_1": ((7.4, 13.3, W), (1.2, 13.3, W))},
    # N-05: crossing at an aisle mouth: one robot along the central aisle, one from aisle W2 to aisle E2
    "crossing": {"robot_1": ((10.0, 8.0, N), (10.0, 18.5, N)), "robot_2": ((3.0, 13.3, E), (17.0, 13.3, E))},
    # N-08: two robots meet inside a dead-end 2.2 m storage aisle (W3: y 15.4 .. 17.6)
    "dead_end": {"robot_1": ((9.0, 16.5, W), (1.5, 16.5, W)), "robot_2": ((1.5, 16.5, E), (9.0, 16.5, E))},
    # the same meeting with the roles swapped: its starts are where dead_end ends, so the two alternate for repeats
    "dead_end_rev": {"robot_1": ((1.5, 16.5, E), (9.0, 16.5, E)), "robot_2": ((9.0, 16.5, W), (1.5, 16.5, W))},
}
# staging order where it matters (a robot parked at an aisle mouth would block the other's way in)
STAGING_ORDER = {"dead_end": ["robot_2", "robot_1"], "dead_end_rev": ["robot_1", "robot_2"]}
NEAR_MISS_M = 0.30


def pose_msg(node, x, y, yaw):
    p = PoseStamped()
    p.header.frame_id = "map"
    p.header.stamp = node.get_clock().now().to_msg()
    p.pose.position.x, p.pose.position.y = float(x), float(y)
    p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
    return p


class Runner(Node):
    def __init__(self, robots):
        super().__init__("nav_scenarios", parameter_overrides=[rclpy.parameter.Parameter("use_sim_time", value=True)])
        self.poses, self.lock = {}, threading.Lock()
        self.nav_clients = {}
        for r in robots:
            self.create_subscription(Odometry, f"/{r}/odom", lambda m, r=r: self._odom(r, m), 10)
            self.nav_clients[r] = ActionClient(self, NavigateThroughPoses, f"/{r}/navigate_through_poses")

    def _odom(self, r, m):
        q = m.pose.pose.orientation
        with self.lock:
            self.poses[r] = (m.pose.pose.position.x, m.pose.pose.position.y,
                             math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def send(self, r, xyyaw):
        goal = NavigateThroughPoses.Goal()
        goal.poses = [pose_msg(self, *xyyaw)]
        fut = self.nav_clients[r].send_goal_async(goal)
        while not fut.done():
            time.sleep(0.05)
        gh = fut.result()
        return gh.get_result_async() if gh.accepted else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("scenario", choices=sorted(SCENARIOS))
    ap.add_argument("--timeout", type=float, default=120.0, help="sim seconds for the simultaneous phase")
    ap.add_argument("--out", default="")
    args = ap.parse_args()
    spec = SCENARIOS[args.scenario]
    rclpy.init()
    node = Runner(sorted(spec))
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    for r, c in node.nav_clients.items():
        if not c.wait_for_server(timeout_sec=90.0):
            print(f"{r}: navigate_through_poses not available"); sys.exit(2)
    # staging: one robot at a time to its start pose (parked robots first)
    for r in STAGING_ORDER.get(args.scenario) or sorted(spec, key=lambda r: spec[r][1] is not None):
        res = node.send(r, spec[r][0])
        while res is not None and not res.done():
            time.sleep(0.1)
        ok = res is not None and res.result().status == GoalStatus.STATUS_SUCCEEDED
        print(f"staging {r}: {'ok' if ok else 'FAILED'}", flush=True)
        if not ok:
            sys.exit(3)
    # simultaneous phase
    movers = [r for r in sorted(spec) if spec[r][1] is not None]
    t0 = node.now()
    futs = {r: node.send(r, spec[r][1]) for r in movers}
    done, min_d, near, active = {}, math.inf, 0, set()
    while len(done) < len(movers) and node.now() - t0 < args.timeout:
        with node.lock:
            poses = dict(node.poses)
        ids = sorted(poses)
        for i, a in enumerate(ids):
            for b in ids[i + 1:]:
                d = footprint_distance(poses[a], poses[b])
                min_d = min(min_d, d)
                close = d < NEAR_MISS_M
                if close and (a, b) not in active:
                    near += 1
                (active.add if close else active.discard)((a, b))
        for r, f in futs.items():
            if r not in done and f is not None and f.done():
                done[r] = (f.result().status == GoalStatus.STATUS_SUCCEEDED, round(node.now() - t0, 1))
        time.sleep(0.1)
    result = {"scenario": args.scenario, "robots": {r: {"success": done.get(r, (False, None))[0],
                                                          "time_s": done.get(r, (False, None))[1]} for r in movers},
              "min_separation_m": round(min_d, 3), "near_misses": near, "contact": min_d < 0.0}
    result["pass"] = all(v["success"] for v in result["robots"].values()) and min_d >= 0.15
    print(json.dumps(result), flush=True)
    if args.out:
        with open(args.out, "w", encoding="utf-8") as f:
            json.dump(result, f, indent=1)
    ex.shutdown()
    rclpy.try_shutdown()
    sys.exit(0 if result["pass"] else 1)


if __name__ == "__main__":
    main()
