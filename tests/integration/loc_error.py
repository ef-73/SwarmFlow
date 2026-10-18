#!/usr/bin/env python3
"""Localisation error recorder (realism ladder I-01). Runs inside a robot container during a run.

Compares every robot's reported pose (/fleet/robot_states, from AMCL when localization:=amcl) with Gazebo's ground
truth (/<robot>/odom_gt, bridged only in the AMCL rung) and writes mean / p95 / max position error per robot.
    python3 /tmp/loc_error.py --robots robot_1,robot_2 --duration 600 --out /tmp/loc_error.json
"""

from __future__ import annotations

import argparse
import json
import math
import threading
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from swarmflow_interfaces.msg import RobotState


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--robots", required=True)
    ap.add_argument("--duration", type=float, default=600.0, help="sim seconds")
    ap.add_argument("--out", required=True)
    args = ap.parse_args()
    robots = args.robots.split(",")
    rclpy.init()
    n = Node("loc_error", parameter_overrides=[rclpy.parameter.Parameter("use_sim_time", value=True)])
    gt, err, lock = {}, {r: [] for r in robots}, threading.Lock()
    for r in robots:
        n.create_subscription(Odometry, f"/{r}/odom_gt",
                              lambda m, r=r: gt.__setitem__(r, (m.pose.pose.position.x, m.pose.pose.position.y)), 10)

    def on_state(m):
        with lock:
            if m.robot_id in gt and m.mode != RobotState.MODE_FAULT:
                g = gt[m.robot_id]
                err[m.robot_id].append(math.hypot(m.x - g[0], m.y - g[1]))
    n.create_subscription(RobotState, "/fleet/robot_states", on_state, 50)
    ex = MultiThreadedExecutor(num_threads=2)
    ex.add_node(n)
    threading.Thread(target=ex.spin, daemon=True).start()
    while n.get_clock().now().nanoseconds == 0:
        time.sleep(0.1)
    t0 = n.get_clock().now().nanoseconds * 1e-9
    def dump():
        out = {}
        with lock:
            for r, e in err.items():
                s = sorted(e)
                out[r] = {"samples": len(s), "mean_m": round(sum(s) / len(s), 3) if s else None,
                          "p95_m": round(s[int(0.95 * (len(s) - 1))], 3) if s else None,
                          "max_m": round(s[-1], 3) if s else None}
        with open(args.out, "w", encoding="utf-8") as f:
            json.dump(out, f, indent=1)
        return out

    # written every 30 s: the run may be torn down before the duration ends
    last = time.time()
    while n.get_clock().now().nanoseconds * 1e-9 - t0 < args.duration:
        time.sleep(1.0)
        if time.time() - last > 30.0:
            dump()
            last = time.time()
    print(json.dumps(dump()))
    ex.shutdown()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
