#!/usr/bin/env python3
"""Motion smoothness recorder (T069). Runs inside a robot container during a run, like loc_error.py.

Reads every robot's wheel odometry (/<robot>/odom: the velocities the robot really drove) at up to ``RATE_HZ`` and
writes, per robot and for the fleet, over the samples where the robot is driving (|v| >= 0.1 m/s):
- rms_ang_accel: RMS of d(turn rate)/dt (rad/s^2): how hard the heading is being jerked around
- rms_lin_accel: RMS of d(speed)/dt (m/s^2)
- steer_reversals_per_m: turn-rate sign changes (|w| >= 0.1 rad/s on both sides) per metre driven: wobble
- metres: distance driven
    python3 /tmp/smooth_probe.py --robots robot_1,robot_2 --duration 600 --out /tmp/smooth.json
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

RATE_HZ = 20.0
MOVING_MPS = 0.1
TURNING_RADPS = 0.1


class Acc:
    def __init__(self):
        self.last = None            # (t, v, w)
        self.sq_alpha = self.sq_acc = 0.0
        self.n = 0
        self.metres = 0.0
        self.reversals = 0
        self.last_turn = 0.0        # sign of the last turn rate above TURNING_RADPS

    def add(self, t, v, w):
        last, self.last = self.last, (t, v, w)
        if last is None:
            return
        dt = t - last[0]
        if dt < 0.5 / RATE_HZ or dt > 0.5:
            return                  # duplicate or a gap: no derivative across it
        if abs(v) < MOVING_MPS and abs(last[1]) < MOVING_MPS:
            return
        self.sq_alpha += ((w - last[2]) / dt) ** 2
        self.sq_acc += ((v - last[1]) / dt) ** 2
        self.n += 1
        self.metres += abs(v) * dt
        if abs(w) >= TURNING_RADPS:
            s = math.copysign(1.0, w)
            if self.last_turn and s != self.last_turn:
                self.reversals += 1
            self.last_turn = s

    def summary(self):
        if not self.n:
            return {"samples": 0}
        return {"samples": self.n, "metres": round(self.metres, 1),
                "rms_ang_accel": round(math.sqrt(self.sq_alpha / self.n), 3),
                "rms_lin_accel": round(math.sqrt(self.sq_acc / self.n), 3),
                "steer_reversals_per_m": round(self.reversals / self.metres, 3) if self.metres else None}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--robots", required=True)
    ap.add_argument("--duration", type=float, default=600.0, help="sim seconds")
    ap.add_argument("--out", required=True)
    args = ap.parse_args()
    robots = args.robots.split(",")
    rclpy.init()
    n = Node("smooth_probe", parameter_overrides=[rclpy.parameter.Parameter("use_sim_time", value=True)])
    acc, lock = {r: Acc() for r in robots}, threading.Lock()

    def on_odom(r, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        a = acc[r]
        if a.last is not None and t - a.last[0] < 1.0 / RATE_HZ - 1e-3:
            return
        with lock:
            a.add(t, m.twist.twist.linear.x, m.twist.twist.angular.z)

    for r in robots:
        n.create_subscription(Odometry, f"/{r}/odom", lambda m, r=r: on_odom(r, m), 10)
    ex = MultiThreadedExecutor(num_threads=2)
    ex.add_node(n)
    threading.Thread(target=ex.spin, daemon=True).start()
    while n.get_clock().now().nanoseconds == 0:
        time.sleep(0.1)
    t0 = n.get_clock().now().nanoseconds * 1e-9

    def dump():
        with lock:
            out = {r: a.summary() for r, a in acc.items()}
            tot = [a for a in acc.values() if a.n]
            n_all = sum(a.n for a in tot)
            m_all = sum(a.metres for a in tot)
            if n_all:
                out["fleet"] = {"samples": n_all, "metres": round(m_all, 1),
                                "rms_ang_accel": round(math.sqrt(sum(a.sq_alpha for a in tot) / n_all), 3),
                                "rms_lin_accel": round(math.sqrt(sum(a.sq_acc for a in tot) / n_all), 3),
                                "steer_reversals_per_m": round(sum(a.reversals for a in tot) / m_all, 3)}
        with open(args.out, "w", encoding="utf-8") as f:
            json.dump(out, f, indent=1)
        return out

    last = time.time()               # written every 30 s: the run may be torn down before the duration ends
    while n.get_clock().now().nanoseconds * 1e-9 - t0 < args.duration:
        time.sleep(1.0)
        if time.time() - last > 30.0:
            dump()
            last = time.time()
    print(json.dumps(dump()))


if __name__ == "__main__":
    main()
