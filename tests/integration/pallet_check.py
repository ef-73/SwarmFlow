#!/usr/bin/env python3
"""Pallet order end to end in Gazebo (M16: F-01 ... F-05). Needs Gazebo; sim lock held by the caller.

Runs inside robot_1's container while gazebo, orchestrator (policy assign), payload and robot_1 are up:
    python3 /tmp/pallet_check.py PL_W1 DK1 [--timeout 300]
Publishes one pallet order, then records:
  - the docking error when the lift completes (first CARRYING sample): lateral and longitudinal distance of the robot
    from the pallet pose along the pallet yaw (target: lateral <= 3 cm);
  - the local costmap footprint while carrying and after delivery (carry 1.25 x 1.05, then normal 0.70 x 0.60);
  - DELIVERED within --timeout sim seconds.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import threading
import time

import rclpy
import yaml
from rcl_interfaces.srv import GetParameters
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from swarmflow_interfaces.msg import Order, OrderStatus, RobotState

ZONES = "/ws/layouts/warehouse_v2/generated/zones.yaml"


class Check(Node):
    def __init__(self):
        super().__init__("pallet_check", parameter_overrides=[rclpy.parameter.Parameter("use_sim_time", value=True)])
        self.lock = threading.Lock()
        self.state, self.status = None, []
        self.create_subscription(RobotState, "/fleet/robot_states", self._s, 50)
        self.create_subscription(OrderStatus, "/fleet/order_status", lambda m: self.status.append(m),
                                 QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                                            durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.pub = self.create_publisher(Order, "/fleet/orders", QoSProfile(depth=100,
                                                                           reliability=ReliabilityPolicy.RELIABLE))
        self.params = self.create_client(GetParameters, "/robot_1/local_costmap/local_costmap/get_parameters")

    def _s(self, m):
        if m.robot_id == "robot_1":
            with self.lock:
                self.state = m

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def footprint(self):
        if not self.params.wait_for_service(timeout_sec=5.0):
            return None
        f = self.params.call_async(GetParameters.Request(names=["footprint"]))
        end = time.time() + 5.0
        while not f.done() and time.time() < end:
            time.sleep(0.05)
        return f.result().values[0].string_value if f.done() and f.result() else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("spot")
    ap.add_argument("dock")
    ap.add_argument("--timeout", type=float, default=300.0)
    args = ap.parse_args()
    v = yaml.safe_load(open(ZONES, encoding="utf-8"))["vertices"][args.spot]
    px, py, pyaw = v["x"], v["y"], v.get("yaw", 0.0)
    rclpy.init()
    n = Check()
    ex = MultiThreadedExecutor(num_threads=3)
    ex.add_node(n)
    threading.Thread(target=ex.spin, daemon=True).start()
    end = time.time() + 30
    while n.pub.get_subscription_count() == 0 and time.time() < end:
        time.sleep(0.2)
    while n.now() <= 0.0:            # sim clock not received yet: a zero start time would end the wait at once
        time.sleep(0.1)
    oid = f"pal_{args.spot}_{int(time.time())}"
    n.pub.publish(Order(order_id=oid, pickup_vertex=args.spot, dropoff_vertex=args.dock, payload_type="pallet",
                        order_type=Order.TYPE_PALLET, pallet_id=f"pallet_{args.spot}"))
    t0, dock_err, fp_carry, delivered, modes = n.now(), None, None, None, []
    while n.now() - t0 < args.timeout:
        with n.lock:
            s = n.state
        if s is not None and s.order_id == oid:
            if not modes or modes[-1] != s.mode:
                modes.append(s.mode)
            if dock_err is None and s.mode == RobotState.MODE_CARRYING:
                dx, dy = s.x - px, s.y - py
                dock_err = {"lateral_m": round(abs(-math.sin(pyaw) * dx + math.cos(pyaw) * dy), 3),
                            "longitudinal_m": round(math.cos(pyaw) * dx + math.sin(pyaw) * dy, 3)}
                fp_carry = n.footprint()
        if [m for m in n.status if m.order_id == oid and m.state == OrderStatus.STATE_DELIVERED]:
            delivered = round(n.now() - t0, 1)
            break
        time.sleep(0.1)
    time.sleep(3.0)
    fp_after = n.footprint()
    names = {getattr(RobotState, k): k[5:] for k in dir(RobotState) if k.startswith("MODE_")}
    out = {"spot": args.spot, "dock": args.dock, "delivered_after_s": delivered, "dock_error": dock_err,
           "footprint_carry": fp_carry, "footprint_after": fp_after, "modes": [names.get(m, m) for m in modes]}
    out["pass"] = bool(delivered and dock_err and dock_err["lateral_m"] <= 0.03 and fp_carry and "0.625" in fp_carry
                       and fp_after and "0.35" in fp_after)
    print(json.dumps(out), flush=True)
    ex.shutdown()
    rclpy.try_shutdown()
    sys.exit(0 if out["pass"] else 1)


if __name__ == "__main__":
    main()
