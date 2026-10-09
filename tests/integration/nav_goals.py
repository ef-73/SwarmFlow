#!/usr/bin/env python3
"""Send NavigateToPose goals to one robot and report success + sim time per goal (M4 / §15.2 item 1).

Run inside a robot container:  python3 /ws/tests/integration/nav_goals.py robot_1 X_1_1 X_1_3 D1 L2 P1
Vertex names are looked up in the layout's generated nav graph; yaw = 0 except stations (station yaw).
Exit code 0 only if every goal succeeded.
"""

import math
import sys
import time

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node

LAYOUT = "/ws/layouts/standard/generated/zones.yaml"


def main():
    robot, goals = sys.argv[1], sys.argv[2:]
    verts = yaml.safe_load(open(LAYOUT, encoding="utf-8"))["vertices"]
    rclpy.init()
    node = Node("nav_goals", parameter_overrides=[rclpy.parameter.Parameter("use_sim_time", value=True)])
    client = ActionClient(node, NavigateToPose, f"/{robot}/navigate_to_pose")
    if not client.wait_for_server(timeout_sec=60.0):
        print("navigate_to_pose server not available"); sys.exit(2)
    ok_all = True
    for g in goals:
        v = verts[g]
        yaw = float(v.get("yaw") or 0.0)
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.pose.position.x, goal.pose.pose.position.y = float(v["x"]), float(v["y"])
        goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        t0 = node.get_clock().now().nanoseconds / 1e9
        w0 = time.time()
        fut = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=30.0)
        handle = fut.result()
        if handle is None or not handle.accepted:
            print(f"{g}: REJECTED"); ok_all = False; continue
        rfut = handle.get_result_async()
        rclpy.spin_until_future_complete(node, rfut, timeout_sec=300.0)
        res = rfut.result()
        status = res.status if res else -1
        t1 = node.get_clock().now().nanoseconds / 1e9
        ok = status == GoalStatus.STATUS_SUCCEEDED
        ok_all &= ok
        print(f"{g}: {'SUCCEEDED' if ok else f'FAILED(status={status})'} sim {t1 - t0:.1f} s, wall {time.time() - w0:.1f} s",
              flush=True)
    rclpy.shutdown()
    sys.exit(0 if ok_all else 1)


if __name__ == "__main__":
    main()
