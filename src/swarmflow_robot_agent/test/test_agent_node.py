"""Robot agent ROS node test against a fake NavigateThroughPoses server (design §6.3, §14.2 WS-D acceptance:
"agent drives a fake NavigateThroughPoses server through a route with a zone hold"). Lead test: do not edit.

Runs in the dev image with plain rclpy (no Gazebo, no Nav2). Wall clock (use_sim_time false) for speed.
"""

from __future__ import annotations

import pathlib
import threading
import time

import pytest

rclpy = pytest.importorskip("rclpy")
from rclpy.action import ActionClient  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from std_msgs.msg import Header  # noqa: E402

from swarmflow_interfaces.action import DispatchTask  # noqa: E402
from swarmflow_interfaces.msg import Order, ReservationRelease, RobotState  # noqa: E402
from swarmflow_interfaces.srv import RequestReservation  # noqa: E402
from swarmflow_robot_agent.agent_node import AgentNode  # noqa: E402
from swarmflow_robot_agent.fake_nav2 import FakeNavigateThroughPoses  # noqa: E402

FIX = pathlib.Path(__file__).resolve().parents[3] / "tests" / "fixtures" / "standard"
ROUTE = ["P1", "X_0_0", "X_1_0", "L1", "X_1_0", "X_2_0", "X_3_0", "H_3_1", "X_3_1", "X_3_3", "X_3_4", "D3"]
TF_REMAP = ["--ros-args", "-r", "/tf:=tf", "-r", "/tf_static:=tf_static"]


@pytest.fixture(scope="module")
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


def test_agent_drives_fake_nav2_with_zone_hold(ros):
    agent = AgentNode(namespace="robot_1", cli_args=TF_REMAP, parameter_overrides=[
        Parameter("robot_id", value="robot_1"),
        Parameter("layout_dir", value=str(FIX)),
        Parameter("traffic_control", value=True)])
    nav = FakeNavigateThroughPoses(namespace="robot_1", cli_args=TF_REMAP, start_xy=(1.3, 1.1), speed_mps=4.0)
    orch = rclpy.create_node("fake_orchestrator")
    requests, releases, states = [], [], []
    deny_first = {"done": False}

    def on_request(req, res):
        requests.append(req)
        if not deny_first["done"]:          # first request denied → agent must hold and retry
            deny_first["done"] = True
            res.result = RequestReservation.Response.RESULT_DENIED
            res.reason = "ZONE_LEASED"
            res.retry_after_s = 0.5
            return res
        res.result = RequestReservation.Response.RESULT_GRANTED
        res.lease_id = f"lease_{len(requests):05d}"
        res.retry_after_s = 0.0
        now = orch.get_clock().now().to_msg()
        res.lease_expiry.sec = now.sec + 5
        return res

    orch.create_service(RequestReservation, "/fleet/request_reservation", on_request)
    orch.create_subscription(ReservationRelease, "/fleet/reservation_release", releases.append, 10)
    orch.create_subscription(RobotState, "/fleet/robot_states", states.append, 50)
    hb = orch.create_publisher(Header, "/fleet/orchestrator_heartbeat", 10)
    orch.create_timer(0.5, lambda: hb.publish(Header(stamp=orch.get_clock().now().to_msg())))
    client = ActionClient(orch, DispatchTask, "/robot_1/dispatch_task")

    ex = MultiThreadedExecutor(num_threads=4)
    for n in (agent, nav, orch):
        ex.add_node(n)
    spin = threading.Thread(target=ex.spin, daemon=True)
    spin.start()
    try:
        assert client.wait_for_server(timeout_sec=10.0)
        time.sleep(1.0)  # TF + heartbeat warm-up
        goal = DispatchTask.Goal(task_id="task_0001", robot_id="robot_1", route=ROUTE,
                                 order=Order(order_id="o1", pickup_vertex="L1", dropoff_vertex="D3",
                                             payload_type="small"))
        fut = client.send_goal_async(goal)
        deadline = time.time() + 10.0
        while not fut.done() and time.time() < deadline:
            time.sleep(0.05)
        handle = fut.result()
        assert handle is not None and handle.accepted
        rfut = handle.get_result_async()
        deadline = time.time() + 60.0
        while not rfut.done() and time.time() < deadline:
            time.sleep(0.1)
        assert rfut.done(), "task did not finish within 60 s"
        result = rfut.result().result
        assert result.success, result.failure_reason
        zone_reqs = [r for r in requests if r.zone_id == "Z_aisle_3"]
        assert len(zone_reqs) >= 2 and zone_reqs[0].entry_vertex == "X_3_1" and zone_reqs[0].robot_id == "robot_1"
        assert any(r.lease_id and r.reason == ReservationRelease.REASON_EXITED for r in releases)
        modes = {s.mode for s in states if s.robot_id == "robot_1"}
        assert {RobotState.MODE_NAVIGATING, RobotState.MODE_WAITING_RESERVATION, RobotState.MODE_LOADING,
                RobotState.MODE_UNLOADING} <= modes
        assert states[-1].header.frame_id == "map"
    finally:
        ex.shutdown()
        for n in (agent, nav, orch):
            n.destroy_node()
