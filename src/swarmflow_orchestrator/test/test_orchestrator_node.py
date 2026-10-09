"""Orchestrator ROS adapter test (design §6.5, §6.6, §10, §13.5; WS-B acceptance "order → DispatchTask goal →
ASSIGN decision event"). Lead test: do not edit. Wall clock (use_sim_time false), rclpy only, no Gazebo/Nav2.
"""

from __future__ import annotations

import json
import pathlib
import threading
import time

import pytest

rclpy = pytest.importorskip("rclpy")
from rclpy.action import ActionServer  # noqa: E402
from rclpy.callback_groups import ReentrantCallbackGroup  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402
from std_msgs.msg import Header  # noqa: E402

from swarmflow_interfaces.action import DispatchTask  # noqa: E402
from swarmflow_interfaces.msg import (DecisionEvent, Order, OrderStatus, ReservationRelease,  # noqa: E402
                                      RobotState, ZoneReservation)
from swarmflow_interfaces.srv import ClearZone, RequestReservation  # noqa: E402
from swarmflow_orchestrator.orchestrator_node import OrchestratorNode  # noqa: E402

FIX = pathlib.Path(__file__).resolve().parents[3] / "tests" / "fixtures" / "standard"


def wait(pred, timeout=10.0):
    end = time.time() + timeout
    while time.time() < end:
        if pred():
            return True
        time.sleep(0.05)
    return False


@pytest.fixture(scope="module")
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


def test_orchestrator_end_to_end(ros, tmp_path):
    orch = OrchestratorNode(parameter_overrides=[
        Parameter("layout_dir", value=str(FIX)),
        Parameter("robots", value=["robot_1"]),
        Parameter("policy", value="fcfs"),
        Parameter("run_dir", value=str(tmp_path)),
    ])
    t = rclpy.create_node("orch_test")
    cb = ReentrantCallbackGroup()
    goals, statuses, decisions, reservations, beats = [], [], [], [], []

    def execute(gh):
        goals.append(gh.request)
        time.sleep(1.0)
        gh.succeed()
        return DispatchTask.Result(success=True)

    ActionServer(t, DispatchTask, "/robot_1/dispatch_task", execute_callback=execute, callback_group=cb)
    latched = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    t.create_subscription(OrderStatus, "/fleet/order_status", statuses.append, latched)
    t.create_subscription(DecisionEvent, "/fleet/decisions", decisions.append, 100)
    t.create_subscription(ZoneReservation, "/fleet/reservations", reservations.append, latched)
    t.create_subscription(Header, "/fleet/orchestrator_heartbeat", beats.append, 10)
    state_pub = t.create_publisher(RobotState, "/fleet/robot_states", 10)
    order_pub = t.create_publisher(Order, "/fleet/orders", 10)
    rel_pub = t.create_publisher(ReservationRelease, "/fleet/reservation_release", 10)
    req_cli = t.create_client(RequestReservation, "/fleet/request_reservation", callback_group=cb)
    clear_cli = t.create_client(ClearZone, "/fleet/clear_zone", callback_group=cb)

    def publish_state():
        s = RobotState(robot_id="robot_1", x=1.3, y=1.1, yaw=0.0, mode=RobotState.MODE_IDLE, last_vertex="P1")
        s.header.stamp = t.get_clock().now().to_msg()
        s.header.frame_id = "map"
        state_pub.publish(s)

    t.create_timer(0.2, publish_state, callback_group=cb)
    ex = MultiThreadedExecutor(num_threads=6)
    ex.add_node(orch)
    ex.add_node(t)
    threading.Thread(target=ex.spin, daemon=True).start()
    try:
        assert wait(lambda: len(beats) >= 2, 10.0), "no orchestrator heartbeat"
        assert beats[-1].frame_id.startswith("epoch:")
        assert req_cli.wait_for_service(timeout_sec=10.0)
        time.sleep(3.0)  # recovery grace (api.RECOVERY_GRACE_S) has passed

        req = RequestReservation.Request(request_id="r1", robot_id="robot_1", zone_id="Z_aisle_1",
                                         entry_vertex="X_1_1", exit_vertex="X_1_3")
        res = req_cli.call(req)
        assert res.result == RequestReservation.Response.RESULT_GRANTED and res.lease_id
        time.sleep(0.6)  # a fresh pose (stamped after the grant, outside the zone) has reached the authority
        rel_pub.publish(ReservationRelease(robot_id="robot_1", lease_id=res.lease_id,
                                           reason=ReservationRelease.REASON_EXITED))
        assert wait(lambda: [r.state for r in reservations if r.lease_id == res.lease_id] ==
                    [ZoneReservation.STATE_GRANTED, ZoneReservation.STATE_RELEASED])
        bad = req_cli.call(RequestReservation.Request(request_id="r2", robot_id="robot_1", zone_id="Z_nope",
                                                      entry_vertex="X_1_1", exit_vertex="X_1_3"))
        assert bad.result == RequestReservation.Response.RESULT_DENIED and bad.reason == "UNKNOWN_ZONE"

        order_pub.publish(Order(order_id="o1", pickup_vertex="L1", dropoff_vertex="D1", payload_type="small"))
        assert wait(lambda: goals, 10.0), "no DispatchTask goal"
        g = goals[0]
        assert g.robot_id == "robot_1" and g.order.order_id == "o1" and g.route[0] == "P1" and g.route[-1] == "D1"
        assert "L1" in g.route
        assert wait(lambda: [s.state for s in statuses if s.order_id == "o1"][-1:] == [OrderStatus.STATE_DELIVERED],
                    15.0), [s.state for s in statuses]
        seq = [s.state for s in statuses if s.order_id == "o1"]
        assert seq[:2] == [OrderStatus.STATE_QUEUED, OrderStatus.STATE_ASSIGNED]
        types = [d.decision_type for d in decisions]
        assert "ASSIGN" in types and "RESERVATION_GRANT" in types and "RESERVATION_DENY" in types
        a = [d for d in decisions if d.decision_type == "ASSIGN"][0]
        assert a.policy == "fcfs" and a.robot_id == "robot_1" and a.order_id == "o1" and a.explanation

        assert clear_cli.call(ClearZone.Request(zone_id="Z_aisle_2", reason="test")).success
        assert not clear_cli.call(ClearZone.Request(zone_id="Z_nope", reason="test")).success

        assert wait(lambda: (tmp_path / "decisions.jsonl").exists() and (tmp_path / "orders.jsonl").exists(), 5.0)
        lines = [json.loads(x) for x in (tmp_path / "decisions.jsonl").read_text().splitlines()]
        assert any(x["decision_type"] == "ASSIGN" and x["order_id"] == "o1" for x in lines)
        orders = [json.loads(x) for x in (tmp_path / "orders.jsonl").read_text().splitlines()]
        assert [o["state"] for o in orders if o["order_id"] == "o1"][-1] == "DELIVERED"
        assert wait(lambda: (tmp_path / "robot_states.jsonl").exists(), 5.0)
    finally:
        ex.shutdown()
        orch.destroy_node()
        t.destroy_node()


def test_independent_policy_never_grants(ros, tmp_path):
    orch = OrchestratorNode(parameter_overrides=[
        Parameter("layout_dir", value=str(FIX)), Parameter("robots", value=["robot_1"]),
        Parameter("policy", value="independent"), Parameter("run_dir", value="")])
    t = rclpy.create_node("orch_test2")
    cli = t.create_client(RequestReservation, "/fleet/request_reservation")
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(orch)
    ex.add_node(t)
    threading.Thread(target=ex.spin, daemon=True).start()
    try:
        assert cli.wait_for_service(timeout_sec=10.0)
        time.sleep(3.0)
        fut = cli.call_async(RequestReservation.Request(request_id="r", robot_id="robot_1", zone_id="Z_aisle_1",
                                                        entry_vertex="X_1_1", exit_vertex="X_1_3"))
        assert wait(fut.done, 5.0)
        assert fut.result().result == RequestReservation.Response.RESULT_DENIED  # Baseline A: no traffic control
    finally:
        ex.shutdown()
        orch.destroy_node()
        t.destroy_node()
