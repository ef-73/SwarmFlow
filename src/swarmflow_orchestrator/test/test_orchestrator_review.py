"""Review-fix tests for the orchestrator adapter (T007): sim-time recovery, cancel hold, stale states, watchdog,
duplicate orders, policy validation, epoch."""

from __future__ import annotations

import pathlib
import threading
import time

import pytest

rclpy = pytest.importorskip("rclpy")
from rclpy.action import CancelResponse, GoalResponse  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from rosgraph_msgs.msg import Clock  # noqa: E402

from swarmflow_interfaces.action import DispatchTask  # noqa: E402
from swarmflow_interfaces.msg import Order, RobotState  # noqa: E402
from swarmflow_interfaces.srv import RequestReservation  # noqa: E402
from swarmflow_orchestrator.orchestrator_node import OrchestratorNode  # noqa: E402
from test_orchestrator_extra import _Harness, ros, wait  # noqa: E402,F401

FIX = pathlib.Path(__file__).resolve().parents[3] / "tests" / "fixtures" / "standard"


def _node(**params):
    base = {"layout_dir": str(FIX), "robots": ["robot_1"], "policy": "fcfs", "run_dir": ""}
    base.update(params)
    return OrchestratorNode(parameter_overrides=[Parameter(k, value=v) for k, v in base.items()])


def test_policy_validated_and_epoch_is_wall_identity(ros):
    with pytest.raises(ValueError):
        _node(policy="predictive")
    n = _node()
    try:
        ns = int(n.epoch.split(":")[1].split("-")[0])
        assert n.epoch.startswith("epoch:") and abs(ns - time.time_ns()) < 60e9
    finally:
        n.destroy_node()


def test_duplicate_order_and_stale_state_ignored(ros):
    n = _node()
    try:
        o = Order(order_id="o1", pickup_vertex="L1", dropoff_vertex="D1")
        n._on_order(o)
        n._on_order(Order(order_id="o1", pickup_vertex="L2", dropoff_vertex="D2"))
        assert n._specs["o1"].pickup_vertex == "L1"

        def st(x, sec):
            s = RobotState(robot_id="robot_1", x=x, y=0.0, mode=RobotState.MODE_IDLE)
            s.header.stamp.sec = sec
            return s
        n._on_robot_state(st(1.0, 100))
        n._on_robot_state(st(2.0, 99))   # older: dropped
        assert n._snapshots["robot_1"].x == 1.0
        n._on_robot_state(st(3.0, 100))  # equal stamp: accepted
        assert n._snapshots["robot_1"].x == 3.0
    finally:
        n.destroy_node()


def test_recovery_waits_for_robots_with_sim_clock_at_500(ros):
    orch = _node(use_sim_time=True)
    t = rclpy.create_node("orch_review_clock")
    clock_pub = t.create_publisher(Clock, "/clock", 10)
    cli = t.create_client(RequestReservation, "/fleet/request_reservation")
    state_pub = t.create_publisher(RobotState, "/fleet/robot_states", 10)
    wall0 = time.time()
    publish_state = [False]

    def sim_now():
        return 500.0 + (time.time() - wall0)

    def tick():
        now = sim_now()
        c = Clock()
        c.clock.sec, c.clock.nanosec = int(now), int((now % 1) * 1e9)
        clock_pub.publish(c)
        if publish_state[0]:
            s = RobotState(robot_id="robot_1", x=1.3, y=1.1, mode=RobotState.MODE_IDLE, last_vertex="P1")
            s.header.stamp = c.clock
            state_pub.publish(s)

    t.create_timer(0.05, tick)
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(orch)
    ex.add_node(t)
    threading.Thread(target=ex.spin, daemon=True).start()

    def ask(rid):
        fut = cli.call_async(RequestReservation.Request(request_id=rid, robot_id="robot_1", zone_id="Z_aisle_1",
                                                        entry_vertex="X_1_1", exit_vertex="X_1_3"))
        assert wait(fut.done, 5.0)
        return fut.result()

    try:
        assert cli.wait_for_service(timeout_sec=10.0)
        assert wait(lambda: orch._now() > 400.0, 5.0)
        time.sleep(4.0)  # more than RECOVERY_GRACE_S of sim time, but no robot has reported
        r = ask("r1")
        assert r.result == RequestReservation.Response.RESULT_DENIED and r.reason == "RECOVERING"
        publish_state[0] = True
        assert wait(lambda: ask("r2").result == RequestReservation.Response.RESULT_GRANTED, 10.0)
    finally:
        ex.shutdown()
        orch.destroy_node()
        t.destroy_node()


def test_dispatch_held_until_cancelled_goal_finishes(ros):
    state = {"busy": False}

    def kw(h):
        def goal_cb(goal):
            if state["busy"]:  # the agent frees its goal slot only after the previous goal finished
                h.goals.append(("REJECTED", goal))
                return GoalResponse.REJECT
            state["busy"] = True
            return GoalResponse.ACCEPT

        def execute(gh):
            h.goals.append(("RUN", gh.request))
            try:
                while not gh.is_cancel_requested:
                    if gh.request.order.order_id:
                        gh.succeed()
                        return DispatchTask.Result(success=True)
                    time.sleep(0.05)
                time.sleep(0.6)  # slow to wind down
                h.cancelled.append(gh.request.task_id)
                gh.canceled()
                return DispatchTask.Result(success=False, failure_reason="CANCELLED")
            finally:
                state["busy"] = False
        return dict(execute_callback=execute, goal_callback=goal_cb, cancel_callback=lambda _g: CancelResponse.ACCEPT)

    h = _Harness("X_1_1", kw)
    try:
        assert wait(lambda: any(k == "RUN" and not g.order.order_id for k, g in h.goals), 15.0), "no park task"
        h.order_pub.publish(Order(order_id="o1", pickup_vertex="L1", dropoff_vertex="D1", payload_type="small"))
        assert wait(lambda: any(k == "RUN" and g.order.order_id == "o1" for k, g in h.goals), 15.0)
        assert not [1 for k, _g in h.goals if k == "REJECTED"]
        assert not any(d.decision_type == "REASSIGN" for d in h.decisions)
    finally:
        h.close()


def test_agent_lost_watchdog_fails_task_once(ros):
    stop = threading.Event()

    def kw(h):
        def execute(gh):
            h.goals.append(gh.request)
            while not gh.is_cancel_requested and not stop.is_set():
                time.sleep(0.05)
            gh.canceled()
            return DispatchTask.Result()
        return dict(execute_callback=execute, cancel_callback=lambda _g: CancelResponse.ACCEPT)

    h = _Harness("P1", kw)
    try:
        time.sleep(0.5)
        h.order_pub.publish(Order(order_id="o1", pickup_vertex="L1", dropoff_vertex="D1", payload_type="small"))
        assert wait(lambda: any(g.order.order_id == "o1" for g in h.goals), 10.0)
        h.publish = False  # robot goes silent
        assert wait(lambda: any(d.decision_type == "REASSIGN" and d.trigger == "task_failed:AGENT_LOST"
                                for d in h.decisions), 20.0)
    finally:
        stop.set()
        h.close()
