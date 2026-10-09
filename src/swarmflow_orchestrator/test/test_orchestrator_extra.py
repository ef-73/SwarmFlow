"""Extra orchestrator adapter tests: run-record format, goal rejection and park-task cancellation (T007)."""

from __future__ import annotations

import json
import pathlib
import threading
import time

import pytest

rclpy = pytest.importorskip("rclpy")
from rclpy.action import ActionServer, CancelResponse, GoalResponse  # noqa: E402
from rclpy.callback_groups import ReentrantCallbackGroup  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402

from swarmflow_core import api  # noqa: E402
from swarmflow_core.decisions import make_decision  # noqa: E402
from swarmflow_core.graph import load_layout  # noqa: E402
from swarmflow_interfaces.action import DispatchTask  # noqa: E402
from swarmflow_interfaces.msg import DecisionEvent, Order, OrderStatus, RobotState  # noqa: E402
from swarmflow_orchestrator.orchestrator_node import OrchestratorNode, RunRecord  # noqa: E402

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


def test_run_record_format(tmp_path):
    rec = RunRecord(str(tmp_path / "sub" / "run"))
    d = make_decision(t=1.5, event_id="e1", policy="fcfs", decision_type=api.DecisionType.ASSIGN, robot_id="robot_1",
                      order_id="o1", cost_keys=("a",), cost_values=(2.0,))
    import dataclasses
    rec.write("decisions", dataclasses.asdict(d))
    rec.write("orders", {"t": 1.0, "order_id": "o1", "state": api.OrderState.QUEUED})
    rec.close()
    run = tmp_path / "sub" / "run"
    for name in RunRecord.FILES:
        assert (run / f"{name}.jsonl").exists()
    raw = (run / "decisions.jsonl").read_text()
    assert raw.endswith("\n") and "\r" not in raw and len(raw.splitlines()) == 1
    obj = json.loads(raw)
    assert obj["decision_type"] == "ASSIGN" and obj["cost_keys"] == ["a"] and obj["explanation"]
    assert list(obj) == sorted(obj)  # sort_keys
    assert json.loads((run / "orders.jsonl").read_text())["state"] == "QUEUED"
    RunRecord("").write("orders", {"x": 1})  # disabled record: no-op, no error


class _Harness:
    def __init__(self, start_vertex, server_kwargs, run_dir=""):
        g = load_layout(str(FIX))
        v = g.vertices[start_vertex]
        self.pose = (v.x, v.y)
        self.start_vertex = start_vertex
        self.orch = OrchestratorNode(parameter_overrides=[
            Parameter("layout_dir", value=str(FIX)), Parameter("robots", value=["robot_1"]),
            Parameter("policy", value="fcfs"), Parameter("run_dir", value=run_dir)])
        self.t = rclpy.create_node("orch_extra")
        cb = ReentrantCallbackGroup()
        self.goals, self.cancelled, self.statuses, self.decisions = [], [], [], []
        self.server = ActionServer(self.t, DispatchTask, "/robot_1/dispatch_task", callback_group=cb,
                                   **server_kwargs(self))
        latched = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.t.create_subscription(OrderStatus, "/fleet/order_status", self.statuses.append, latched)
        self.t.create_subscription(DecisionEvent, "/fleet/decisions", self.decisions.append, 100)
        self.state_pub = self.t.create_publisher(RobotState, "/fleet/robot_states", 10)
        self.order_pub = self.t.create_publisher(Order, "/fleet/orders", 10)
        self.t.create_timer(0.2, self._publish_state, callback_group=cb)
        self.ex = MultiThreadedExecutor(num_threads=6)
        self.ex.add_node(self.orch)
        self.ex.add_node(self.t)
        threading.Thread(target=self.ex.spin, daemon=True).start()

    publish = True

    def _publish_state(self):
        if not self.publish:
            return
        s = RobotState(robot_id="robot_1", x=self.pose[0], y=self.pose[1], mode=RobotState.MODE_IDLE,
                       last_vertex=self.start_vertex)
        s.header.stamp = self.t.get_clock().now().to_msg()
        self.state_pub.publish(s)

    def close(self):
        self.ex.shutdown()
        self.orch.destroy_node()
        self.t.destroy_node()


def test_rejected_goal_requeues_order(ros):
    def kw(h):
        return dict(execute_callback=lambda gh: DispatchTask.Result(),
                    goal_callback=lambda goal: (h.goals.append(goal), GoalResponse.REJECT)[1])
    h = _Harness("P1", kw)
    try:
        time.sleep(0.5)
        h.order_pub.publish(Order(order_id="o1", pickup_vertex="L1", dropoff_vertex="D1", payload_type="small"))
        assert wait(lambda: any(d.decision_type == "REASSIGN" for d in h.decisions), 15.0)
        seq = [s.state for s in h.statuses if s.order_id == "o1"]
        assert seq[:3] == [OrderStatus.STATE_QUEUED, OrderStatus.STATE_ASSIGNED, OrderStatus.STATE_QUEUED]
        r = [d for d in h.decisions if d.decision_type == "REASSIGN"][0]
        assert r.trigger == "task_failed:REJECTED" and r.order_id == "o1"
    finally:
        h.close()


def test_order_preempts_park_task_and_cancels_goal(ros):
    def kw(h):
        def execute(gh):
            h.goals.append(gh.request)
            while not gh.is_cancel_requested:
                if gh.request.order.order_id:  # order task: finish
                    gh.succeed()
                    return DispatchTask.Result(success=True)
                time.sleep(0.05)
            h.cancelled.append(gh.request.task_id)
            gh.canceled()
            return DispatchTask.Result(success=False, failure_reason="CANCELLED")
        return dict(execute_callback=execute, cancel_callback=lambda _g: CancelResponse.ACCEPT)
    h = _Harness("X_1_1", kw)
    try:
        assert wait(lambda: h.goals and not h.goals[0].order.order_id, 15.0), "no park task"
        park_id = h.goals[0].task_id
        h.order_pub.publish(Order(order_id="o1", pickup_vertex="L1", dropoff_vertex="D1", payload_type="small"))
        assert wait(lambda: h.cancelled == [park_id], 10.0), "park goal not cancelled"
        assert wait(lambda: any(g.order.order_id == "o1" for g in h.goals), 10.0)
    finally:
        h.close()
