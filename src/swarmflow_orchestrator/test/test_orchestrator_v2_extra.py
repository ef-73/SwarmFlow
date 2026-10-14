"""T030 extra: v2 order/goal conversion and policy selection (no executor needed)."""

from __future__ import annotations

import pathlib

import pytest

rclpy = pytest.importorskip("rclpy")
from rclpy.parameter import Parameter  # noqa: E402

from swarmflow_core import api  # noqa: E402
from swarmflow_core.fleet import FleetCore  # noqa: E402
from swarmflow_core.fleet_v2 import FleetCoreV2  # noqa: E402
from swarmflow_interfaces.action import DispatchTask  # noqa: E402
from swarmflow_interfaces.msg import Order, RobotState  # noqa: E402
from swarmflow_orchestrator.orchestrator_node import OrchestratorNode  # noqa: E402

ROOT = pathlib.Path(__file__).resolve().parents[3] / "layouts"


@pytest.fixture(scope="module")
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


def _node(layout, policy, tmp_path):
    return OrchestratorNode(parameter_overrides=[
        Parameter("layout_dir", value=str(ROOT / layout / "generated")), Parameter("robots", value=["robot_1"]),
        Parameter("policy", value=policy), Parameter("run_dir", value=str(tmp_path))])


def test_policy_selects_core(ros, tmp_path):
    n = _node("warehouse_v2", "assign", tmp_path / "a")
    try:
        assert isinstance(n.core, FleetCoreV2)
    finally:
        n.destroy_node()
    n = _node("warehouse_v2", "fcfs", tmp_path / "b")
    try:
        assert isinstance(n.core, FleetCore)
    finally:
        n.destroy_node()


def test_order_fields_and_goal_conversion(ros, tmp_path):
    n = _node("warehouse_v2", "assign", tmp_path)
    try:
        n._on_order(Order(order_id="s1", dropoff_vertex="DK2", order_type=Order.TYPE_SHELF,
                          pick_vertices=["PK_W1_S_01", "PK_W1_N_02"], pallet_id="p"))
        spec = n._specs["s1"]
        assert spec.order_type == api.ORDER_SHELF and spec.pick_vertices == ("PK_W1_S_01", "PK_W1_N_02")
        a = api.Assignment(task_id="T1", robot_id="robot_1", order_id="s1", route=("PK_W1_S_01", "DK2"),
                           route_kind=api.ROUTE_GOALS) if "route_kind" in api.Assignment.__dataclass_fields__ else None
        assert a is not None
        g = n._goal_of(a)
        assert g.route_kind == DispatchTask.Goal.ROUTE_GOALS
        assert g.order.order_type == Order.TYPE_SHELF and list(g.order.pick_vertices) == list(spec.pick_vertices)
        assert g.order.pallet_id == "p"
    finally:
        n.destroy_node()


def test_unknown_order_type_rejected(ros, tmp_path):
    n = _node("warehouse_v2", "assign", tmp_path)
    try:
        n._on_order(Order(order_id="x", dropoff_vertex="DK2", order_type=9))
        import json
        recs = [json.loads(x) for x in (tmp_path / "orders.jsonl").read_text().splitlines()]
        assert recs[-1]["state"] == "FAILED" and recs[-1]["failure_reason"].startswith("UNSUPPORTED_ORDER")
    finally:
        n.destroy_node()


def test_robot_state_angular_speed(ros, tmp_path):
    n = _node("warehouse_v2", "assign", tmp_path)
    try:
        m = RobotState(robot_id="robot_1", angular_speed=0.5, last_vertex="HM1")
        n._on_robot_state(m)
        assert n._snapshots["robot_1"].angular_speed == 0.5
    finally:
        n.destroy_node()
