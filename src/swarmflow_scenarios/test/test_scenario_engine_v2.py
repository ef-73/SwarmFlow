"""T027: the scenario engine publishes v2 order fields (order_type, pick_vertices, pallet_id)."""

from __future__ import annotations

import json
import pathlib
import time

import pytest

rclpy = pytest.importorskip("rclpy")
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from rclpy.qos import QoSProfile, ReliabilityPolicy  # noqa: E402

from swarmflow_core import api  # noqa: E402
from swarmflow_interfaces.msg import Order  # noqa: E402
from swarmflow_scenarios.scenario_engine import ScenarioEngineNode  # noqa: E402

ROOT = pathlib.Path(__file__).resolve().parents[3]
LAYOUT = ROOT / "layouts" / "warehouse_v2" / "generated"
TOOLS = ROOT / "tools" / "scenarios"

SCENARIO = """name: tiny_v2
seed: 4
duration_s: 600
layout: warehouse_v2
policy: assign
robots: [robot_1]
orders:
  mode: v2_rate
  docks: [DK2]
  shelf: {rate_per_min: 30.0, picks: [2, 2], slots: all, popularity: uniform}
  pallet: {rate_per_min: 30.0}
"""


@pytest.fixture(scope="module")
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


def test_v2_fields_published(ros, tmp_path):
    sf = tmp_path / "tiny_v2.yaml"
    sf.write_text(SCENARIO, encoding="utf-8")
    run_dir = tmp_path / "run"
    node = ScenarioEngineNode(parameter_overrides=[
        Parameter("scenario_file", value=str(sf)), Parameter("layout_dir", value=str(LAYOUT)),
        Parameter("run_dir", value=str(run_dir)), Parameter("start_delay_s", value=0.0),
        Parameter("scenarios_tools", value=str(TOOLS)),
    ])
    sink = rclpy.create_node("order_sink_v2")
    got = []
    sink.create_subscription(Order, "/fleet/orders", got.append,
                             QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE))
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    ex.add_node(sink)
    deadline = time.monotonic() + 15.0
    while not ({m.order_type for m in got} >= {Order.TYPE_SHELF, Order.TYPE_PALLET}) and time.monotonic() < deadline:
        ex.spin_once(timeout_sec=0.05)
    ex.remove_node(node)
    ex.remove_node(sink)
    node.destroy_node()
    sink.destroy_node()

    shelf = [m for m in got if m.order_type == Order.TYPE_SHELF]
    pallet = [m for m in got if m.order_type == Order.TYPE_PALLET]
    assert shelf and pallet
    assert all(len(m.pick_vertices) == 2 and m.pickup_vertex == "" and m.dropoff_vertex == "DK2" for m in shelf)
    assert all(m.pallet_id == f"pallet_{m.pickup_vertex}" and m.payload_type == "pallet" for m in pallet)
    ev = [json.loads(x) for x in (run_dir / "events.jsonl").read_text(encoding="utf-8").splitlines()]
    first = {e["order_id"]: e for e in ev}
    m = shelf[0]
    assert first[m.order_id]["order_type"] == api.ORDER_SHELF
    assert first[m.order_id]["pick_vertices"] == list(m.pick_vertices)
