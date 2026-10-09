"""scenario_engine node test (wall clock, no Gazebo): 2 explicit orders arrive in order; run record is written."""

from __future__ import annotations

import json
import pathlib
import time

import pytest

rclpy = pytest.importorskip("rclpy")
import yaml  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from rclpy.qos import QoSProfile, ReliabilityPolicy  # noqa: E402

from swarmflow_interfaces.msg import Order  # noqa: E402
from swarmflow_scenarios.scenario_engine import ScenarioEngineNode  # noqa: E402

ROOT = pathlib.Path(__file__).resolve().parents[3]
LAYOUT = ROOT / "tests" / "fixtures" / "standard"
TOOLS = ROOT / "tools" / "scenarios"

SCENARIO = """name: tiny
seed: 3
duration_s: 30
layout: standard
policy: fcfs
robots: [robot_1]
orders:
  mode: explicit
  list:
    - {t: 0.0, pickup: L1, dropoff: D1}
    - {t: 0.5, pickup: L3, dropoff: D2}
"""


@pytest.fixture(scope="module")
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


def test_orders_published_in_order_and_record_written(ros, tmp_path):
    sf = tmp_path / "tiny.yaml"
    sf.write_text(SCENARIO, encoding="utf-8")
    run_dir = tmp_path / "run"
    node = ScenarioEngineNode(parameter_overrides=[
        Parameter("scenario_file", value=str(sf)),
        Parameter("layout_dir", value=str(LAYOUT)),
        Parameter("run_dir", value=str(run_dir)),
        Parameter("git_sha", value="abc123"),
        Parameter("image_digests", value="dev@sha256:0"),
        Parameter("start_delay_s", value=0.2),
        Parameter("scenarios_tools", value=str(TOOLS)),
    ])
    sink = rclpy.create_node("order_sink")
    got = []
    sink.create_subscription(Order, "/fleet/orders", got.append,
                             QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE))
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    ex.add_node(sink)
    deadline = time.monotonic() + 10.0
    while len(got) < 2 and time.monotonic() < deadline:
        ex.spin_once(timeout_sec=0.05)
    ex.spin_once(timeout_sec=0.05)
    ex.remove_node(node)
    ex.remove_node(sink)
    node.destroy_node()
    sink.destroy_node()

    assert [(o.order_id, o.pickup_vertex, o.dropoff_vertex) for o in got] == [
        ("o0001", "L1", "D1"), ("o0002", "L3", "D2")]
    assert all(o.payload_type == "small" and o.priority == 0 for o in got)
    assert (got[1].release_time.sec + got[1].release_time.nanosec * 1e-9) >= \
        (got[0].release_time.sec + got[0].release_time.nanosec * 1e-9) + 0.4
    cfg = yaml.safe_load((run_dir / "config.yaml").read_text(encoding="utf-8"))
    assert cfg["git_sha"] == "abc123" and cfg["image_digests"] == "dev@sha256:0"
    assert cfg["scenario"]["name"] == "tiny" and cfg["scenario"]["seed"] == 3
    lines = (run_dir / "events.jsonl").read_text(encoding="utf-8").splitlines()
    ev = [json.loads(x) for x in lines]
    assert [e["order_id"] for e in ev] == ["o0001", "o0002"]
