"""2D kinematic backend skeleton (design §6.7, §14.2 WS-C v1 acceptance: "skeleton runs the FCFS policy on the
standard layout from sim2d.json deterministically"). Lead test: do not edit."""

from __future__ import annotations

import pathlib
import sys

import pytest
import yaml

ROOT = pathlib.Path(__file__).resolve().parents[2]
for p in (ROOT / "sim2d", ROOT / "src" / "swarmflow_robot_agent"):
    sys.path.insert(0, str(p))

from sim2d import Sim2DBackend, run  # noqa: E402
from swarmflow_core import api  # noqa: E402
from swarmflow_core.api import OrderSpec, OrderState  # noqa: E402

LAYOUT = ROOT / "layouts" / "standard" / "generated" / "sim2d.json"


def orders():
    data = yaml.safe_load((ROOT / "tests" / "fixtures" / "orders_standard.yaml").read_text(encoding="utf-8"))
    return [OrderSpec(**o) for o in data["orders"]]


def test_backend_protocol():
    b = Sim2DBackend.from_json(LAYOUT, robots=["robot_1", "robot_2", "robot_3"])
    snap = b.snapshot()
    assert sorted(r.robot_id for r in snap.robots) == ["robot_1", "robot_2", "robot_3"]
    assert {r.last_vertex for r in snap.robots} == {"P1", "P2", "P3"}
    assert all(r.mode == api.RobotMode.IDLE for r in snap.robots)
    assert b.poll_results() == []


@pytest.mark.parametrize("policy", ["fcfs", "independent"])
def test_fcfs_run_delivers_all(policy):
    r = run(LAYOUT, orders(), policy=policy, robots=["robot_1", "robot_2", "robot_3"], until_t=1800.0, dt=0.1, seed=1)
    assert len(r.order_states) == 10
    assert all(s == OrderState.DELIVERED for s in r.order_states.values()), r.order_states
    if policy == "fcfs":
        assert r.max_robots_in_zone <= 1
    assert 0.0 < r.finish_t < 1800.0


def test_deterministic():
    a = run(LAYOUT, orders(), policy="fcfs", robots=["robot_1", "robot_2", "robot_3"], until_t=1200.0, dt=0.1, seed=7)
    b = run(LAYOUT, orders(), policy="fcfs", robots=["robot_1", "robot_2", "robot_3"], until_t=1200.0, dt=0.1, seed=7)
    assert a.finish_t == b.finish_t
    assert a.decisions == b.decisions
    assert a.trajectory_digest == b.trajectory_digest


def test_uses_real_agent_logic():
    """The robots are driven by swarmflow_robot_agent.agent_core.AgentCore (same protocol client as Gazebo)."""
    b = Sim2DBackend.from_json(LAYOUT, robots=["robot_1"])
    from swarmflow_robot_agent.agent_core import AgentCore
    assert isinstance(b.agent("robot_1"), AgentCore)
