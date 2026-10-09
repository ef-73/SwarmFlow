"""Scenario / order generator tests (design §12.3, §13.5; WS-F "same seed → identical stream"). Lead test.

API under test (tools/scenarios/scenarios/): ``load_scenario(path) -> Scenario`` and
``generate_orders(scenario, graph) -> list[api.OrderSpec]``.
"""

from __future__ import annotations

import dataclasses
import pathlib
import sys

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "tools" / "scenarios"))

from scenarios import generate_orders, load_scenario  # noqa: E402
from swarmflow_core import api  # noqa: E402
from swarmflow_core.api import DecisionType  # noqa: E402
from swarmflow_core.backends.fake import FakeBackend, run_fake  # noqa: E402
from swarmflow_core.fleet import FleetCore  # noqa: E402
from swarmflow_core.graph import load_layout  # noqa: E402
from swarmflow_core.policies.fcfs import FcfsPolicy  # noqa: E402
from swarmflow_core.reservations import FcfsReservationAuthority  # noqa: E402

GRAPH = load_layout(ROOT / "layouts" / "standard" / "generated")
DEMO = ROOT / "scenarios" / "v1_demo.yaml"
CORRIDOR = ROOT / "scenarios" / "v1_corridor.yaml"
SPAWN = {"robot_1": "P1", "robot_2": "P2", "robot_3": "P3"}


def test_demo_fields():
    s = load_scenario(DEMO)
    assert s.name == "v1_demo" and s.layout == "standard" and s.policy in ("fcfs", "independent")
    assert s.seed == 1 and s.duration_s == 600.0
    assert s.robots == ["robot_1", "robot_2", "robot_3"]


def test_same_seed_identical_stream():
    s = load_scenario(DEMO)
    assert generate_orders(s, GRAPH) == generate_orders(s, GRAPH)


def test_different_seed_differs():
    s = load_scenario(DEMO)
    assert generate_orders(s, GRAPH) != generate_orders(dataclasses.replace(s, seed=2), GRAPH)


def test_demo_orders_valid():
    s = load_scenario(DEMO)
    orders = generate_orders(s, GRAPH)
    assert 15 <= len(orders) <= 60          # enough for >= 10 deliveries in 10 min, not a flood
    assert [o.order_id for o in orders] == [f"o{i:04d}" for i in range(1, len(orders) + 1)]
    ts = [o.release_t for o in orders]
    assert ts == sorted(ts) and 0.0 <= ts[0] and ts[-1] < s.duration_s
    for o in orders:
        assert GRAPH.vertices[o.pickup_vertex].kind == "loading"
        assert GRAPH.vertices[o.dropoff_vertex].kind == "delivery"
        assert o.payload_type == "small" and o.deadline_t is None
    assert {o.pickup_vertex for o in orders} == {"L1", "L2", "L3"}
    assert {o.dropoff_vertex for o in orders} == {"D1", "D2", "D3"}


def test_demo_delivers_in_fake_backend():
    s = load_scenario(DEMO)
    auth = FcfsReservationAuthority(GRAPH)
    core = FleetCore(GRAPH, FcfsPolicy(), auth)
    r = run_fake(core, FakeBackend(GRAPH, SPAWN, authority=auth), generate_orders(s, GRAPH), until_t=900.0)
    delivered = sum(core.order_state(o) == api.OrderState.DELIVERED for o in r.order_ids)
    assert delivered >= 10


def test_corridor_makes_robots_meet():
    """Under FCFS reservations robots must be denied at aisles repeatedly; without traffic control two robots end up
    inside one aisle at the same time (design §13.5: opposing entries within 5 s, repeated every 60 s)."""
    s = load_scenario(CORRIDOR)
    assert s.name == "v1_corridor"
    orders = generate_orders(s, GRAPH)
    spawn = {r: SPAWN[r] for r in s.robots}
    auth = FcfsReservationAuthority(GRAPH)
    core = FleetCore(GRAPH, FcfsPolicy(), auth)
    r = run_fake(core, FakeBackend(GRAPH, spawn, authority=auth), orders, until_t=s.duration_s)
    denies = [d for d in r.decisions if d.decision_type == DecisionType.RESERVATION_DENY]
    assert len({round(d.t / 60.0) for d in denies}) >= 3, [round(d.t, 1) for d in denies]
    core_a = FleetCore(GRAPH, FcfsPolicy(name=api.POLICY_INDEPENDENT), None, traffic_control=False)
    r_a = run_fake(core_a, FakeBackend(GRAPH, spawn, authority=None), orders, until_t=s.duration_s)
    assert r_a.max_robots_in_zone >= 2


def test_invalid_scenario_rejected(tmp_path):
    p = tmp_path / "bad.yaml"
    p.write_text("name: bad\nseed: 1\nduration_s: 10\nlayout: standard\npolicy: fcfs\nrobots: [robot_1]\n"
                 "orders: {mode: explicit, list: [{t: 1.0, pickup: D1, dropoff: L1}]}\n", encoding="utf-8")
    with pytest.raises(ValueError):
        generate_orders(load_scenario(p), GRAPH)
