"""FCFS policy, fleet core and fake-backend end-to-end tests (design §6.3, §6.7, §10, §13.5, §14.2 WS-B).
Lead test: do not edit."""

from __future__ import annotations

import pathlib

import pytest
import yaml

from swarmflow_core import api
from swarmflow_core.api import DecisionType, OrderSpec, OrderState, RobotMode, RobotSnapshot
from swarmflow_core.backends.fake import FakeBackend, run_fake
from swarmflow_core.decisions import render
from swarmflow_core.fleet import FleetCore
from swarmflow_core.graph import load_layout
from swarmflow_core.policies.fcfs import FcfsPolicy
from swarmflow_core.reservations import FcfsReservationAuthority

FIXTURES = pathlib.Path(__file__).resolve().parents[3] / "tests" / "fixtures"
GRAPH = load_layout(FIXTURES / "standard")


def snap(rid, vertex, mode=RobotMode.IDLE, **kw):
    v = GRAPH.vertices[vertex]
    return RobotSnapshot(robot_id=rid, x=v.x, y=v.y, yaw=0.0, mode=mode, last_vertex=vertex, **kw)


def order(oid, p, d, t=0.0):
    return OrderSpec(order_id=oid, release_t=t, deadline_t=None, pickup_vertex=p, dropoff_vertex=d)


def fixture_orders():
    data = yaml.safe_load((FIXTURES / "orders_standard.yaml").read_text(encoding="utf-8"))
    return [OrderSpec(**o) for o in data["orders"]]


# ---- FCFS policy ----------------------------------------------------------------------------------------------

def test_fcfs_nearest_idle_robot_by_route_distance():
    pol = FcfsPolicy()
    assert pol.name == api.POLICY_FCFS
    snapshot = api.FleetSnapshot(t=0.0, robots=[snap("robot_1", "P1"), snap("robot_2", "P2"), snap("robot_3", "P3")],
                                 open_orders=[order("o1", "L3", "D3")])
    res = pol.plan(snapshot, GRAPH)
    assert len(res.assignments) == 1
    a = res.assignments[0]
    assert (a.robot_id, a.order_id) == ("robot_2", "o1")  # P2 is closest to L3
    assert list(a.route) == list(GRAPH.shortest_path("P2", "L3")) + list(GRAPH.shortest_path("L3", "D3"))[1:]
    assert "H_3_1" in a.route  # holds included before the zone entry
    d = res.decisions[0]
    assert d.decision_type == DecisionType.ASSIGN and d.robot_id == "robot_2" and d.order_id == "o1"
    assert d.policy == "fcfs" and d.cost_keys and len(d.cost_keys) == len(d.cost_values)
    assert d.explanation == render(d) and "robot_2" in d.explanation and "o1" in d.explanation


def test_fcfs_fifo_skips_busy_and_ties_by_robot_id():
    pol = FcfsPolicy()
    robots = [snap("robot_2", "P1"), snap("robot_1", "P1"),
              snap("robot_3", "P3", mode=RobotMode.NAVIGATING, task_id="t9", order_id="o9")]
    orders = [order("oB", "L1", "D1", 1.0), order("oA", "L2", "D2", 0.0), order("oC", "L3", "D3", 2.0)]
    res = pol.plan(api.FleetSnapshot(t=5.0, robots=robots, open_orders=orders), GRAPH)
    pairs = [(a.order_id, a.robot_id) for a in res.assignments]
    # oldest release first (oA), tie between robot_1/robot_2 at P1 → robot_1; then oB → robot_2; oC waits
    assert pairs == [("oA", "robot_1"), ("oB", "robot_2")]
    ids = {a.task_id for a in res.assignments}
    assert len(ids) == 2


def test_independent_policy_same_assignment_different_name():
    s = api.FleetSnapshot(t=0.0, robots=[snap("robot_1", "P1")], open_orders=[order("o1", "L1", "D1")])
    a = FcfsPolicy().plan(s, GRAPH)
    b = FcfsPolicy(name=api.POLICY_INDEPENDENT).plan(s, GRAPH)
    assert [x.route for x in a.assignments] == [x.route for x in b.assignments]
    assert b.decisions[0].policy == "independent"


def test_policy_deterministic():
    s = api.FleetSnapshot(t=0.0, robots=[snap("robot_1", "P1"), snap("robot_2", "P3")],
                          open_orders=[order("o1", "L2", "D2"), order("o2", "L1", "D3")])
    assert FcfsPolicy().plan(s, GRAPH) == FcfsPolicy().plan(s, GRAPH)


# ---- Fleet core lifecycle --------------------------------------------------------------------------------------

def _core(**kw):
    return FleetCore(GRAPH, FcfsPolicy(), FcfsReservationAuthority(GRAPH), **kw)


def test_order_lifecycle_to_delivered():
    core = _core()
    core.update_robot(snap("robot_1", "P1"))
    core.add_order(order("o1", "L1", "D1", t=1.0))
    out = core.tick(0.5)
    assert out.dispatches == [] and core.order_state("o1") is None  # not released yet
    out = core.tick(1.0)
    assert len(out.dispatches) == 1 and out.dispatches[0].order_id == "o1"
    task = out.dispatches[0].task_id
    assert [(c.order_id, c.state) for c in out.status_changes] == [("o1", OrderState.QUEUED), ("o1", OrderState.ASSIGNED)]
    assert core.order_state("o1") == OrderState.ASSIGNED
    core.update_robot(snap("robot_1", "L1", RobotMode.LOADING, task_id=task, order_id="o1"))
    assert [c.state for c in core.tick(10.0).status_changes] == [OrderState.PICKING_UP]
    core.update_robot(snap("robot_1", "X_1_0", RobotMode.NAVIGATING, task_id=task, order_id="o1", payload_type="small"))
    assert [c.state for c in core.tick(14.0).status_changes] == [OrderState.IN_TRANSIT]
    core.task_result(task, True, "", 40.0)
    out = core.tick(40.0)
    assert [c.state for c in out.status_changes] == [OrderState.DELIVERED]
    assert core.order_state("o1") == OrderState.DELIVERED


def test_stuck_timeout_fails_order_and_parks_robot():
    core = _core()
    core.update_robot(snap("robot_1", "P1"))
    core.update_robot(snap("robot_2", "P2", RobotMode.NAVIGATING, task_id="other", order_id=""))
    core.add_order(order("o1", "L1", "D3"))
    task = core.tick(0.0).dispatches[0].task_id
    core.update_robot(snap("robot_1", "X_3_1", RobotMode.STUCK, task_id=task, order_id="o1"))
    core.task_result(task, False, api.FAILURE_STUCK_TIMEOUT, 70.0)
    out = core.tick(70.0)
    fails = [c for c in out.status_changes if c.state == OrderState.FAILED]
    assert fails and fails[0].failure_reason == api.FAILURE_STUCK_TIMEOUT
    assert any(d.decision_type == DecisionType.STUCK_FAIL for d in out.decisions)
    parks = [a for a in out.dispatches if a.robot_id == "robot_1"]
    assert len(parks) == 1 and parks[0].order_id == "" and parks[0].route[-1] in GRAPH.vertices_of_kind("park")
    assert parks[0].route[0] == "X_3_1"


def test_nav_failure_before_pickup_reassigns():
    core = _core()
    core.update_robot(snap("robot_1", "P1"))
    core.add_order(order("o1", "L2", "D2"))
    task = core.tick(0.0).dispatches[0].task_id
    core.update_robot(snap("robot_2", "P2"))
    core.update_robot(snap("robot_1", "X_1_0", RobotMode.STUCK, task_id=task, order_id="o1"))
    core.task_result(task, False, "NAV_FAILED:X_2_0", 20.0)
    out = core.tick(20.0)
    assert any(d.decision_type == DecisionType.REASSIGN for d in out.decisions)
    reassigned = [a for a in out.dispatches if a.order_id == "o1"]
    assert reassigned and reassigned[0].robot_id == "robot_2"
    assert core.order_state("o1") == OrderState.ASSIGNED


def test_nav_failure_after_pickup_fails_order():
    core = _core()
    core.update_robot(snap("robot_1", "P1"))
    core.add_order(order("o1", "L1", "D1"))
    task = core.tick(0.0).dispatches[0].task_id
    core.update_robot(snap("robot_1", "L1", RobotMode.LOADING, task_id=task, order_id="o1"))
    core.tick(5.0)
    core.update_robot(snap("robot_1", "X_1_0", RobotMode.NAVIGATING, task_id=task, order_id="o1", payload_type="small"))
    core.tick(9.0)
    core.task_result(task, False, "NAV_FAILED:H_1_1", 30.0)
    out = core.tick(30.0)
    f = [c for c in out.status_changes if c.state == OrderState.FAILED]
    assert f and f[0].failure_reason == api.FAILURE_STUCK_AFTER_PICKUP


# ---- Fake backend end-to-end (WS-B acceptance: 10 orders × 3 robots) -----------------------------------------

@pytest.mark.parametrize("traffic_control", [True, False])
def test_fake_backend_ten_orders_three_robots(traffic_control):
    authority = FcfsReservationAuthority(GRAPH) if traffic_control else None
    policy = FcfsPolicy() if traffic_control else FcfsPolicy(name=api.POLICY_INDEPENDENT)
    core = FleetCore(GRAPH, policy, authority, traffic_control=traffic_control)
    backend = FakeBackend(GRAPH, {"robot_1": "P1", "robot_2": "P2", "robot_3": "P3"}, authority=authority,
                          speed_mps=0.5)
    result = run_fake(core, backend, fixture_orders(), until_t=1800.0, dt=0.1)
    states = {o: core.order_state(o) for o in result.order_ids}
    assert len(states) == 10
    assert all(s == OrderState.DELIVERED for s in states.values()), states
    if traffic_control:
        # safety: never two robots inside one zone polygon at the same sample time
        assert result.max_robots_in_zone == 1
        assert any(d.decision_type == DecisionType.ASSIGN for d in result.decisions)


def test_fake_run_deterministic():
    def once():
        auth = FcfsReservationAuthority(GRAPH)
        core = FleetCore(GRAPH, FcfsPolicy(), auth)
        backend = FakeBackend(GRAPH, {"robot_1": "P1", "robot_2": "P2", "robot_3": "P3"}, authority=auth)
        r = run_fake(core, backend, fixture_orders()[:5], until_t=900.0, dt=0.1)
        return [(round(d.t, 3), d.decision_type, d.robot_id, d.order_id) for d in r.decisions], r.finish_t
    assert once() == once()


# ---- idle robots return to park; park tasks are preemptible -------------------------------------------------

def test_idle_robot_off_park_is_sent_to_park():
    core = _core()
    core.update_robot(snap("robot_1", "D2"))           # idle on a delivery station, nothing to do
    out = core.tick(0.0)
    parks = [a for a in out.dispatches if a.robot_id == "robot_1"]
    assert len(parks) == 1 and parks[0].order_id == "" and parks[0].route[0] == "D2"
    assert parks[0].route[-1] in ("P3", "P4")          # nearest park by route distance (east side)
    assert core.tick(0.5).dispatches == []             # not re-sent while that task runs
    core.update_robot(snap("robot_2", "P1"))
    assert [a for a in core.tick(1.0).dispatches if a.robot_id == "robot_2"] == []   # already parked


def test_parking_robot_is_preempted_for_an_order():
    core = _core()
    core.update_robot(snap("robot_1", "D1"))
    park = core.tick(0.0).dispatches[0]
    core.update_robot(snap("robot_1", "X_1_4", RobotMode.NAVIGATING, task_id=park.task_id))
    core.add_order(order("o1", "L2", "D2", t=1.0))
    out = core.tick(1.0)
    assert out.cancels == [park.task_id]
    a = [x for x in out.dispatches if x.order_id == "o1"]
    assert a and a[0].robot_id == "robot_1" and a[0].route[0] == "X_1_4"


def test_two_parking_robots_get_different_parks():
    core = _core()
    core.update_robot(snap("robot_1", "D1"))
    core.update_robot(snap("robot_2", "D2"))
    out = core.tick(0.0)
    ends = sorted(a.route[-1] for a in out.dispatches)
    assert len(ends) == 2 and ends[0] != ends[1]
