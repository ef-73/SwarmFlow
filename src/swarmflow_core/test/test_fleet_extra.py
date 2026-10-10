"""Extra tests for the fleet core, decision rendering and fake backend (T004)."""

from __future__ import annotations

import pathlib

import pytest
import yaml

from swarmflow_core import api
from swarmflow_core.api import DecisionType, OrderSpec, OrderState, RobotMode, RobotSnapshot
from swarmflow_core.backends.fake import FakeBackend, run_fake
from swarmflow_core.decisions import make_decision, render, reservation_decision
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


def _core():
    return FleetCore(GRAPH, FcfsPolicy(), FcfsReservationAuthority(GRAPH))


@pytest.mark.parametrize("dtype", list(DecisionType))
def test_render_every_decision_type(dtype):
    d = make_decision(t=1.0, event_id="e", policy="fcfs", decision_type=dtype, robot_id="robot_1", order_id="o1",
                      trigger="trig", previous_decision="prev", new_decision="new",
                      cost_keys=("a",), cost_values=(1.5,))
    assert d.explanation == render(d) and ("robot_1" in d.explanation or "o1" in d.explanation)
    assert d.explanation.endswith(".")
    assert render(d) == render(d)


def test_render_assign_template():
    d = make_decision(t=0.0, event_id="e", policy="fcfs", decision_type=DecisionType.ASSIGN, robot_id="robot_2",
                      order_id="o003", cost_keys=("route_to_pickup_m", "route_total_m"), cost_values=(7.4, 24.9))
    assert d.explanation == ("robot_2 assigned order o003: nearest available robot, 7.4 m to pickup "
                             "(route 24.9 m).")


def test_reservation_decision_grant_and_deny():
    req = api.ReservationRequest("r1", "robot_1", "Z_aisle_1", "X_1_1", "X_1_3")
    g = reservation_decision(2.0, "fcfs", req, api.ReservationDecision(True, "lease_00001", 7.0))
    assert g.decision_type == DecisionType.RESERVATION_GRANT and g.new_decision == "lease_00001"
    assert g.trigger == "request:r1" and g.explanation == render(g)
    n = reservation_decision(2.0, "fcfs", req, api.ReservationDecision(False, "", 0.0, api.DENY_ZONE_LEASED, 1.0))
    assert n.decision_type == DecisionType.RESERVATION_DENY and n.new_decision == api.DENY_ZONE_LEASED
    assert "ZONE_LEASED" in n.explanation


def test_second_attempt_gets_suffixed_task_id():
    core = _core()
    core.update_robot(snap("robot_1", "P1"))
    core.add_order(order("o1", "L1", "D1"))
    t1 = core.tick(0.0).dispatches[0].task_id
    assert t1 == "task_o1"
    core.update_robot(snap("robot_1", "X_1_0", RobotMode.STUCK, task_id=t1, order_id="o1"))
    core.task_result(t1, False, "NAV_FAILED:H_1_1", 5.0)
    out = core.tick(5.0)
    assert [c.state for c in out.status_changes if c.order_id == "o1"] == [OrderState.QUEUED]
    # robot_1 is STUCK, so the order waits; a second robot picks it up as attempt 2
    core.update_robot(snap("robot_2", "P2"))
    out = core.tick(6.0)
    a = [x for x in out.dispatches if x.order_id == "o1"]
    assert a and a[0].task_id == "task_o1_1" and a[0].robot_id == "robot_2"


def test_stale_result_for_unknown_task_is_ignored():
    core = _core()
    core.update_robot(snap("robot_1", "P1"))
    core.add_order(order("o1", "L1", "D1"))
    core.tick(0.0)
    core.task_result("ghost", True, "", 3.0)
    out = core.tick(3.0)
    assert out.status_changes == [] and core.order_state("o1") == OrderState.ASSIGNED


def test_status_changes_emitted_once():
    core = _core()
    core.update_robot(snap("robot_1", "P1"))
    core.add_order(order("o1", "L1", "D1"))
    task = core.tick(0.0).dispatches[0].task_id
    core.update_robot(snap("robot_1", "L1", RobotMode.LOADING, task_id=task, order_id="o1"))
    assert len(core.tick(1.0).status_changes) == 1
    assert core.tick(2.0).status_changes == []


def test_loaded_snapshot_without_loading_seen_counts_as_picked_up():
    core = _core()
    core.update_robot(snap("robot_1", "P1"))
    core.add_order(order("o1", "L1", "D1"))
    task = core.tick(0.0).dispatches[0].task_id
    core.update_robot(snap("robot_1", "X_1_0", RobotMode.NAVIGATING, task_id=task, order_id="o1", payload_type="small"))
    assert [c.state for c in core.tick(1.0).status_changes] == [OrderState.PICKING_UP, OrderState.IN_TRANSIT]


def test_traffic_control_flag_recorded_and_assignment_unchanged():
    core = FleetCore(GRAPH, FcfsPolicy(name=api.POLICY_INDEPENDENT), None, traffic_control=False)
    assert core.traffic_control is False
    core.update_robot(snap("robot_1", "P1"))
    core.add_order(order("o1", "L1", "D1"))
    out = core.tick(0.0)
    assert out.dispatches[0].robot_id == "robot_1" and out.decisions[0].policy == "independent"


def test_park_not_resent_after_park_task_finishes_off_park():
    core = _core()
    core.update_robot(snap("robot_1", "D1"))
    park = core.tick(0.0).dispatches[0]
    core.task_result(park.task_id, False, "NAV_FAILED:X_1_4", 3.0)
    core.update_robot(snap("robot_1", "X_1_4"))
    assert core.tick(3.0).dispatches == []


def test_stuck_robot_is_not_used_or_parked():
    core = _core()
    core.update_robot(snap("robot_1", "X_1_4", RobotMode.FAULT))
    core.add_order(order("o1", "L1", "D1"))
    out = core.tick(0.0)
    assert out.dispatches == [] and core.order_state("o1") == OrderState.QUEUED


def test_fake_backend_dwells_and_reports_modes():
    auth = FcfsReservationAuthority(GRAPH)
    backend = FakeBackend(GRAPH, {"robot_1": "L1"}, authority=auth, speed_mps=1.0)
    spec = order("o1", "L1", "D1")
    backend.register_orders([spec])
    route = list(GRAPH.shortest_path("L1", "D1"))
    auth.recover(backend.snapshot().robots, 0.0)
    backend.step(0.0)
    backend.dispatch(api.Assignment("robot_1", "o1", "task_o1", tuple(route)))
    modes = []
    t = 0.0
    while t < 120.0 and not backend._results:
        t = round(t + 0.1, 9)
        backend.step(t)
        modes.append(backend.snapshot().robots[0].mode)
    assert RobotMode.LOADING in modes and RobotMode.UNLOADING in modes
    assert modes.index(RobotMode.LOADING) < modes.index(RobotMode.UNLOADING)
    res = backend.poll_results()
    assert [(r.task_id, r.success) for r in res] == [("task_o1", True)]
    assert backend.snapshot().robots[0].mode == RobotMode.IDLE and backend.snapshot().robots[0].task_id == ""


def test_fake_run_orders_from_fixture_finish_and_no_leases_leak():
    data = yaml.safe_load((FIXTURES / "orders_standard.yaml").read_text(encoding="utf-8"))
    orders = [OrderSpec(**o) for o in data["orders"]]
    auth = FcfsReservationAuthority(GRAPH)
    core = FleetCore(GRAPH, FcfsPolicy(), auth)
    backend = FakeBackend(GRAPH, {"robot_1": "P1", "robot_2": "P2", "robot_3": "P3"}, authority=auth)
    res = run_fake(core, backend, orders, until_t=1800.0)
    assert res.finish_t < 1800.0
    assert any(d.decision_type == DecisionType.RESERVATION_GRANT for d in res.decisions)
    assert all(d.explanation for d in res.decisions)
    assert all(l.state != api.LeaseState.GRANTED for l in auth.active_leases())


# ---- T020 review fixes: STUCK recovery only for STUCK/IDLE robots that report no live task -----------------------

def _failed_core():
    from swarmflow_core.fleet import STUCK_RECOVERY_S  # noqa: F401
    c = FleetCore(GRAPH, FcfsPolicy(), FcfsReservationAuthority(GRAPH))
    c.update_robot(snap("robot_1", "P1"))
    c.add_order(order("o1", "L1", "D1"))
    task = c.tick(0.0).dispatches[0].task_id
    c.update_robot(snap("robot_1", "X_1_0", RobotMode.STUCK, task_id=task, order_id="o1"))
    c.task_result(task, False, api.FAILURE_STUCK_TIMEOUT, 60.0)
    park = [a for a in c.tick(60.0).dispatches if a.robot_id == "robot_1"][0]
    c.task_result(park.task_id, False, api.FAILURE_STUCK_TIMEOUT, 130.0)
    c.tick(130.0)
    return c


def _recoveries(c, t):
    return [a for a in c.tick(t).dispatches if a.robot_id == "robot_1"]


def test_fault_robot_gets_no_recovery():
    from swarmflow_core.fleet import STUCK_RECOVERY_S
    c = _failed_core()
    c.update_robot(snap("robot_1", "X_1_0", RobotMode.FAULT, fault_reason="NO_POSE"))
    assert not _recoveries(c, 130.0 + STUCK_RECOVERY_S)
    assert not _recoveries(c, 130.0 + 5 * STUCK_RECOVERY_S)
    c.update_robot(snap("robot_1", "X_1_0", RobotMode.STUCK))
    assert len(_recoveries(c, 130.0 + 6 * STUCK_RECOVERY_S)) == 1


def test_robot_still_reporting_a_task_gets_no_recovery():
    from swarmflow_core.fleet import STUCK_RECOVERY_S
    c = _failed_core()
    c.update_robot(snap("robot_1", "X_1_0", RobotMode.STUCK, task_id="some_live_task"))
    assert not _recoveries(c, 130.0 + STUCK_RECOVERY_S)
    c.update_robot(snap("robot_1", "X_1_0", RobotMode.STUCK))
    assert len(_recoveries(c, 130.0 + 2 * STUCK_RECOVERY_S)) == 1
