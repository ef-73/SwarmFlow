"""Additional AgentCore tests (WS-D) for spec items the lead tests do not pin down."""

from __future__ import annotations

import pathlib

from swarmflow_core import api
from swarmflow_core.api import ReleaseReason, ReservationDecision, RobotMode
from swarmflow_core.graph import load_layout
from swarmflow_robot_agent.agent_core import (AgentCore, CancelNav, Navigate, Release, RequestReservation,
                                              TaskFinished)

FIX = pathlib.Path(__file__).resolve().parents[3] / "tests" / "fixtures" / "standard"
GRAPH = load_layout(FIX)
HOLD_ROUTE = ["H_3_1", "X_3_1", "X_3_3", "X_3_4", "D3"]


def of(actions, kind):
    return [a for a in actions if isinstance(a, kind)]


def agent_at_hold(**kw):
    a = AgentCore("robot_2", GRAPH, **kw)
    v = GRAPH.vertices["H_3_1"]
    a.on_pose(v.x, v.y, 0.0, 0.0, t=0.0)
    a.on_orchestrator_heartbeat(t=0.0)
    return a


def test_request_ids_and_expected_exit():
    a = agent_at_hold()
    req = of(a.start_task("t1", "o1", HOLD_ROUTE, t=2.0), RequestReservation)[0].request
    assert req.request_id == "robot_2-1" and req.earliest_entry_t == 2.0
    assert req.expected_exit_t == 2.0 + 10.0 / 0.5      # 10 m zone path at 0.5 m/s


def test_stale_response_is_ignored():
    a = agent_at_hold()
    a.start_task("t1", "o1", HOLD_ROUTE, t=0.0)
    acts = a.on_reservation_response("robot_2-99", ReservationDecision(True, lease_id="Lx", lease_expiry_t=5.0), t=0.1)
    assert acts == [] and a.state().held_lease_ids == () and a.state().mode == RobotMode.WAITING_RESERVATION


def test_deny_without_retry_after_uses_default():
    a = agent_at_hold()
    a.start_task("t1", "o1", HOLD_ROUTE, t=0.0)
    a.on_reservation_response(a.pending_request_id(), ReservationDecision(False, reason="ZONE_LEASED"), t=0.0)
    assert not of(a.tick(t=0.7), RequestReservation)
    assert of(a.tick(t=1.25), RequestReservation)


def test_failed_service_call_retries():
    a = agent_at_hold()
    a.start_task("t1", "o1", HOLD_ROUTE, t=0.0)
    a.on_reservation_response(a.pending_request_id(), None, t=0.1)
    assert a.pending_request_id() == ""
    assert of(a.tick(t=1.5), RequestReservation) and a.pending_request_id()


def test_timeout_issues_new_request_id():
    a = agent_at_hold()
    a.start_task("t1", "o1", HOLD_ROUTE, t=0.0)
    first = a.pending_request_id()
    a.tick(t=1.0)
    assert a.pending_request_id() == first
    assert of(a.tick(t=2.0), RequestReservation)
    assert a.pending_request_id() not in ("", first)


def test_nav_failure_releases_leases_as_fault():
    a = agent_at_hold()
    a.start_task("t1", "o1", HOLD_ROUTE, t=0.0)
    a.on_reservation_response(a.pending_request_id(), ReservationDecision(True, lease_id="Lf", lease_expiry_t=5.0), t=0.1)
    assert of(a.on_nav_result(False, t=1.0), Navigate)
    acts = a.on_nav_result(False, t=2.0)
    rel = of(acts, Release)
    assert rel and rel[0].reason == ReleaseReason.FAULT
    fin = of(acts, TaskFinished)[0]
    assert fin.failure_reason == api.FAILURE_NAV_PREFIX + "D3" and a.can_accept()
    assert a.state().mode == RobotMode.STUCK and a.state().held_lease_ids == ()


def test_stuck_agent_can_take_new_task():
    a = agent_at_hold()
    a.start_task("t1", "", ["H_3_1", "X_3_1"], t=0.0)
    # route ends at the entry (no zone edge after it), so no hold: the task is a plain move
    a.on_nav_result(False, t=1.0)
    a.on_nav_result(False, t=2.0)
    assert a.can_accept()
    acts = a.start_task("t2", "", ["X_3_1", "X_3_3"], t=3.0)
    assert a.state().task_id == "t2"
    assert not of(acts, Navigate) and of(acts, RequestReservation)     # starts at a zone entry: must request first


def test_cancel_idle_is_noop_and_cancel_while_loading_has_no_cancelnav():
    a = AgentCore("robot_2", GRAPH)
    assert a.cancel(t=0.0) == []
    v = GRAPH.vertices["L1"]
    a.on_pose(v.x, v.y, 0.0, 0.0, t=0.0)
    a.start_task("t1", "o1", ["L1", "X_1_0"], t=0.0, pickup="L1", dropoff="X_1_0")
    assert a.state().mode == RobotMode.LOADING
    acts = a.cancel(t=1.0)
    assert not of(acts, CancelNav) and of(acts, TaskFinished)[0].failure_reason == api.FAILURE_CANCELLED


def test_explicit_pickup_dropoff_and_last_pose_yaw():
    a = AgentCore("robot_2", GRAPH, traffic_control=False)
    v = GRAPH.vertices["P1"]
    a.on_pose(v.x, v.y, 0.0, 0.0, t=0.0)
    route = ["P1", "X_0_0", "X_1_0", "L1"]
    nav = of(a.start_task("t1", "o1", route, t=0.0, pickup="L1", dropoff="L1"), Navigate)[0]
    assert nav.vertices[-1] == "L1"
    assert nav.poses[-1][2] == GRAPH.vertices["L1"].yaw       # station yaw for the last pose of the route


def test_release_after_passing_exit_without_inside_pose():
    a = agent_at_hold()
    a.start_task("t1", "o1", HOLD_ROUTE, t=0.0)
    a.on_reservation_response(a.pending_request_id(), ReservationDecision(True, lease_id="Lq", lease_expiry_t=5.0), t=0.1)
    v = GRAPH.vertices["X_3_4"]
    a.on_pose(v.x, v.y, 0.0, 0.5, t=5.0)      # jumped past the zone exit (sparse poses): X_3_4 is 2.6 m outside
    rel = of(a.tick(t=5.0), Release)
    assert rel and rel[0].lease_id == "Lq" and rel[0].reason == ReleaseReason.EXITED


def test_waiting_agent_reports_zone():
    a = agent_at_hold()
    a.start_task("t1", "o1", HOLD_ROUTE, t=0.0)
    assert a.waiting_zone() == "Z_aisle_3"
