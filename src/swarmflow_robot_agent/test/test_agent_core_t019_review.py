"""T019 review fixes (own tests): entry-vertex lease loss, early lease events, abandoned timed-out requests."""

from __future__ import annotations

import pathlib

from swarmflow_core import api
from swarmflow_core.api import LeaseState, ReleaseReason, ReservationDecision, RobotMode
from swarmflow_core.graph import load_layout
from swarmflow_robot_agent.agent_core import AgentCore, CancelNav, Navigate, Release, RequestReservation

GRAPH = load_layout(pathlib.Path(__file__).resolve().parents[3] / "tests" / "fixtures" / "standard")
ROUTE = ["H_3_1", "X_3_1", "X_3_3", "X_3_4", "D3"]


def of(acts, kind):
    return [a for a in acts if isinstance(a, kind)]


def agent():
    a = AgentCore("robot_1", GRAPH)
    v = GRAPH.vertices["H_3_1"]
    a.on_pose(v.x, v.y, 0.0, 0.0, t=0.0)
    a.on_orchestrator_heartbeat(t=0.0)
    a.start_task("t1", "", ROUTE, t=0.0)
    return a


def grant(a, lid, t=0.1):
    return a.on_reservation_response(a.pending_request_id(), ReservationDecision(True, lease_id=lid, lease_expiry_t=9.0), t=t)


def test_revoked_while_standing_at_entry_vertex_is_not_inside():
    a = agent()
    assert of(grant(a, "L1"), Navigate)
    v = GRAPH.vertices["X_3_1"]                     # entry vertex lies on the zone boundary
    a.on_pose(v.x, v.y, 0.0, 0.0, t=0.5)
    acts = a.on_lease_event("L1", LeaseState.REVOKED, t=0.6)
    assert of(acts, CancelNav) and of(acts, RequestReservation)
    assert a.state().mode == RobotMode.WAITING_RESERVATION and a.state().held_lease_ids == ()


def test_lease_event_before_grant_response_invalidates_grant():
    a = agent()
    rid = a.pending_request_id()
    assert a.on_lease_event("L1", LeaseState.EXPIRED, t=0.05) == []
    acts = a.on_reservation_response(rid, ReservationDecision(True, lease_id="L1", lease_expiry_t=9.0), t=0.1)
    assert not of(acts, Navigate) and of(acts, RequestReservation)
    assert a.state().held_lease_ids == ()
    acts = grant(a, "L2", t=0.2)
    assert of(acts, Navigate) and a.state().held_lease_ids == ("L2",)


def test_timed_out_request_late_grant_is_released():
    a = agent()
    old = a.pending_request_id()
    acts = a.tick(t=api.ORCH_RESPONSE_TIMEOUT_S + 0.1)
    assert of(acts, RequestReservation) and a.pending_request_id() != old
    late = a.on_reservation_response(old, ReservationDecision(True, lease_id="Lold", lease_expiry_t=9.0), t=3.0)
    rel = of(late, Release)
    assert rel and rel[0].lease_id == "Lold" and rel[0].reason == ReleaseReason.TASK_CANCELLED
