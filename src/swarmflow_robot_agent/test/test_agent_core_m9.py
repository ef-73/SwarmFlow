"""M9 robot-agent hardening (independent review S1, S4). Lead test: do not edit."""

from __future__ import annotations

import pathlib

from swarmflow_core import api
from swarmflow_core.api import LeaseState, ReservationDecision, RobotMode
from swarmflow_core.graph import load_layout
from swarmflow_robot_agent.agent_core import AgentCore, CancelNav, Navigate, RequestReservation

GRAPH = load_layout(pathlib.Path(__file__).resolve().parents[3] / "tests" / "fixtures" / "standard")
ROUTE = ["H_3_1", "X_3_1", "X_3_3", "X_3_4", "D3"]


def of(acts, kind):
    return [a for a in acts if isinstance(a, kind)]


def granted_agent():
    a = AgentCore("robot_1", GRAPH)
    v = GRAPH.vertices["H_3_1"]
    a.on_pose(v.x, v.y, 0.0, 0.0, t=0.0)
    a.on_orchestrator_heartbeat(t=0.0)
    a.start_task("t1", "", ROUTE, t=0.0)
    acts = a.on_reservation_response(a.pending_request_id(),
                                     ReservationDecision(True, lease_id="lease_1", lease_expiry_t=5.0), t=0.1)
    assert of(acts, Navigate)
    return a


# ---- S1: a lease lost before entering the zone stops the robot and is re-requested -------------------------------

def test_lease_lost_before_entry_stops_and_rerequests():
    for state in (LeaseState.OCCUPIED_UNKNOWN, LeaseState.REVOKED, LeaseState.RELEASED, LeaseState.EXPIRED):
        a = granted_agent()
        a.on_pose(5.0, 7.6, 0.3, 0.4, t=0.5)                       # between hold and entry, still outside
        acts = a.on_lease_event("lease_1", state, t=0.6)
        assert of(acts, CancelNav), state
        st = a.state()
        assert st.mode == RobotMode.WAITING_RESERVATION and st.held_lease_ids == (), state
        req = of(acts, RequestReservation) or of(a.tick(t=0.7), RequestReservation)
        assert req and req[0].request.zone_id == "Z_aisle_3", state
        acts = a.on_reservation_response(req[0].request.request_id,
                                         ReservationDecision(True, lease_id="lease_2", lease_expiry_t=6.0), t=1.0)
        assert of(acts, Navigate)


def test_lease_lost_inside_zone_keeps_driving_out():
    a = granted_agent()
    a.on_pose(11.0, 8.45, 0.0, 0.5, t=10.0)                      # inside Z_aisle_3
    acts = a.on_lease_event("lease_1", LeaseState.OCCUPIED_UNKNOWN, t=10.1)
    assert not of(acts, CancelNav)
    assert a.state().mode == RobotMode.NAVIGATING


def test_events_for_other_leases_ignored():
    a = granted_agent()
    assert a.on_lease_event("lease_99", LeaseState.REVOKED, t=0.5) == []
    assert a.on_lease_event("lease_1", LeaseState.GRANTED, t=0.5) == []
    assert a.state().held_lease_ids == ("lease_1",)


# ---- S4: no pose → FAULT/NO_POSE; stale pose → FAULT/STALE_POSE -------------------------------------------------

def test_no_pose_reports_fault_and_cannot_accept():
    a = AgentCore("robot_1", GRAPH)
    st = a.state()
    assert st.mode == RobotMode.FAULT and st.fault_reason == "NO_POSE"
    assert a.has_pose() is False and a.can_accept() is False
    a.on_pose(1.3, 1.1, 0.0, 0.0, t=1.0)
    assert a.has_pose() and a.state().mode == RobotMode.IDLE and a.can_accept()


def test_stale_pose_reports_fault_until_fresh():
    a = AgentCore("robot_1", GRAPH)
    a.on_pose(1.3, 1.1, 0.0, 0.0, t=1.0)
    a.tick(t=1.0 + api.ROBOT_STATE_PERIOD_S)
    assert a.state().mode == RobotMode.IDLE
    a.tick(t=3.0)                                                  # > 1 s since the last pose
    st = a.state()
    assert st.mode == RobotMode.FAULT and st.fault_reason == "STALE_POSE"
    assert a.pose_time() == 1.0
    a.on_pose(1.3, 1.1, 0.0, 0.0, t=3.1)
    assert a.state().mode == RobotMode.IDLE
