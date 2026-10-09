"""Robot agent core tests — safety-critical (design §6.3, §6.5 rules 3–6, §13.5). Lead test: do not edit.

Implementation under test: ``swarmflow_robot_agent.agent_core.AgentCore`` (pure Python, no ROS).
"""

from __future__ import annotations

import math
import pathlib

import pytest

from swarmflow_core import api
from swarmflow_core.api import RobotMode, ReleaseReason, ReservationDecision
from swarmflow_core.graph import load_layout
from swarmflow_robot_agent.agent_core import (AgentCore, CancelNav, Navigate, Release, RequestReservation,
                                              TaskFinished)

FIX = pathlib.Path(__file__).resolve().parents[3] / "tests" / "fixtures" / "standard"
GRAPH = load_layout(FIX)
# P1 → pickup L1 → through storage aisle 3 → dropoff D3 (every consecutive pair is a graph edge)
ROUTE = ["P1", "X_0_0", "X_1_0", "L1", "X_1_0", "X_2_0", "X_3_0", "H_3_1", "X_3_1", "X_3_3", "X_3_4", "D3"]


def V(name):
    v = GRAPH.vertices[name]
    return v.x, v.y


def of(actions, kind):
    return [a for a in actions if isinstance(a, kind)]


def new_agent(traffic_control=True, start="P1"):
    a = AgentCore("robot_1", GRAPH, traffic_control=traffic_control)
    a.on_pose(*V(start), 0.0, 0.0, t=0.0)
    a.on_orchestrator_heartbeat(t=0.0)
    return a


def drive_to(agent, vertex, t):
    """Pose at vertex + Nav2 success for the current segment."""
    agent.on_pose(*V(vertex), 0.0, 0.0, t=t)
    return agent.on_nav_result(True, t=t)


def test_route_is_valid():
    assert all((a, b) in GRAPH.edges for a, b in zip(ROUTE, ROUTE[1:]))


def test_segments_end_at_pickup_hold_and_goal_with_dwell_and_grant():
    a = new_agent()
    acts = a.start_task("task_1", "o1", ROUTE, t=0.0)
    nav = of(acts, Navigate)
    assert len(nav) == 1 and nav[0].vertices[-1] == "L1"           # segment 1 ends at pickup
    assert nav[0].vertices[0] == "X_0_0"                           # starts at the next vertex
    assert len(nav[0].poses) == len(nav[0].vertices)
    x, y, yaw = nav[0].poses[-1]
    assert (x, y) == pytest.approx(V("L1"))
    assert a.state().mode == RobotMode.NAVIGATING

    acts = drive_to(a, "L1", 30.0)                                  # arrive at pickup → LOADING dwell
    assert not of(acts, Navigate) and a.state().mode == RobotMode.LOADING
    assert not of(a.tick(t=30.0 + api.LOAD_DWELL_S - 0.1), Navigate)
    acts = a.tick(t=30.0 + api.LOAD_DWELL_S)
    nav = of(acts, Navigate)
    assert len(nav) == 1 and nav[0].vertices[-1] == "H_3_1"         # segment 2 ends at the hold
    assert a.state().mode == RobotMode.NAVIGATING

    acts = drive_to(a, "H_3_1", 50.0)                               # at hold → request, no nav
    reqs = of(acts, RequestReservation)
    assert len(reqs) == 1 and not of(acts, Navigate)
    r = reqs[0].request
    assert (r.zone_id, r.entry_vertex, r.exit_vertex) == ("Z_aisle_3", "X_3_1", "X_3_3")
    assert r.direction == api.Direction.FORWARD and r.robot_id == "robot_1"
    assert a.state().mode == RobotMode.WAITING_RESERVATION

    acts = a.on_reservation_response(r.request_id, ReservationDecision(False, reason="ZONE_LEASED", retry_after_s=1.0),
                                     t=50.2)
    assert not of(acts, Navigate)
    retry = []
    t = 50.2
    while not retry and t < 53.0:                                   # retry after ~1 s (±20 % jitter)
        t = round(t + 0.05, 3)
        retry = of(a.tick(t=t), RequestReservation)
    assert retry and 50.2 + 0.8 - 1e-6 <= t <= 50.2 + 1.2 + 1e-6

    acts = a.on_reservation_response(retry[0].request.request_id,
                                     ReservationDecision(True, lease_id="lease_00001", lease_expiry_t=t + 5), t=t)
    nav = of(acts, Navigate)
    assert len(nav) == 1 and nav[0].vertices[0] == "X_3_1" and nav[0].vertices[-1] == "D3"
    assert a.state().held_lease_ids == ("lease_00001",)

    a.on_pose(11.0, 8.45, 0.0, 0.5, t=t + 10)                       # inside the zone: no release
    assert not of(a.tick(t=t + 10), Release)
    a.on_pose(16.5, 8.45, 0.0, 0.5, t=t + 21)                       # outside, but < RELEASE_MARGIN_M
    assert not of(a.tick(t=t + 21), Release)
    a.on_pose(17.1, 8.45, 0.0, 0.5, t=t + 22)                       # >= 1.0 m outside the zone
    rel = of(a.tick(t=t + 22), Release)
    assert len(rel) == 1 and rel[0].lease_id == "lease_00001" and rel[0].reason == ReleaseReason.EXITED
    assert a.state().held_lease_ids == ()

    acts = drive_to(a, "D3", t + 30)                                # at dropoff → UNLOADING dwell → finished
    assert a.state().mode == RobotMode.UNLOADING and not of(acts, TaskFinished)
    fin = of(a.tick(t=t + 30 + api.LOAD_DWELL_S), TaskFinished)
    assert len(fin) == 1 and fin[0].success and fin[0].task_id == "task_1"
    assert a.state().mode == RobotMode.IDLE and a.state().last_vertex == "D3"


def test_heartbeats_list_held_leases():
    a = new_agent()
    a.start_task("task_1", "o1", ["H_3_1", "X_3_1", "X_3_3", "X_3_4"], t=0.0)
    a.on_pose(*V("H_3_1"), 0.0, 0.0, t=0.0)
    req = [x for x in a.tick(t=0.1) if isinstance(x, RequestReservation)]
    if not req:  # agent may have requested immediately in start_task
        req = []
    st = a.state()
    assert st.mode == RobotMode.WAITING_RESERVATION
    rid = a.pending_request_id()
    a.on_reservation_response(rid, ReservationDecision(True, lease_id="lease_7", lease_expiry_t=5.0), t=0.2)
    assert a.heartbeat_lease_ids() == ["lease_7"]


def test_never_enters_zone_without_grant_when_orchestrator_down():
    a = new_agent()
    a.start_task("task_1", "o1", ["H_3_1", "X_3_1", "X_3_3", "X_3_4"], t=0.0)
    rid = a.pending_request_id()
    assert rid
    # no response within 2 s → orchestrator considered down; keeps holding, re-requests later, never navigates
    for k in range(1, 400):
        t = k * 0.05
        acts = a.tick(t=t)
        assert not of(acts, Navigate), f"navigated into zone without grant at t={t}"
    st = a.state()
    assert st.mode == RobotMode.FAULT and st.fault_reason == api.FAULT_ORCHESTRATOR_TIMEOUT  # after 10 s
    # orchestrator back → leaves FAULT, requests again; a grant lets it go
    a.on_orchestrator_heartbeat(t=20.0)
    acts = a.tick(t=20.0)
    assert a.state().mode == RobotMode.WAITING_RESERVATION
    rid = a.pending_request_id()
    assert rid
    acts = a.on_reservation_response(rid, ReservationDecision(True, lease_id="L9", lease_expiry_t=25.0), t=20.1)
    assert of(acts, Navigate)


def test_missing_orchestrator_heartbeat_marks_down_outside_zone():
    a = new_agent()
    a.start_task("task_1", "o1", ["H_3_1", "X_3_1", "X_3_3", "X_3_4"], t=0.0)
    rid = a.pending_request_id()
    a.on_reservation_response(rid, ReservationDecision(False, reason="ZONE_LEASED", retry_after_s=1.0), t=0.1)
    for k in range(1, 300):
        t = round(k * 0.05, 3)
        if t <= 2.0:
            a.on_orchestrator_heartbeat(t=t)
        assert not of(a.tick(t=t), Navigate)
    assert a.state().mode == RobotMode.FAULT     # heartbeat gone since t=2 → down at 5 s → FAULT at 12 s


def test_inside_zone_continues_when_orchestrator_down():
    a = new_agent(start="H_3_1")
    a.start_task("task_1", "o1", ["H_3_1", "X_3_1", "X_3_3", "X_3_4", "D3"], t=0.0)
    a.on_reservation_response(a.pending_request_id(), ReservationDecision(True, lease_id="L1x", lease_expiry_t=5.0),
                              t=0.1)
    a.on_pose(11.0, 8.45, 0.0, 0.5, t=5.0)          # inside the zone; orchestrator silent from now on
    for k in range(1, 100):
        acts = a.tick(t=5.0 + k * 0.1)
        assert not of(acts, CancelNav)               # keeps driving through the zone
    assert a.state().mode in (RobotMode.NAVIGATING, RobotMode.FAULT)


def test_nav_failure_retry_once_then_stuck():
    a = new_agent()
    acts = a.start_task("task_1", "o1", ROUTE, t=0.0)
    first = of(acts, Navigate)[0]
    acts = a.on_nav_result(False, t=5.0)
    again = of(acts, Navigate)
    assert len(again) == 1 and again[0].vertices[-1] == first.vertices[-1]   # one retry of the same segment
    acts = a.on_nav_result(False, t=9.0)
    fin = of(acts, TaskFinished)
    assert len(fin) == 1 and not fin[0].success and fin[0].failure_reason.startswith("NAV_FAILED:")
    assert a.state().mode == RobotMode.STUCK


def test_stuck_timeout_no_progress():
    a = new_agent()
    a.start_task("task_1", "o1", ROUTE, t=0.0)
    fin = []
    t = 0.0
    while not fin and t < 120.0:
        t = round(t + 0.5, 3)
        a.on_pose(*V("P1"), 0.0, 0.0, t=t)          # never moves
        acts = a.tick(t=t)
        fin = of(acts, TaskFinished)
    assert fin and fin[0].failure_reason == api.FAILURE_STUCK_TIMEOUT
    assert api.STUCK_WINDOW_S - 0.5 <= t <= api.STUCK_WINDOW_S + 1.0
    assert of(acts, CancelNav)
    assert a.state().mode == RobotMode.STUCK


def test_waiting_for_grant_is_not_stuck():
    a = new_agent(start="H_3_1")
    a.start_task("task_1", "o1", ["H_3_1", "X_3_1", "X_3_3", "X_3_4"], t=0.0)
    t = 0.0
    while t < 150.0:
        t = round(t + 0.5, 3)
        a.on_orchestrator_heartbeat(t=t)
        for x in a.tick(t=t):
            if isinstance(x, RequestReservation):
                a.on_reservation_response(x.request.request_id,
                                          ReservationDecision(False, reason="ZONE_LEASED", retry_after_s=1.0), t=t)
            assert not isinstance(x, TaskFinished)
    assert a.state().mode == RobotMode.WAITING_RESERVATION


def test_cancel_releases_leases():
    a = new_agent(start="H_3_1")
    a.start_task("task_1", "o1", ["H_3_1", "X_3_1", "X_3_3", "X_3_4"], t=0.0)
    a.on_reservation_response(a.pending_request_id(), ReservationDecision(True, lease_id="Lc", lease_expiry_t=5.0), t=0.1)
    acts = a.cancel(t=1.0)
    assert of(acts, CancelNav)
    rel = of(acts, Release)
    assert rel and rel[0].lease_id == "Lc" and rel[0].reason == ReleaseReason.TASK_CANCELLED
    fin = of(acts, TaskFinished)
    assert fin and not fin[0].success
    assert a.state().mode == RobotMode.IDLE


def test_baseline_a_no_reservations_single_segment_after_pickup():
    a = new_agent(traffic_control=False)
    acts = a.start_task("task_1", "o1", ROUTE, t=0.0)
    assert of(acts, Navigate)[0].vertices[-1] == "L1"
    drive_to(a, "L1", 30.0)
    acts = a.tick(t=30.0 + api.LOAD_DWELL_S)
    nav = of(acts, Navigate)
    assert len(nav) == 1 and nav[0].vertices[-1] == "D3"          # whole rest of the route, holds included
    assert not of(acts, RequestReservation)


def test_move_only_task_has_no_dwell():
    a = new_agent(start="X_3_1")
    route = ["X_3_1", "X_3_0", "X_2_0", "X_1_0", "X_0_0", "P1"]
    acts = a.start_task("park_1", "", route, t=0.0)
    assert of(acts, Navigate)[0].vertices[-1] == "P1"
    acts = drive_to(a, "P1", 20.0)
    fin = of(acts, TaskFinished) or of(a.tick(t=20.0), TaskFinished)
    assert fin and fin[0].success


def test_yaw_follows_next_edge():
    a = new_agent()
    nav = of(a.start_task("task_1", "o1", ROUTE, t=0.0), Navigate)[0]
    # X_0_0 → X_1_0 heads north (+y): pose at X_0_0 has yaw ≈ +pi/2
    i = nav.vertices.index("X_0_0")
    assert nav.poses[i][2] == pytest.approx(math.pi / 2, abs=1e-6)


def test_busy_agent_rejects_new_task():
    a = new_agent()
    a.start_task("task_1", "o1", ROUTE, t=0.0)
    assert a.can_accept() is False
    with pytest.raises(RuntimeError):
        a.start_task("task_2", "o2", ROUTE, t=1.0)
