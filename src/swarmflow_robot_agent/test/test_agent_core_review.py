"""AgentCore tests for the safety-review fixes (WS-D): zone starts, orphaned grants, stuck metric, epochs."""

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


def agent_at(xy, robot="robot_3", graph=GRAPH):
    a = AgentCore(robot, graph)
    a.on_pose(xy[0], xy[1], 0.0, 0.0, t=0.0)
    a.on_orchestrator_heartbeat(t=0.0)
    return a


def agent_at_hold():
    v = GRAPH.vertices["H_3_1"]
    return agent_at((v.x, v.y))


def test_start_at_zone_entry_requests_before_moving():
    v = GRAPH.vertices["X_3_1"]
    a = agent_at((v.x, v.y))
    acts = a.start_task("t", "", ["X_3_1", "X_3_3"], t=0.0)
    assert not of(acts, Navigate)
    r = of(acts, RequestReservation)[0].request
    assert (r.zone_id, r.entry_vertex, r.exit_vertex, r.direction) == ("Z_aisle_3", "X_3_1", "X_3_3",
                                                                       api.Direction.FORWARD)
    assert a.state().mode == RobotMode.WAITING_RESERVATION
    acts = a.on_reservation_response(r.request_id, ReservationDecision(True, lease_id="Le", lease_expiry_t=5.0), t=0.1)
    assert of(acts, Navigate)[0].vertices == ("X_3_3",)


def test_start_at_other_entry_is_reverse_even_with_inside_pose():
    a = agent_at((11.0, 8.45))                # physically inside the zone
    acts = a.start_task("t", "", ["X_3_3", "X_3_1"], t=0.0)
    assert not of(acts, Navigate)
    r = of(acts, RequestReservation)[0].request
    assert (r.entry_vertex, r.exit_vertex, r.direction) == ("X_3_3", "X_3_1", api.Direction.REVERSE)


def test_start_at_interior_vertex_uses_nearest_entry():
    from swarmflow_core.api import Edge, Vertex
    from swarmflow_core.graph import WarehouseGraph
    verts = dict(GRAPH.vertices)
    verts["M"] = Vertex("M", 11.0, 8.45, "intersection")
    edges = dict(GRAPH.edges)
    edges[("M", "X_3_3")] = Edge("M", "X_3_3", 5.0, 1.3, True, zone="Z_aisle_3")
    g = WarehouseGraph(verts, edges, GRAPH.zones, GRAPH.payloads)
    a = agent_at((14.0, 8.45), graph=g)
    acts = a.start_task("t", "", ["M", "X_3_3"], t=0.0)
    assert not of(acts, Navigate)
    r = of(acts, RequestReservation)[0].request
    assert (r.entry_vertex, r.exit_vertex) == ("X_3_3", "X_3_1")      # nearest entry to the pose


def test_orphaned_grant_is_released():
    a = agent_at_hold()
    a.start_task("t1", "o1", HOLD_ROUTE, t=0.0)
    rid = a.pending_request_id()
    a.cancel(t=1.0)
    acts = a.on_reservation_response(rid, ReservationDecision(True, lease_id="Lo", lease_expiry_t=5.0), t=1.1)
    assert acts == [Release("Lo", ReleaseReason.TASK_CANCELLED)]
    assert a.on_reservation_response(rid, ReservationDecision(True, lease_id="Lo", lease_expiry_t=5.0), t=1.2) == []
    assert a.state().held_lease_ids == ()


def test_abandoned_set_is_bounded_and_denials_do_nothing():
    a = agent_at_hold()
    rids = []
    for k in range(40):
        a.start_task(f"t{k}", "o1", HOLD_ROUTE, t=float(k))
        rids.append(a.pending_request_id())
        a.cancel(t=k + 0.5)
    assert a.on_reservation_response(rids[0], ReservationDecision(True, lease_id="Lold", lease_expiry_t=9.0),
                                     t=50.0) == []                      # fell out of the bounded set
    assert of(a.on_reservation_response(rids[-1], ReservationDecision(True, lease_id="Lnew", lease_expiry_t=9.0),
                                        t=50.0), Release)
    assert a.on_reservation_response(rids[-2], ReservationDecision(False, reason="ZONE_LEASED"), t=50.0) == []


def test_stuck_uses_route_distance_not_straight_line():
    p = GRAPH.vertices["P1"]
    a = agent_at((p.x, p.y))
    a.start_task("t", "o1", ["P1", "X_0_0", "X_1_0", "L1"], t=0.0)
    fin = []
    t = 0.0
    while not fin and t < 120.0:
        t = round(t + 0.5, 3)
        a.on_pose(1.3, 3.0, 0.0, 0.0, t=t)        # straight-line closer to L1, but further along the route
        fin = of(a.tick(t=t), TaskFinished)
    assert fin and fin[0].failure_reason == api.FAILURE_STUCK_TIMEOUT and 59.5 <= t <= 61.0


def granted_at_hold(epoch="A"):
    a = agent_at_hold()
    a.on_orchestrator_heartbeat(t=0.0, epoch=epoch)
    a.start_task("t1", "o1", HOLD_ROUTE, t=0.0)
    acts = a.on_reservation_response(a.pending_request_id(),
                                     ReservationDecision(True, lease_id="Lz", lease_expiry_t=5.0), t=0.1)
    assert of(acts, Navigate)
    return a


def test_epoch_change_outside_zone_drops_lease_and_rerequests():
    a = granted_at_hold()
    assert a.on_orchestrator_heartbeat(t=1.0, epoch="A") == []          # same epoch: nothing
    acts = a.on_orchestrator_heartbeat(t=2.0, epoch="B")
    assert of(acts, CancelNav) and not of(acts, Release) and not of(acts, Navigate)
    assert of(acts, RequestReservation)
    assert a.state().held_lease_ids == () and a.state().mode == RobotMode.WAITING_RESERVATION
    for k in range(1, 20):
        assert not of(a.tick(t=2.0 + k * 0.1), Navigate)
    acts = a.on_reservation_response(a.pending_request_id(),
                                     ReservationDecision(True, lease_id="Lz2", lease_expiry_t=9.0), t=4.0)
    assert of(acts, Navigate) and a.state().held_lease_ids == ("Lz2",)


def test_epoch_change_inside_zone_keeps_lease_and_still_exits():
    a = granted_at_hold()
    a.on_pose(11.0, 8.45, 0.0, 0.5, t=3.0)
    a.tick(t=3.0)
    assert a.on_orchestrator_heartbeat(t=4.0, epoch="B") == []
    assert a.state().held_lease_ids == ("Lz",) and a.state().mode == RobotMode.NAVIGATING
    a.on_pose(17.5, 8.45, 0.0, 0.5, t=9.0)
    assert of(a.tick(t=9.0), Release)[0].reason == ReleaseReason.EXITED


def test_first_epoch_and_empty_epoch_are_not_changes():
    a = granted_at_hold(epoch="")
    assert a.on_orchestrator_heartbeat(t=1.0, epoch="A") == []
    assert a.on_orchestrator_heartbeat(t=2.0) == []
    assert a.state().held_lease_ids == ("Lz",)


def test_epoch_change_while_waiting_rerequests():
    a = agent_at_hold()
    a.on_orchestrator_heartbeat(t=0.0, epoch="A")
    a.start_task("t1", "o1", HOLD_ROUTE, t=0.0)
    old = a.pending_request_id()
    acts = a.on_orchestrator_heartbeat(t=1.0, epoch="B")
    assert of(acts, RequestReservation) and a.pending_request_id() != old
