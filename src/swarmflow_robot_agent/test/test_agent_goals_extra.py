"""T029 extra tests: goal-mode cancel mid-dwell, carrying vs blocked reporting, waypoint task after a goal task."""

from __future__ import annotations

import math

import pytest

from swarmflow_core import api
from swarmflow_core.api import Edge, RobotMode, Vertex
from swarmflow_core.graph import WarehouseGraph
from swarmflow_robot_agent.agent_core import AgentCore, CancelNav, Navigate, RequestReservation, TaskFinished

V = {"A": (0, 0, "intersection"), "B": (4, 0, "intersection"), "C": (8, 0, "intersection"),
     "PL": (0, 4, "pallet"), "DK": (8, 4, "dock")}
LINKS = [("A", "B"), ("B", "C"), ("A", "PL"), ("C", "DK")]


def graph():
    verts = {n: Vertex(n, float(x), float(y), k, None) for n, (x, y, k) in V.items()}
    edges = {}
    for a, b in LINKS:
        d = math.hypot(V[a][0] - V[b][0], V[a][1] - V[b][1])
        for s, e in ((a, b), (b, a)):
            edges[(s, e)] = Edge(s, e, d, 3.0, True, None, {})
    return WarehouseGraph(verts, edges, {}, name="extra")


def of(actions, kind):
    return [a for a in actions if isinstance(a, kind)]


def at(agent, name, t, speed=0.0):
    x, y, _ = V[name]
    return agent.on_pose(float(x), float(y), 0.0, speed, t)


@pytest.fixture
def agent():
    a = AgentCore("robot_1", graph(), traffic_control=True)
    at(a, "A", 0.0)
    a.on_orchestrator_heartbeat(t=0.0)
    return a


def test_cancel_mid_dwell_finishes_cancelled_without_cancelnav(agent):
    agent.start_task("t1", "s1", ["DK"], 1.0, route_kind=api.ROUTE_GOALS, order_type=api.ORDER_SHELF)
    at(agent, "DK", 5.0)
    agent.on_nav_result(True, 5.0)
    assert agent.state().mode == RobotMode.UNLOADING
    acts = agent.cancel(6.0)
    done = of(acts, TaskFinished)
    assert done and not done[0].success and done[0].failure_reason == api.FAILURE_CANCELLED
    assert not of(acts, CancelNav)                       # nothing in flight during a dwell
    s = agent.state()
    assert s.mode == RobotMode.IDLE and s.last_vertex == "DK" and s.task_id == ""
    assert agent.can_accept()


def test_nav_failure_while_carrying_reports_blocked_then_back_to_carrying(agent):
    agent.start_task("t1", "p1", ["PL", "DK"], 1.0, route_kind=api.ROUTE_GOALS, order_type=api.ORDER_PALLET)
    agent.on_pose(0.0, 2.8, math.pi / 2, 0.0, 5.0)               # pre-dock pose (T042: 1.2 m in front of the pallet)
    agent.on_nav_result(True, 5.0)
    assert agent.state().mode == RobotMode.LIFTING
    at(agent, "PL", 5.5)                                          # drove under the pallet
    assert of(agent.tick(5.5 + 3.0), Navigate)
    assert agent.state().mode == RobotMode.CARRYING
    agent.on_nav_result(False, 12.0)
    s = agent.state()
    assert s.mode == RobotMode.STUCK and s.fault_reason == api.FAULT_BLOCKED and s.task_id == "t1"
    retry = []
    for k in range(1, 8):
        agent.on_pose(0.0, 4.0, 0.0, 0.0, 12.0 + 0.5 * k)         # fresh poses, no progress
        retry += of(agent.tick(12.0 + 0.5 * k), Navigate)
    assert retry and retry[0].vertices == ("DK",)
    assert agent.state().mode == RobotMode.STUCK          # still reported until the robot makes progress
    agent.on_pose(2.0, 3.0, 0.0, 0.4, 16.0)
    agent.tick(16.0)
    s = agent.state()
    assert s.mode == RobotMode.CARRYING and s.fault_reason == ""


def test_pallet_follow_up_task_is_carrying(agent):
    agent.start_task("t2", "p1", ["DK"], 1.0, route_kind=api.ROUTE_GOALS, order_type=api.ORDER_PALLET)
    assert agent.state().mode == RobotMode.CARRYING


def test_waypoint_task_after_goal_task_still_requests_reservations():
    from swarmflow_core.api import Zone, ZoneEntry
    verts = {n: Vertex(n, float(x), float(y), k, None) for n, (x, y, k) in V.items()}
    edges = {}
    for a, b in LINKS:
        d = math.hypot(V[a][0] - V[b][0], V[a][1] - V[b][1])
        for s, e in ((a, b), (b, a)):
            edges[(s, e)] = Edge(s, e, d, 3.0, True, "Z1" if {a, b} == {"B", "C"} else None, {})
    zone = Zone("Z1", 1, (("B", "C"), ("C", "B")), ("B", "C"), (ZoneEntry("B", "A"), ZoneEntry("C", "DK")),
                ((3.5, -1.0), (8.5, -1.0), (8.5, 1.0), (3.5, 1.0)))
    g = WarehouseGraph(verts, edges, {"Z1": zone}, name="zoned")
    a = AgentCore("robot_1", g, traffic_control=True)
    at(a, "A", 0.0)
    a.on_orchestrator_heartbeat(t=0.0)
    a.start_task("g1", "", ["B"], 1.0, route_kind=api.ROUTE_GOALS)
    at(a, "B", 3.0)
    assert a.state().mode == RobotMode.IDLE
    acts = a.start_task("w1", "", ["B", "C"], 4.0)
    assert of(acts, RequestReservation)
    assert a.state().mode == RobotMode.WAITING_RESERVATION


# -- review fixes ------------------------------------------------------------------------------------------------


def test_detour_is_not_blocked(agent):
    agent.start_task("t1", "", ["C"], 1.0, route_kind=api.ROUTE_GOALS)       # goal (8, 0); robot at (0, 0)
    # drives 0.6 m every 3 s on a circle-ish path around a rack: straight-line distance to the goal never shrinks
    path = [(0.0, 0.0), (0.0, 0.6), (0.0, 1.2), (0.0, 0.6), (0.0, 0.0), (0.0, -0.6), (0.0, 0.0)]
    t = 1.0
    for x, y in path:
        t += 3.0
        agent.on_pose(x, y, 0.0, 0.2, t)
        agent.tick(t)
        assert agent.state().mode == RobotMode.NAVIGATING


def test_turning_in_place_is_blocked(agent):
    agent.start_task("t1", "", ["C"], 1.0, route_kind=api.ROUTE_GOALS)
    for k in range(int(api.BLOCKED_REPORT_S) + 2):
        t = 2.0 + k
        agent.on_pose(0.0, 0.0, 0.3 * k, 0.0, t, angular_speed=0.5)
        agent.tick(t)
    assert agent.state().mode == RobotMode.STUCK


def test_stale_pose_in_carrying_shows_fault(agent):
    agent.start_task("t2", "p1", ["DK"], 1.0, route_kind=api.ROUTE_GOALS, order_type=api.ORDER_PALLET)
    agent.tick(1.0 + 2.0 * 1.0 + 0.1)
    s = agent.state()
    assert s.mode == RobotMode.FAULT and s.fault_reason == "STALE_POSE"


def test_stale_pose_never_skips_a_goal(agent):
    # robot is within tolerance of "A" but its pose is 5 s old
    acts = agent.start_task("t1", "", ["A", "B"], 5.0, route_kind=api.ROUTE_GOALS)
    nav = of(acts, Navigate)
    assert nav and nav[0].vertices == ("A",)
    assert agent.state().next_vertex == "A"


def test_cancel_while_stuck_with_retry_pending(agent):
    agent.start_task("t1", "", ["C"], 1.0, route_kind=api.ROUTE_GOALS)
    agent.on_nav_result(False, 4.0)
    assert agent.state().mode == RobotMode.STUCK
    done = of(agent.cancel(4.2), TaskFinished)
    assert done and done[0].failure_reason == api.FAILURE_CANCELLED
    assert agent.state().mode == RobotMode.IDLE
    for k in range(1, 12):
        agent.on_pose(0.0, 0.0, 0.0, 0.0, 4.2 + 0.5 * k)
        assert not of(agent.tick(4.2 + 0.5 * k), Navigate)


def test_tolerance_advance_cancels_nav_before_next_navigate(agent):
    agent.start_task("t1", "", ["B", "C"], 1.0, route_kind=api.ROUTE_GOALS)
    acts = agent.on_pose(4.0, 0.0, 0.0, 0.3, 3.0)
    kinds = [type(a) for a in acts]
    assert kinds == [CancelNav, Navigate]
    assert acts[1].vertices == ("C",)
