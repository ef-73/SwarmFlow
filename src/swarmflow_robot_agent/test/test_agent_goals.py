"""T029: robot agent goal mode (api.py v2 lifecycle: ROUTE_GOALS, dwell by vertex kind, blocked, pallets).
"""

from __future__ import annotations

import math

import pytest

from swarmflow_core import api
from swarmflow_core.api import Edge, RobotMode, Vertex
from swarmflow_core.graph import WarehouseGraph
from swarmflow_robot_agent.agent_core import (PALLET_LIFT_S, AgentCore, CancelNav, Navigate, RequestReservation,
                                              TaskFinished)

V = {"X_0_0": (0, 0, "intersection"), "X_0_1": (2, 0, "intersection"), "HM1": (0, -2, "home"),
     "PK_A_1": (2, 3, "pick"), "PK_A_2": (2, 5, "pick"), "PL_1": (0, 3, "pallet"),
     "DK1": (6, 4, "dock"), "Q_DK1_1": (6, 2.4, "queue"), "L1": (4, -2, "loading")}
LINKS = [("X_0_0", "X_0_1"), ("X_0_0", "HM1"), ("X_0_1", "PK_A_1"), ("PK_A_1", "PK_A_2"), ("X_0_0", "PL_1"),
         ("X_0_1", "Q_DK1_1"), ("Q_DK1_1", "DK1"), ("X_0_1", "L1")]


def graph():
    verts = {n: Vertex(n, float(x), float(y), k, 0.0 if k in ("dock", "pallet", "home") else None)
             for n, (x, y, k) in V.items()}
    edges = {}
    for a, b in LINKS:
        d = math.hypot(V[a][0] - V[b][0], V[a][1] - V[b][1])
        for s, e in ((a, b), (b, a)):
            edges[(s, e)] = Edge(s, e, d, 3.0, True, None, {})
    return WarehouseGraph(verts, edges, {}, name="goals")


def of(actions, kind):
    return [a for a in actions if isinstance(a, kind)]


def at(agent, name, t, speed=0.0, angular=0.0):
    x, y, _ = V[name]
    return agent.on_pose(float(x), float(y), 0.0, speed, t, angular_speed=angular)


@pytest.fixture
def agent():
    a = AgentCore("robot_1", graph(), traffic_control=True)       # goal tasks ignore traffic control
    at(a, "HM1", 0.0)
    a.on_orchestrator_heartbeat(t=0.0)
    return a


def start(agent, route, t=1.0, order="s1", order_type=api.ORDER_SHELF):
    return agent.start_task("task_1", order, route, t, route_kind=api.ROUTE_GOALS, order_type=order_type)


def test_goal_task_sends_one_pose_per_goal_and_no_reservations(agent):
    acts = start(agent, ["PK_A_2", "DK1"])                        # not adjacent vertices: no edge check
    nav = of(acts, Navigate)
    assert len(nav) == 1 and nav[0].vertices == ("PK_A_2",) and len(nav[0].poses) == 1
    assert nav[0].poses[0][:2] == (2.0, 5.0)
    assert not of(acts, RequestReservation)
    s = agent.state()
    assert s.mode == RobotMode.NAVIGATING and s.next_vertex == "PK_A_2" and s.remaining_route == ("PK_A_2", "DK1")


def test_pick_counts_on_arrival_within_tolerance_and_moves_on(agent):
    start(agent, ["PK_A_1", "PK_A_2", "DK1"])
    x, y, _ = V["PK_A_1"]
    acts = agent.on_pose(x + 0.5 * api.GOAL_TOLERANCE_M, y, 0.0, 0.4, 5.0)     # inside tolerance, still moving
    nav = of(acts, Navigate)
    assert nav and nav[0].vertices == ("PK_A_2",)                               # next goal sent at once, no dwell
    s = agent.state()
    assert s.last_vertex == "PK_A_1" and s.next_vertex == "PK_A_2" and s.mode == RobotMode.NAVIGATING
    acts = agent.on_pose(x + 2.0 * api.GOAL_TOLERANCE_M, y + 1.0, 0.0, 0.4, 6.0)  # not near anything new
    assert not of(acts, Navigate) and agent.state().last_vertex == "PK_A_1"


def test_dock_dwell_for_shelf_orders(agent):
    start(agent, ["PK_A_1", "DK1"])
    at(agent, "PK_A_1", 5.0)
    at(agent, "DK1", 20.0)
    acts = agent.on_nav_result(True, 20.0)
    assert not of(acts, TaskFinished) and agent.state().mode == RobotMode.UNLOADING
    assert not of(agent.tick(20.0 + api.UNLOAD_DWELL_BOX_S - 0.1), TaskFinished)
    done = of(agent.tick(20.0 + api.UNLOAD_DWELL_BOX_S), TaskFinished)
    assert done and done[0].success and done[0].task_id == "task_1"
    s = agent.state()
    assert s.mode == RobotMode.IDLE and s.last_vertex == "DK1" and s.task_id == ""


def test_queue_and_home_goals_have_no_dwell(agent):
    start(agent, ["Q_DK1_1"])
    acts = at(agent, "Q_DK1_1", 8.0)                               # within tolerance: done without waiting
    acts += agent.on_nav_result(True, 8.0)                          # a late Nav2 success changes nothing
    done = of(acts, TaskFinished)
    assert len(done) == 1 and done[0].success
    agent.start_task("task_2", "", ["HM1"], 9.0, route_kind=api.ROUTE_GOALS)
    done = of(at(agent, "HM1", 15.0), TaskFinished)                            # arrival within tolerance suffices
    assert done and done[0].success and agent.state().mode == RobotMode.IDLE


def test_loading_station_keeps_v1_dwell(agent):
    start(agent, ["L1"], order_type=api.ORDER_TRANSPORT)
    at(agent, "L1", 8.0)
    agent.on_nav_result(True, 8.0)
    s = agent.state()
    assert s.mode in (RobotMode.LOADING, RobotMode.UNLOADING)
    assert not of(agent.tick(8.0 + api.LOAD_DWELL_S - 0.1), TaskFinished)
    assert of(agent.tick(8.0 + api.LOAD_DWELL_S), TaskFinished)


# test_pallet_lift_carry_set_down: superseded by test_pallet_docking.py (T042: pre-dock, drive under, deep drop)


def test_blocked_reported_after_no_progress_and_cleared_on_progress(agent):
    start(agent, ["PK_A_2"])
    x, y = 0.0, -1.0
    for k in range(int(api.BLOCKED_REPORT_S) + 2):                 # stands still for > BLOCKED_REPORT_S
        agent.on_pose(x, y, 0.0, 0.0, 2.0 + k)
        acts = agent.tick(2.0 + k)
        assert not of(acts, TaskFinished) and not of(acts, CancelNav)
    s = agent.state()
    assert s.mode == RobotMode.STUCK and s.fault_reason == api.FAULT_BLOCKED and s.task_id == "task_1"
    agent.on_pose(0.0, 0.5, 0.0, 0.4, 20.0)                        # moves 1.5 m towards the goal
    agent.tick(20.0)
    s = agent.state()
    assert s.mode == RobotMode.NAVIGATING and s.fault_reason == ""


def test_nav_failure_reports_blocked_and_retries_same_goal(agent):
    start(agent, ["PK_A_2"])
    acts = agent.on_nav_result(False, 4.0)
    assert not of(acts, TaskFinished)
    s = agent.state()
    assert s.mode == RobotMode.STUCK and s.fault_reason == api.FAULT_BLOCKED
    retries = []
    for k in range(1, 7):
        retries += of(agent.tick(4.0 + 0.5 * k), Navigate)
    assert retries and retries[0].vertices == ("PK_A_2",)          # tries again within 3 s, keeps the task
    assert agent.state().task_id == "task_1"


def test_cancel_still_finishes_with_cancelled(agent):
    start(agent, ["PK_A_2", "DK1"])
    acts = agent.cancel(3.0)
    done = of(acts, TaskFinished)
    assert done and done[0].failure_reason == api.FAILURE_CANCELLED and of(acts, CancelNav)
    assert agent.state().mode == RobotMode.IDLE


def test_follow_up_task_keeps_last_vertex(agent):
    start(agent, ["Q_DK1_1"])
    at(agent, "Q_DK1_1", 8.0)
    agent.on_nav_result(True, 8.0)
    assert agent.state().mode == RobotMode.IDLE
    agent.start_task("task_2", "s1", ["DK1"], 9.0, route_kind=api.ROUTE_GOALS, order_type=api.ORDER_SHELF)
    s = agent.state()
    assert s.last_vertex == "Q_DK1_1" and s.order_id == "s1" and s.next_vertex == "DK1"


def test_angular_speed_reported(agent):
    at(agent, "HM1", 2.0, speed=0.3, angular=-0.7)
    s = agent.state()
    assert s.linear_speed == pytest.approx(0.3) and s.angular_speed == pytest.approx(-0.7)


def test_unknown_goal_rejected(agent):
    with pytest.raises(ValueError):
        agent.start_task("t", "o", ["NOPE"], 1.0, route_kind=api.ROUTE_GOALS)
