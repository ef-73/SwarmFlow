"""T042: pallet docking in the robot agent (F-02, F-04): pre-dock pose, straight drive under the
pallet, lift, carry with the large footprint, deep drop at the dock, unload, back out, normal footprint.
"""

from __future__ import annotations

import math

import pytest

from swarmflow_core import api
from swarmflow_core.api import Edge, RobotMode, Vertex
from swarmflow_core.graph import WarehouseGraph
from swarmflow_robot_agent.agent_core import (DOCK_AHEAD_M, DOCK_TIMEOUT_S, PALLET_LIFT_S, PREDOCK_M, UNDOCK_M,
                                              AgentCore, CancelNav, DockDrive, Navigate, SetFootprint, StopDrive,
                                              TaskFinished)

N = math.pi / 2
V = {"X_0_0": (0, 0, "intersection", None), "HM1": (0, -2, "home", N), "PL_1": (0, 4, "pallet", N),
     "DK1": (6, 8, "dock", N), "PK_A_1": (3, 3, "pick", None)}
LINKS = [("X_0_0", "HM1"), ("X_0_0", "PL_1"), ("PL_1", "DK1"), ("X_0_0", "PK_A_1")]


def graph():
    verts = {n: Vertex(n, float(x), float(y), k, yaw) for n, (x, y, k, yaw) in V.items()}
    edges = {}
    for a, b in LINKS:
        d = math.hypot(V[a][0] - V[b][0], V[a][1] - V[b][1])
        for s, e in ((a, b), (b, a)):
            edges[(s, e)] = Edge(s, e, d, 3.0, True, None, {})
    return WarehouseGraph(verts, edges, {}, name="dock")


def of(acts, kind):
    return [a for a in acts if isinstance(a, kind)]


@pytest.fixture
def agent():
    a = AgentCore("robot_1", graph(), traffic_control=False)
    a.on_pose(0.0, -2.0, N, 0.0, 0.0)
    a.on_orchestrator_heartbeat(t=0.0)
    return a


def start(a, t=1.0):
    return a.start_task("t1", "p1", ["PL_1", "DK1"], t, route_kind=api.ROUTE_GOALS, order_type=api.ORDER_PALLET)


def test_pallet_goal_first_goes_to_the_predock_pose(agent):
    nav = of(start(agent), Navigate)
    assert len(nav) == 1 and nav[0].vertices == ("PL_1",)
    x, y, yaw = nav[0].poses[0]
    assert (x, y) == pytest.approx((0.0, 4.0 - PREDOCK_M)) and yaw == pytest.approx(N)
    assert agent.state().mode == RobotMode.NAVIGATING


def _dock_under(agent, t=10.0):
    start(agent)
    agent.on_pose(0.0, 4.0 - PREDOCK_M, N, 0.0, t)
    acts = agent.on_nav_result(True, t)
    return acts


def test_drive_under_lift_switch_footprint_carry(agent):
    acts = _dock_under(agent)
    dd = of(acts, DockDrive)
    assert len(dd) == 1 and not dd[0].reverse and dd[0].target == pytest.approx((0.0, 4.0, N))
    assert agent.state().mode == RobotMode.LIFTING
    assert not of(agent.on_pose(0.0, 3.9, N, 0.1, 13.0), StopDrive)          # 10 cm short: keep driving
    acts = agent.on_pose(0.01, 3.99, N, 0.05, 14.0)                           # within 3 cm / 2 cm
    assert of(acts, StopDrive) and agent.state().mode == RobotMode.LIFTING
    assert not of(agent.tick(14.0 + PALLET_LIFT_S - 0.1), Navigate)
    acts = agent.tick(14.0 + PALLET_LIFT_S)
    fp = of(acts, SetFootprint)
    assert fp and fp[0].kind == "carry"
    nav = of(acts, Navigate)
    assert nav and nav[0].vertices == ("DK1",)
    assert acts.index(fp[0]) < acts.index(nav[0])                              # footprint first, then drive
    s = agent.state()
    assert s.mode == RobotMode.CARRYING and s.last_vertex == "PL_1" and s.next_vertex == "DK1"


def _to_dock(agent):
    _dock_under(agent)
    agent.on_pose(0.0, 4.0, N, 0.0, 14.0)
    agent.tick(14.0 + PALLET_LIFT_S)
    agent.on_pose(6.0, 8.0 - PREDOCK_M, N, 0.0, 40.0)
    return agent.on_nav_result(True, 40.0)


def test_drop_on_the_dock_pad_unload_back_out_normal_footprint(agent):
    acts = _to_dock(agent)
    assert DOCK_AHEAD_M == 0.0                                       # T066: the pallet goes down on the dock pad
    dd = of(acts, DockDrive)
    assert dd and not dd[0].reverse and dd[0].target == pytest.approx((6.0, 8.0 + DOCK_AHEAD_M, N))
    assert agent.state().mode == RobotMode.CARRYING
    acts = agent.on_pose(6.0, 8.0 + DOCK_AHEAD_M - 0.01, N, 0.05, 45.0)
    assert of(acts, StopDrive) and agent.state().mode == RobotMode.UNLOADING
    assert not of(agent.tick(45.0 + api.UNLOAD_DWELL_PALLET_S - 0.1), DockDrive)
    acts = agent.tick(45.0 + api.UNLOAD_DWELL_PALLET_S)
    dd = of(acts, DockDrive)
    assert dd and dd[0].reverse
    assert dd[0].target == pytest.approx((6.0, 8.0 + DOCK_AHEAD_M - UNDOCK_M, N))
    assert agent.state().mode == RobotMode.LIFTING                           # setting down / backing out
    acts = agent.on_pose(6.0, 8.0 + DOCK_AHEAD_M - UNDOCK_M + 0.01, N, -0.05, 62.0)
    assert of(acts, StopDrive)
    fp = of(acts, SetFootprint)
    assert fp and fp[0].kind == "normal"
    done = of(acts, TaskFinished)
    assert done and done[0].success
    assert agent.state().mode == RobotMode.IDLE and agent.state().last_vertex == "DK1"


def test_dock_drive_timeout_fails_the_task(agent):
    _dock_under(agent, t=10.0)
    acts = []
    for k in range(int(DOCK_TIMEOUT_S) + 2):                 # stuck in front of the pallet, no progress
        acts += agent.on_pose(0.0, 3.0, N, 0.0, 10.0 + k)
        acts += agent.tick(10.0 + k)
    done = of(acts, TaskFinished)
    assert done and not done[0].success and done[0].failure_reason == api.FAILURE_NAV_PREFIX + "PL_1"
    assert of(acts, StopDrive)


def test_cancel_while_docking_stops_the_drive(agent):
    _dock_under(agent)
    acts = agent.cancel(11.0)
    assert of(acts, StopDrive) and not of(acts, CancelNav)
    done = of(acts, TaskFinished)
    assert done and done[0].failure_reason == api.FAILURE_CANCELLED


def test_carry_follow_up_task_starts_carrying_with_carry_footprint(agent):
    """A follow-up task (dock queue) for a pallet already on board: no pickup, carry footprint kept."""
    acts = agent.start_task("t2", "p1", ["DK1"], 50.0, route_kind=api.ROUTE_GOALS, order_type=api.ORDER_PALLET)
    assert agent.state().mode == RobotMode.CARRYING
    assert not of(acts, DockDrive) and of(acts, Navigate)[0].vertices == ("DK1",)


def test_shelf_orders_never_dock(agent):
    acts = agent.start_task("t3", "s1", ["PK_A_1", "DK1"], 1.0, route_kind=api.ROUTE_GOALS,
                            order_type=api.ORDER_SHELF)
    assert of(acts, Navigate)[0].poses[0][:2] == pytest.approx((3.0, 3.0))
    agent.on_pose(3.0, 3.0, 0.0, 0.0, 5.0)
    agent.on_pose(6.0, 8.0, N, 0.0, 20.0)
    acts = agent.on_nav_result(True, 20.0)
    assert not of(acts, DockDrive) and agent.state().mode == RobotMode.UNLOADING


def test_pallet_dock_goal_is_the_pre_dock_pose(agent):
    """T066: Nav2 stops PREDOCK_M before a pallet order's dock; the straight drive has room to fix a lateral offset."""
    _dock_under(agent)
    agent.on_pose(0.0, 4.0, N, 0.0, 14.0)
    acts = agent.tick(14.0 + PALLET_LIFT_S)
    nav = of(acts, Navigate)
    assert nav and nav[0].vertices == ("DK1",) and nav[0].poses[0] == pytest.approx((6.0, 8.0 - PREDOCK_M, N))
