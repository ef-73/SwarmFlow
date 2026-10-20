"""T068: the agent yields on reserved trajectories (SWARMFLOW_YIELD) and breaks deadlock cycles (SWARMFLOW_UNJAM)."""

from __future__ import annotations

import math

import pytest

from swarmflow_core import api
from swarmflow_core.api import RobotMode
from swarmflow_robot_agent.agent_core import (YIELD_MAX_S, AgentCore, BackUp, CancelBackUp, CancelNav, Navigate,
                                              TaskFinished)
from swarmflow_robot_agent.deadlock import HOLD_S
from swarmflow_robot_agent.trajectory import Reservation
from test_agent_goals import graph, of  # noqa: E402  (shared fixture graph)

# my way to PK_A_2 (2, 5): up the x = 0 line to X_0_0, east along y = 0 to X_0_1, then north
PLAN = [(0.0, -2.5), (0.0, 0.0), (2.0, 0.0), (2.0, 3.0), (2.0, 5.0)]


def agent(yield_on=True, unjam=False, rid="robot_2", start=(0.0, -2.5)):
    a = AgentCore(rid, graph(), traffic_control=True, yield_to_others=yield_on, unjam=unjam)
    a.on_pose(start[0], start[1], math.pi / 2, 0.0, 0.0)
    a.on_orchestrator_heartbeat(t=0.0)
    a.start_task("task_1", "s1", ["PK_A_2"], 1.0, route_kind=api.ROUTE_GOALS, order_type=api.ORDER_SHELF)
    return a


def moving(a, t, x=0.0, y=-2.5):
    a.on_pose(x, y, math.pi / 2, 0.5, t)
    a.on_plan([(x, y)] + PLAN[1:], t)


def westbound(t0, x0=4.0, n=31, rid="robot_1", reserved=0.5, pallet=False):
    """Another robot driving west along y = 0 from x0 at 0.5 m/s from t0."""
    return Reservation(rid, reserved, pallet, tuple((t0 + 0.5 * k, x0 - 0.25 * k, 0.0) for k in range(n)))


def test_off_by_default_never_yields():
    a = agent(yield_on=False)
    moving(a, 2.0)
    a.on_reservation(westbound(2.0), 2.0)
    assert not of(a.tick(2.0), Navigate) and a.state().mode == RobotMode.NAVIGATING


def test_yields_to_an_older_reservation_by_waiting_clear_of_its_path():
    a = agent()
    moving(a, 2.0)
    a.on_reservation(westbound(2.0), 2.0)
    acts = a.tick(2.0)
    nav = of(acts, Navigate)
    assert nav and nav[0].vertices == ("WAIT",)
    wx, wy, _ = nav[0].poses[0]
    assert wx == pytest.approx(0.0) and wy <= -1.3 + 1e-9                 # >= 1.3 m from its path along y = 0
    s = a.state()
    assert s.mode == RobotMode.WAITING_RESERVATION and s.fault_reason == "YIELD:robot_1"
    res = a.reservation(2.5)
    assert res.points == ((2.5, wx, wy),) and res.reserved_t == 1.0      # says: standing there, same reservation
    assert a.yield_events[-1]["type"] == "yield_start"


def test_waits_while_its_path_is_reserved_and_goes_on_once_the_other_passed():
    a = agent()
    moving(a, 2.0)
    a.on_reservation(westbound(2.0), 2.0)
    a.tick(2.0)
    a.on_pose(0.0, -1.5, math.pi / 2, 0.0, 6.0)
    a.on_reservation(westbound(6.0, x0=2.0), 6.0)                          # still coming
    assert not of(a.tick(6.0), Navigate) and a.state().mode == RobotMode.WAITING_RESERVATION
    a.on_reservation(westbound(12.0, x0=-1.5), 12.0)                       # past my corridor
    nav = of(a.tick(12.0), Navigate)
    assert nav and nav[0].vertices == ("PK_A_2",)
    assert a.state().mode == RobotMode.NAVIGATING and a.reservation(12.0).reserved_t == 1.0
    assert a.yield_events[-1]["type"] == "yield_end" and not a.yield_events[-1]["timed_out"]


def test_a_younger_reservation_yields_to_me_not_the_other_way():
    a = agent()
    moving(a, 2.0)
    a.on_reservation(westbound(2.0, reserved=1.5), 2.0)
    assert not of(a.tick(2.0), Navigate) and a.state().mode == RobotMode.NAVIGATING


def test_a_pallet_carrier_outranks_an_older_reservation():
    a = agent()
    moving(a, 2.0)
    a.on_reservation(westbound(2.0, reserved=50.0, pallet=True), 2.0)
    assert of(a.tick(2.0), Navigate)[0].vertices == ("WAIT",)


def test_paths_that_cross_at_different_times_are_no_conflict():
    a = agent()
    moving(a, 2.0)
    a.on_reservation(westbound(2.0 + 12.0), 2.0)                          # the same path, 12 s later
    assert a.state().mode == RobotMode.NAVIGATING and not of(a.tick(2.0), Navigate)


def test_no_stop_when_there_is_no_place_out_of_its_way():
    a = agent(start=(0.0, -0.8))                                          # already within 1.3 m of its path
    moving(a, 2.0, y=-0.8)
    a.on_reservation(westbound(2.0), 2.0)
    assert not of(a.tick(2.0), Navigate) and not of(a.tick(2.0), CancelNav)
    assert a.state().mode == RobotMode.NAVIGATING


def test_the_wait_is_limited_then_a_cooldown():
    a = agent()
    moving(a, 2.0)
    a.on_reservation(westbound(2.0), 2.0)
    a.tick(2.0)
    t = 2.0 + YIELD_MAX_S
    a.on_pose(0.0, -1.5, math.pi / 2, 0.0, t)
    a.on_reservation(Reservation("robot_1", 0.5, False, ((t, 0.0, 0.0),)), t)   # parked in my way
    assert of(a.tick(t), Navigate)[0].vertices == ("PK_A_2",) and a.yield_events[-1]["timed_out"]
    moving(a, t + 1.0, y=-1.5)
    a.on_reservation(westbound(t + 1.0), t + 1.0)
    assert a.state().mode == RobotMode.NAVIGATING and not of(a.tick(t + 1.0), Navigate)    # cooldown


def test_the_highest_ranked_robot_never_waits():
    a = agent(rid="robot_1")                                              # reserved at 1.0
    moving(a, 2.0)
    a.on_reservation(westbound(2.0, rid="robot_2", reserved=3.0), 2.0)
    a.on_reservation(westbound(2.0, rid="robot_3", reserved=4.0, x0=3.0), 2.0)
    assert not of(a.tick(2.0), Navigate) and a.state().mode == RobotMode.NAVIGATING


def test_stale_reservations_are_ignored():
    a = agent()
    a.on_reservation(westbound(0.0), 0.0)
    moving(a, 3.0)
    assert not of(a.tick(3.0), Navigate)


def test_cancel_while_waiting_ends_the_task_and_the_wait():
    a = agent()
    moving(a, 2.0)
    a.on_reservation(westbound(2.0), 2.0)
    a.tick(2.0)
    acts = a.cancel(3.0)
    assert of(acts, TaskFinished) and a.yielding_to() == "" and a.state().mode == RobotMode.IDLE
    assert a.reservation(3.0) is None                                     # no task: no reservation


# -- deadlock breaker -------------------------------------------------------------------------------------------

def blocked_head_on(unjam=True, my_reserved_later=True):
    """robot_2 blocked at (0, 0) facing east, robot_1 blocked 0.8 m ahead facing west."""
    a = agent(yield_on=False, unjam=unjam, start=(0.0, 0.0))
    a.on_pose(0.0, 0.0, 0.0, 0.0, 2.0)
    a.on_nav_result(False, 2.0)                                           # Nav2 gave up: STUCK / BLOCKED
    assert a.state().mode == RobotMode.STUCK and a.state().fault_reason == api.FAULT_BLOCKED
    return a


def other_blocked(a, t, reserved=0.5):
    a.on_other("robot_1", 0.8, 0.0, math.pi, 0.0, "STUCK", t, fault_reason="BLOCKED")
    a.on_reservation(Reservation("robot_1", reserved, False, ((t, 0.8, 0.0),)), t)


def test_the_lower_ranked_member_of_a_cycle_backs_up_then_replans():
    a = blocked_head_on()
    for k in range(int(HOLD_S) + 1):
        t = 2.0 + k
        a.on_pose(0.0, 0.0, 0.0, 0.0, t)
        other_blocked(a, t)
        acts = a.tick(t)
        if k < HOLD_S:
            assert not of(acts, BackUp)
    assert of(acts, BackUp) and a.state().fault_reason == "UNJAM"
    assert any(e["type"] == "deadlock_cycle" and e["cycle"] == ["robot_1", "robot_2"] for e in a.yield_events)
    nav = of(a.on_backup_result(True, 12.0), Navigate)
    assert nav and nav[0].vertices == ("PK_A_2",) and a.state().mode == RobotMode.NAVIGATING


def test_the_higher_ranked_member_waits_its_turn():
    a = blocked_head_on()
    acts = []
    for k in range(int(HOLD_S) + 2):
        t = 2.0 + k
        a.on_pose(0.0, 0.0, 0.0, 0.0, t)
        other_blocked(a, t, reserved=5.0)                                 # robot_1 reserved later: it goes first
        acts += a.tick(t)
    assert not of(acts, BackUp)


def test_no_breaker_when_switched_off_and_cancel_stops_a_backup():
    a = blocked_head_on(unjam=False)
    for k in range(int(HOLD_S) + 2):
        a.on_pose(0.0, 0.0, 0.0, 0.0, 2.0 + k)
        other_blocked(a, 2.0 + k)
        assert not of(a.tick(2.0 + k), BackUp)
    b = blocked_head_on()
    for k in range(int(HOLD_S) + 1):
        b.on_pose(0.0, 0.0, 0.0, 0.0, 2.0 + k)
        other_blocked(b, 2.0 + k)
        b.tick(2.0 + k)
    assert of(b.cancel(20.0), CancelBackUp)
