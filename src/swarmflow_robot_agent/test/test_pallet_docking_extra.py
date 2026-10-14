"""T042 extra tests: dock-drive control law (bounds, convergence in a unicycle simulation) and edge cases of the
pallet docking phases in the core."""

from __future__ import annotations

import math

import pytest

from swarmflow_core import api
from swarmflow_core.api import RobotMode
from swarmflow_robot_agent.agent_core import (DOCK_AHEAD_M, PALLET_LIFT_S, UNDOCK_M, AgentCore, CancelNav, DockDrive,
                                              Navigate, SetFootprint, StopDrive, TaskFinished)
from test_pallet_docking import N, graph  # noqa: E402  (shared fixture graph)

from swarmflow_robot_agent import agent_core as an  # noqa: E402


def of(acts, kind):
    return [a for a in acts if isinstance(a, kind)]


@pytest.fixture
def agent():
    a = AgentCore("robot_1", graph(), traffic_control=False)
    a.on_pose(0.0, -2.0, N, 0.0, 0.0)
    a.on_orchestrator_heartbeat(t=0.0)
    return a


def docked(a, t=10.0):
    a.start_task("t1", "p1", ["PL_1", "DK1"], 1.0, route_kind=api.ROUTE_GOALS, order_type=api.ORDER_PALLET)
    a.on_pose(0.0, 2.8, N, 0.0, t)
    return a.on_nav_result(True, t)


# -- control law --------------------------------------------------------------------------------------------------


def simulate(pose, target, reverse, steps=4000, dt=0.05):
    x, y, yaw = pose
    vmax = wmax = 0.0
    for _ in range(steps):
        v, w = an.drive_command((x, y, yaw), target, reverse)
        vmax, wmax = max(vmax, abs(v)), max(wmax, abs(w))
        x += v * math.cos(yaw) * dt
        y += v * math.sin(yaw) * dt
        yaw += w * dt
        if v == 0.0 and abs(w) < 1e-3:
            break
    return (x, y, yaw), vmax, wmax


@pytest.mark.parametrize("reverse", [False, True])
@pytest.mark.parametrize("offset", [(0.0, 0.0), (0.08, 0.0), (-0.1, 0.05), (0.15, -0.1)])
def test_drive_converges_within_tolerance_and_respects_limits(reverse, offset):
    target = (0.0, 4.0, N)
    start = (offset[0], (5.2 if reverse else 2.8) + offset[1], N + 2.0 * offset[1])
    (x, y, yaw), vmax, wmax = simulate(start, target, reverse)
    assert vmax <= an.DRIVE_MAX_MPS + 1e-9 and wmax <= an.DRIVE_MAX_RADPS + 1e-9
    assert abs(x - target[0]) <= 0.03 and abs(y - target[1]) <= 0.02 + 0.005


def test_drive_command_is_zero_at_target_and_signed_by_direction():
    assert an.drive_command((0.0, 4.0, N), (0.0, 4.0, N), False)[0] == 0.0
    assert an.drive_command((0.0, 3.0, N), (0.0, 4.0, N), False)[0] > 0.0
    assert an.drive_command((0.0, 5.0, N), (0.0, 4.0, N), True)[0] < 0.0
    assert an.drive_command((0.0, 4.5, N), (0.0, 4.0, N), False)[0] == 0.0     # passed the target: never back up


def test_drive_command_turns_in_place_when_heading_is_far_off():
    v, w = an.drive_command((0.0, 3.0, 0.0), (0.0, 4.0, N), False)
    assert v == 0.0 and w == pytest.approx(an.DRIVE_MAX_RADPS)


# -- core edge cases ------------------------------------------------------------------------------------------------


def test_nav_result_during_the_drive_is_ignored(agent):
    docked(agent)
    assert agent.driving()
    assert agent.on_nav_result(True, 11.0) == [] and agent.on_nav_result(False, 11.5) == []
    assert agent.driving()


def test_overshoot_with_lateral_error_does_not_stop(agent):
    docked(agent)
    assert not of(agent.on_pose(0.1, 4.0, N, 0.05, 11.0), StopDrive)      # 10 cm off to the side: not reached
    assert of(agent.on_pose(0.0, 4.05, N, 0.05, 12.0), StopDrive)         # passed the target in line: stop


def test_drive_ends_with_stop_and_driving_flag_clears(agent):
    docked(agent)
    agent.on_pose(0.0, 4.0, N, 0.0, 12.0)
    assert not agent.driving()


def test_cancel_during_lift_dwell_and_unload_has_no_drive_actions(agent):
    docked(agent)
    agent.on_pose(0.0, 4.0, N, 0.0, 12.0)                                 # lifting dwell
    acts = agent.cancel(13.0)
    assert not of(acts, DockDrive) and not of(acts, CancelNav) and of(acts, TaskFinished)


def test_cancel_while_backing_out_stops_and_restores_footprint(agent):
    docked(agent)
    agent.on_pose(0.0, 4.0, N, 0.0, 12.0)
    agent.tick(12.0 + PALLET_LIFT_S)
    agent.on_pose(6.0, 8.0, N, 0.0, 40.0)
    agent.on_nav_result(True, 40.0)
    agent.on_pose(6.0, 8.0 + DOCK_AHEAD_M, N, 0.0, 45.0)
    acts = agent.tick(45.0 + api.UNLOAD_DWELL_PALLET_S)
    assert of(acts, DockDrive)[0].reverse
    acts = agent.cancel(60.0)
    assert of(acts, StopDrive) and [a.kind for a in of(acts, SetFootprint)] == ["normal"]


def test_dock_timeout_on_the_way_back_restores_normal_footprint(agent):
    docked(agent)
    agent.on_pose(0.0, 4.0, N, 0.0, 12.0)
    agent.tick(12.0 + PALLET_LIFT_S)
    agent.on_pose(6.0, 8.0, N, 0.0, 40.0)
    agent.on_nav_result(True, 40.0)
    agent.on_pose(6.0, 8.0 + DOCK_AHEAD_M, N, 0.0, 45.0)
    agent.tick(45.0 + api.UNLOAD_DWELL_PALLET_S)
    acts = []
    t0 = 56.0
    for k in range(40):
        agent.on_pose(6.0, 8.0 + DOCK_AHEAD_M, N, 0.0, t0 + k)
        acts += agent.tick(t0 + k)
    done = of(acts, TaskFinished)
    assert done and not done[0].success and done[0].failure_reason == api.FAILURE_NAV_PREFIX + "DK1"
    assert of(acts, StopDrive) and [a.kind for a in of(acts, SetFootprint)] == ["normal"]


def test_progress_resets_the_drive_timeout(agent):
    docked(agent, t=10.0)
    acts = []
    y = 2.8
    for k in range(60):                                                    # 1 cm/s crawl: slow but progressing
        y = min(3.8, y + 0.01)
        t = 10.0 + k
        acts += agent.on_pose(0.0, y, N, 0.01, t)
        acts += agent.tick(t)
    assert not of(acts, TaskFinished)


# -- review fixes --------------------------------------------------------------------------------------------------


def to_unload(a):
    docked(a)
    a.on_pose(0.0, 4.0, N, 0.0, 12.0)
    a.tick(12.0 + PALLET_LIFT_S)
    a.on_pose(6.0, 8.0, N, 0.0, 40.0)
    a.on_nav_result(True, 40.0)
    return a.on_pose(6.0, 8.0 + DOCK_AHEAD_M, N, 0.0, 45.0)


def test_cancel_during_unload_restores_normal_footprint(agent):
    to_unload(agent)
    assert agent.state().mode == RobotMode.UNLOADING
    acts = agent.cancel(46.0)
    assert [a.kind for a in of(acts, SetFootprint)] == ["normal"]
    assert agent.state().mode == RobotMode.IDLE                          # pallet is down: nothing on board


def carry_then_cancel(a):
    docked(a)
    a.on_pose(0.0, 4.0, N, 0.0, 12.0)
    a.tick(12.0 + PALLET_LIFT_S)
    return a.cancel(20.0)


def test_pallet_stays_on_board_after_cancel_and_blocks_other_tasks(agent):
    carry_then_cancel(agent)
    s = agent.state()
    assert s.mode == RobotMode.CARRYING and s.task_id == "" and agent.can_accept()
    acts = agent.start_task("t9", "s1", ["PK_A_1"], 30.0, route_kind=api.ROUTE_GOALS, order_type=api.ORDER_SHELF)
    assert len(acts) == 1 and isinstance(acts[0], TaskFinished)
    assert not acts[0].success and acts[0].failure_reason == "PALLET_ON_BOARD"
    assert agent.state().task_id == "" and agent.state().mode == RobotMode.CARRYING
    acts = agent.start_task("t8", "p2", ["PL_1"], 31.0, route_kind=api.ROUTE_GOALS, order_type=api.ORDER_PALLET)
    assert acts[0].failure_reason == "PALLET_ON_BOARD"                    # a second pallet is refused too
    acts = agent.start_task("t7", "", ["DK1"], 31.5, route_kind=api.ROUTE_WAYPOINTS)
    assert acts[0].failure_reason == "PALLET_ON_BOARD"


def test_follow_up_to_the_dock_is_accepted_after_cancel_and_finishes_the_unload(agent):
    carry_then_cancel(agent)
    acts = agent.start_task("t2", "p1", ["DK1"], 30.0, route_kind=api.ROUTE_GOALS, order_type=api.ORDER_PALLET)
    assert of(acts, Navigate)[0].vertices == ("DK1",) and agent.state().mode == RobotMode.CARRYING
    agent.on_pose(6.0, 8.0, N, 0.0, 40.0)
    dd = of(agent.on_nav_result(True, 40.0), DockDrive)
    assert dd and not dd[0].reverse
    agent.on_pose(6.0, 8.0 + DOCK_AHEAD_M, N, 0.0, 45.0)
    agent.tick(45.0 + api.UNLOAD_DWELL_PALLET_S)
    acts = agent.on_pose(6.0, 8.0 + DOCK_AHEAD_M - UNDOCK_M, N, 0.0, 62.0)
    assert of(acts, TaskFinished)[0].success and agent.state().mode == RobotMode.IDLE
    assert agent.start_task("t3", "s1", ["PK_A_1"], 70.0, route_kind=api.ROUTE_GOALS,
                            order_type=api.ORDER_SHELF)[0].__class__ is Navigate


def test_pallet_on_board_survives_a_failed_task_as_stuck(agent):
    carry_then_cancel(agent)
    agent.start_task("t2", "p1", ["DK1"], 30.0, route_kind=api.ROUTE_GOALS, order_type=api.ORDER_PALLET)
    agent.on_pose(6.0, 8.0, N, 0.0, 40.0)
    agent.on_nav_result(True, 40.0)
    acts = []
    for k in range(40):                                                   # stuck in front of the dock
        agent.on_pose(6.0, 8.0, N, 0.0, 41.0 + k)
        acts += agent.tick(41.0 + k)
    assert of(acts, TaskFinished) and not of(acts, SetFootprint)
    assert agent.state().mode == RobotMode.STUCK
    assert agent.start_task("t3", "s1", ["PK_A_1"], 90.0, route_kind=api.ROUTE_GOALS,
                            order_type=api.ORDER_SHELF)[0].failure_reason == "PALLET_ON_BOARD"


def test_lateral_error_at_the_stop_distance_stalls_and_times_out(agent):
    docked(agent)
    pose = (0.06, 3.99, N)                                                # 6 cm off the line, 1 cm short
    v, w = an.drive_command(pose, (0.0, 4.0, N), False)
    assert v != 0.0 or w != 0.0                                           # (steers toward the line while creeping)
    acts = []
    for k in range(int(an.DOCK_TIMEOUT_S) + 2):                           # stalled: never reached, no StopDrive-as-success
        acts += agent.on_pose(*pose[:2], pose[2], 0.0, 10.0 + k)
        acts += agent.tick(10.0 + k)
    done = of(acts, TaskFinished)
    assert done and not done[0].success and done[0].failure_reason == api.FAILURE_NAV_PREFIX + "PL_1"
    assert agent.state().mode == RobotMode.STUCK


def test_non_finite_input_gives_zero_command():
    nan, inf = float("nan"), float("inf")
    for pose in ((nan, 3.0, N), (0.0, inf, N), (0.0, 3.0, nan)):
        assert an.drive_command(pose, (0.0, 4.0, N), False) == (0.0, 0.0)
    assert an.drive_command((0.0, 3.0, N), (0.0, nan, N), True) == (0.0, 0.0)


def test_footprint_sync_retries_until_success():
    fp = an.FootprintSync(("local", "global"), period_s=1.0)
    assert fp.due(0.0) == [] and not fp.pending()
    fp.set_wanted("carry")
    due = fp.due(0.0)
    assert sorted(due) == [("global", "carry"), ("local", "carry")]
    for n, _ in due:
        fp.sent(n, 0.0)
    assert fp.due(0.5) == []                                               # throttled
    fp.done("local", "carry", True)
    fp.done("global", "carry", False)                                     # failed: stays pending
    assert fp.due(1.0) == [("global", "carry")] and fp.pending()
    fp.sent("global", 1.0)
    fp.done("global", "normal", True)                                      # stale reply for another kind: ignored
    assert fp.pending()
    fp.set_wanted("normal")                                                # a new wish is sent at once
    assert len(fp.due(1.1)) == 2
    for n in ("local", "global"):
        fp.done(n, "normal", True)
    assert not fp.pending() and fp.due(9.0) == []
