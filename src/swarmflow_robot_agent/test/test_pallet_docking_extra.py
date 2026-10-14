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
