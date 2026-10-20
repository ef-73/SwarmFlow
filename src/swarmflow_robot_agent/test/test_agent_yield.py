"""T067: the agent yields to another robot about to meet it (switch SWARMFLOW_YIELD), and says so."""

from __future__ import annotations

import math

from swarmflow_core import api
from swarmflow_core.api import RobotMode
from swarmflow_robot_agent.agent_core import AgentCore, CancelNav, Navigate, TaskFinished
from swarmflow_robot_agent.yield_policy import YIELD_COOLDOWN_S, YIELD_MAX_S
from test_agent_goals import at, graph, of  # noqa: E402  (shared fixture graph)

GOAL = (2.0, 5.0)                                      # PK_A_2
START = (0.0, -1.0)
BEARING = math.atan2(GOAL[1] - START[1], GOAL[0] - START[0])


def agent(enabled=True, rid="robot_2"):
    a = AgentCore(rid, graph(), traffic_control=True, yield_to_others=enabled)
    at(a, "HM1", 0.0)
    a.on_orchestrator_heartbeat(t=0.0)
    a.start_task("task_1", "s1", ["PK_A_2"], 1.0, route_kind=api.ROUTE_GOALS, order_type=api.ORDER_SHELF)
    return a


def drive(a, t, d=0.0):
    a.on_pose(START[0] + d * math.cos(BEARING), START[1] + d * math.sin(BEARING), BEARING, 0.5, t)


def oncoming(a, t, ahead=2.5, rid="robot_1", speed=0.5, mode="NAVIGATING"):
    """Another robot ``ahead`` metres along this robot's way, driving straight at it."""
    x, y = START[0] + ahead * math.cos(BEARING), START[1] + ahead * math.sin(BEARING)
    a.on_other(rid, x, y + 0.1, BEARING + math.pi, speed, mode, t)


def test_off_by_default_never_yields():
    a = agent(enabled=False)
    drive(a, 2.0)
    oncoming(a, 2.0)
    assert not of(a.tick(2.0), CancelNav) and a.state().mode == RobotMode.NAVIGATING


def test_head_on_yields_says_so_and_resumes_the_same_goal_when_passed():
    a = agent()
    drive(a, 2.0)
    oncoming(a, 2.0)
    acts = a.tick(2.0)
    s = a.state()
    assert of(acts, CancelNav) and s.mode == RobotMode.WAITING_RESERVATION and s.fault_reason == "YIELD:robot_1"
    assert a.yielding_to() == "robot_1" and a.yield_events[-1]["type"] == "yield_start"
    a.on_pose(START[0], START[1], BEARING, 0.0, 3.0)
    oncoming(a, 3.0, ahead=1.2)                                       # still coming: wait on
    assert a.tick(3.0) == [] and a.state().mode == RobotMode.WAITING_RESERVATION
    a.on_other("robot_1", START[0] - 1.0, START[1], BEARING + math.pi, 0.5, "NAVIGATING", 4.0)   # passed
    acts = a.tick(4.0)
    nav = of(acts, Navigate)
    assert nav and nav[0].vertices == ("PK_A_2",)
    s = a.state()
    assert s.mode == RobotMode.NAVIGATING and s.fault_reason == "" and a.yield_events[-1]["type"] == "yield_end"


def test_wait_is_limited_then_no_yield_to_the_same_robot_during_the_cooldown():
    a = agent()
    drive(a, 2.0)
    oncoming(a, 2.0)
    a.tick(2.0)
    for k in range(1, int(YIELD_MAX_S) + 1):                          # the other robot keeps coming
        a.on_pose(START[0], START[1], BEARING, 0.0, 2.0 + k)
        oncoming(a, 2.0 + k)
        acts = a.tick(2.0 + k)
    assert of(acts, Navigate) and a.state().mode == RobotMode.NAVIGATING
    t = 2.0 + YIELD_MAX_S + 1.0
    drive(a, t)
    oncoming(a, t)
    assert not of(a.tick(t), CancelNav)                               # cooldown
    t = 2.0 + YIELD_MAX_S + YIELD_COOLDOWN_S + 0.5
    drive(a, t)
    oncoming(a, t)
    assert of(a.tick(t), CancelNav)


def test_no_yield_to_a_waiting_robot_so_two_robots_never_wait_for_each_other():
    a = agent()
    drive(a, 2.0)
    oncoming(a, 2.0, mode="WAITING_RESERVATION")
    assert not of(a.tick(2.0), CancelNav)


def test_lower_id_does_not_yield_head_on():
    a = agent(rid="robot_1")
    drive(a, 2.0)
    oncoming(a, 2.0, rid="robot_2")
    assert not of(a.tick(2.0), CancelNav) and a.state().mode == RobotMode.NAVIGATING


def test_stale_other_state_is_ignored():
    a = agent()
    oncoming(a, 0.5)
    drive(a, 2.0)
    assert not of(a.tick(2.0), CancelNav)


def test_no_yield_on_the_way_into_the_dock_line():
    a = AgentCore("robot_2", graph(), traffic_control=True, yield_to_others=True)
    at(a, "HM1", 0.0)
    a.on_orchestrator_heartbeat(t=0.0)
    a.start_task("task_1", "s1", ["Q_DK1_1"], 1.0, route_kind=api.ROUTE_GOALS, order_type=api.ORDER_SHELF)
    b = math.atan2(2.4 - (-2.0), 6.0)
    a.on_pose(0.0, -2.0, b, 0.5, 2.0)
    a.on_other("robot_1", 2.0 * math.cos(b), -2.0 + 2.0 * math.sin(b), b + math.pi, 0.5, "NAVIGATING", 2.0)
    assert not of(a.tick(2.0), CancelNav)


def test_cancel_while_yielding_ends_the_task_and_the_yield():
    a = agent()
    drive(a, 2.0)
    oncoming(a, 2.0)
    a.tick(2.0)
    acts = a.cancel(3.0)
    assert of(acts, TaskFinished) and not of(acts, CancelNav)          # nothing in flight any more
    assert a.yielding_to() == "" and a.state().mode == RobotMode.IDLE and a.state().fault_reason == ""
