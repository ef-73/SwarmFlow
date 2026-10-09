"""Own tests for sim2d kinematics and protocol wiring."""

from __future__ import annotations

import math
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parents[2]
for p in (ROOT / "sim2d", ROOT / "src" / "swarmflow_robot_agent"):
    sys.path.insert(0, str(p))

from sim2d import Sim2DBackend  # noqa: E402
from swarmflow_core import api  # noqa: E402
from swarmflow_core.api import Assignment  # noqa: E402

LAYOUT = ROOT / "layouts" / "standard" / "generated" / "sim2d.json"


def _drive(b, t0, seconds, dt=0.1):
    t = t0
    for _ in range(int(seconds / dt)):
        t = round(t + dt, 9)
        b.step(t, dt)
    return t


def test_park_move_reaches_goal_and_reports_result():
    b = Sim2DBackend.from_json(LAYOUT, robots=["robot_1"])
    g = b.graph
    route = list(g.shortest_path("P1", "L1"))
    b.dispatch(Assignment("robot_1", "", "park-1", route))
    assert b.snapshot().robots[0].mode == api.RobotMode.NAVIGATING
    _drive(b, 0.0, 120.0)
    res = b.poll_results()
    assert [(r.task_id, r.success) for r in res] == [("park-1", True)]
    s = b.snapshot().robots[0]
    v = g.vertices["L1"]
    assert math.hypot(s.x - v.x, s.y - v.y) < 1e-6
    assert s.mode == api.RobotMode.IDLE and s.last_vertex == "L1"


def test_speed_is_respected():
    b = Sim2DBackend.from_json(LAYOUT, robots=["robot_1"], speed_mps=0.5)
    route = list(b.graph.shortest_path("P1", "L1"))
    length = b.graph.path_length(route)
    b.dispatch(Assignment("robot_1", "", "park-1", route))
    t = 0.0
    while not b.poll_results() and t < 300.0:
        t = _drive(b, t, 0.1)
    assert length / 0.5 <= t <= length / 0.5 + 20.0  # plus turning time


def test_cancel_stops_robot():
    b = Sim2DBackend.from_json(LAYOUT, robots=["robot_1"])
    route = list(b.graph.shortest_path("P1", "L1"))
    b.dispatch(Assignment("robot_1", "", "park-1", route))
    t = _drive(b, 0.0, 5.0)
    b.cancel("park-1")
    s1 = b.snapshot().robots[0]
    _drive(b, t, 5.0)
    s2 = b.snapshot().robots[0]
    assert (s1.x, s1.y) == (s2.x, s2.y) and s2.mode == api.RobotMode.IDLE
    assert [(r.task_id, r.success, r.failure_reason) for r in b.poll_results()] == [
        ("park-1", False, api.FAILURE_CANCELLED)]
