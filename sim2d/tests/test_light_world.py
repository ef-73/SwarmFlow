"""T025 light mode: pure kinematic world with simple local avoidance (improvements_v2 X-09 / C-03).
Lead test: do not edit."""

from __future__ import annotations

import math
import pathlib
import random
import sys
import time

ROOT = pathlib.Path(__file__).resolve().parents[2]
for p in (ROOT / "sim2d", ROOT / "src" / "swarmflow_core"):
    sys.path.insert(0, str(p))

from sim2d.light_world import LightWorld  # noqa: E402
from swarmflow_core.metrics.clearance import footprint_distance  # noqa: E402

DT = 0.1


def world(**kw):
    return LightWorld(max_speed=0.5, max_turn=1.5, **kw)


def run(w, until_s, check=None):
    """Step until every robot is idle/arrived or until_s; returns elapsed sim time and min pair distance."""
    t, min_d = 0.0, math.inf
    ids = w.robot_ids()
    while t < until_s:
        w.step(DT)
        t += DT
        poses = {r: w.pose(r) for r in ids}
        for i, a in enumerate(ids):
            for b in ids[i + 1:]:
                min_d = min(min_d, footprint_distance(poses[a], poses[b]))
        if check:
            check(t)
        if all(w.state(r) in ("idle", "arrived") for r in ids):
            break
    return t, min_d


def test_single_robot_reaches_goal_in_expected_time():
    w = world()
    w.add_robot("robot_1", 0.0, 0.0, 0.0)
    w.set_goals("robot_1", [(3.0, 0.0, None)])
    assert w.state("robot_1") == "moving"
    t, _ = run(w, 30.0)
    x, y, _ = w.pose("robot_1")
    assert w.state("robot_1") == "arrived"
    assert math.hypot(x - 3.0, y) < 0.01
    assert 5.9 <= t <= 7.5          # 3 m at 0.5 m/s, plus acceleration slack


def test_goal_behind_turns_in_place_first():
    w = world()
    w.add_robot("robot_1", 0.0, 0.0, 0.0)
    w.set_goals("robot_1", [(-2.0, 0.0, None)])
    xs = []
    t, _ = run(w, 30.0, check=lambda _t: xs.append(w.pose("robot_1")[0]))
    assert w.state("robot_1") == "arrived"
    assert max(xs) < 0.05                       # never drives forward away from the goal
    assert t <= 2.0 / 0.5 + math.pi / 1.5 + 1.5


def test_goal_list_and_final_yaw():
    w = world()
    w.add_robot("robot_1", 0.0, 0.0, 0.0)
    w.set_goals("robot_1", [(1.0, 0.0, None), (1.0, 1.0, math.pi)])
    run(w, 30.0)
    x, y, yaw = w.pose("robot_1")
    assert math.hypot(x - 1.0, y - 1.0) < 0.01
    assert abs(math.atan2(math.sin(yaw - math.pi), math.cos(yaw - math.pi))) < 0.05


def test_cancel_stops_the_robot():
    w = world()
    w.add_robot("robot_1", 0.0, 0.0, 0.0)
    w.set_goals("robot_1", [(10.0, 0.0, None)])
    for _ in range(20):
        w.step(DT)
    w.cancel("robot_1")
    x0 = w.pose("robot_1")[0]
    for _ in range(20):
        w.step(DT)
    assert w.state("robot_1") == "idle" and w.speed("robot_1") == 0.0
    assert abs(w.pose("robot_1")[0] - x0) < 0.06   # may finish one braking step


def test_crossing_paths_do_not_overlap():
    w = world()
    w.add_robot("robot_1", 0.0, 0.0, 0.0)
    w.add_robot("robot_2", 3.0, -3.0, math.pi / 2)
    w.set_goals("robot_1", [(6.0, 0.0, None)])
    w.set_goals("robot_2", [(3.0, 3.0, None)])
    t, min_d = run(w, 60.0)
    assert w.state("robot_1") == "arrived" and w.state("robot_2") == "arrived"
    assert min_d >= 0.0


def test_head_on_resolves_without_overlap():
    w = world()
    w.add_robot("robot_1", 0.0, 0.0, 0.0)
    w.add_robot("robot_2", 6.0, 0.0, math.pi)
    w.set_goals("robot_1", [(6.0, 0.0, None)])
    w.set_goals("robot_2", [(0.0, 0.0, None)])
    t, min_d = run(w, 90.0)
    assert w.state("robot_1") == "arrived" and w.state("robot_2") == "arrived", t
    assert min_d >= 0.0


def test_follower_keeps_distance():
    w = world()
    w.add_robot("robot_1", 1.0, 0.0, 0.0)
    w.add_robot("robot_2", 0.0, 0.0, 0.0)           # 1 m behind, footprints 0.30 m apart
    w.set_goals("robot_1", [(8.0, 0.0, None)])
    w.set_goals("robot_2", [(7.0, 0.0, None)])
    _, min_d = run(w, 60.0)
    assert min_d >= 0.0
    assert w.state("robot_1") == "arrived" and w.state("robot_2") == "arrived"


def test_blocked_state_reported_while_waiting():
    w = world()
    w.add_robot("robot_1", 0.0, 0.0, 0.0)
    w.add_robot("robot_2", 1.0, 0.0, 0.0)           # parked in the way, no goals
    w.set_goals("robot_1", [(3.0, 0.0, None)])
    states = []
    run(w, 3.0, check=lambda _t: states.append(w.state("robot_1")))
    assert "blocked" in states


def _random_world(seed):
    rng = random.Random(seed)
    w = world()
    for i in range(10):
        w.add_robot(f"robot_{i + 1}", 2.0 * i, 0.0, 0.0)
    for i in range(10):
        w.set_goals(f"robot_{i + 1}", [(rng.uniform(0, 20), rng.uniform(-8, 8), None) for _ in range(3)])
    return w


def test_deterministic():
    traces = []
    for _ in range(2):
        w = _random_world(7)
        tr = []
        for _ in range(600):
            w.step(DT)
            tr.append(tuple(round(c, 9) for r in w.robot_ids() for c in w.pose(r)))
        traces.append(tr)
    assert traces[0] == traces[1]


def test_ten_robots_run_much_faster_than_real_time():
    w = _random_world(3)
    t0 = time.process_time()
    for _ in range(6000):           # 600 sim-seconds
        w.step(DT)
    assert time.process_time() - t0 < 20.0      # ≤ 1/30 of a core in real time
