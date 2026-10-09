"""Corridor scenario must make robots meet in the 2D backend calibrated to Gazebo timing (M7 finding). Lead test.

Gazebo M7 run 1 showed the fake-backend choreography never produced opposing aisle traffic. The 2D backend drives the
real AgentCore (holds, reservations, dwell) — here slowed to Gazebo-like kinematics.
"""

from __future__ import annotations

import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parents[3]
for p in (ROOT / "sim2d", ROOT / "src" / "swarmflow_robot_agent", ROOT / "tools" / "scenarios"):
    sys.path.insert(0, str(p))

import sim2d.runner as runner  # noqa: E402
from scenarios import generate_orders, load_scenario  # noqa: E402
from swarmflow_core.api import DecisionType  # noqa: E402
from swarmflow_core.graph import load_layout  # noqa: E402

LAYOUT = ROOT / "layouts" / "standard" / "generated" / "sim2d.json"
GRAPH = load_layout(ROOT / "layouts" / "standard" / "generated")
GAZEBO_SPEED = 0.45     # m/s effective (Gazebo: 10 m aisle in ~17 s incl. accel; legs ~1.3x slower than 0.5 m/s ideal)
GAZEBO_TURN = 0.8       # rad/s effective in-place rotation
START_DELAY = 10.0      # s: Gazebo orders start ~10 s after the robots are ready


def run(policy, monkeypatch):
    orig = runner.Sim2DBackend.from_json.__func__

    def slow(cls, path, robots, **kw):
        kw.setdefault("speed_mps", GAZEBO_SPEED)
        kw.setdefault("turn_rate", GAZEBO_TURN)
        return orig(cls, path, robots, **kw)

    monkeypatch.setattr(runner.Sim2DBackend, "from_json", classmethod(slow))
    s = load_scenario(ROOT / "scenarios" / "v1_corridor.yaml")
    orders = [o.__class__(**{**o.__dict__, "release_t": o.release_t + START_DELAY}) for o in generate_orders(s, GRAPH)]
    return s, runner.run(LAYOUT, orders, policy=policy, robots=s.robots, until_t=s.duration_s + START_DELAY, dt=0.1)


def test_fcfs_robots_are_denied_every_minute(monkeypatch):
    s, r = run("fcfs", monkeypatch)
    denies = [d for d in r.decisions if d.decision_type == DecisionType.RESERVATION_DENY]
    minutes = {int((d.t - START_DELAY) // 60) for d in denies}
    assert len(minutes) >= 3, sorted(round(d.t, 1) for d in denies)


def test_independent_robots_meet_inside_an_aisle(monkeypatch):
    s, r = run("independent", monkeypatch)
    assert r.max_robots_in_zone >= 2
