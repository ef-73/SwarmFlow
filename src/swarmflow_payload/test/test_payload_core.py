"""Package pose-follower core (design §7.4). Lead test: do not edit.

Under test: ``swarmflow_payload.payload_core.PayloadCore`` — pure Python, time passed in, no ROS / gz imports.
"""

from __future__ import annotations

import math
import pathlib

import pytest

from swarmflow_core import api
from swarmflow_core.graph import load_layout
from swarmflow_payload.payload_core import PLATFORM_Z, POOL_SIZE, PayloadCore

ROOT = pathlib.Path(__file__).resolve().parents[3]
GEN = ROOT / "layouts" / "standard" / "generated"
GRAPH = load_layout(GEN)
BOUNDS = (0.0, 0.0, 22.0, 12.3)


def core():
    return PayloadCore(GRAPH, bounds=BOUNDS, drop_offsets={"D1": 0.9, "D2": 0.9, "D3": 0.9})


def state(rid, mode, order_id="", t=0.0):
    return {"robot_id": rid, "mode": mode, "order_id": order_id}


def test_pool_parked_outside():
    c = core()
    poses = c.initial_poses()
    assert sorted(poses) == [f"pkg_{i:02d}" for i in range(POOL_SIZE)] and POOL_SIZE == 12
    for name, (x, y, z, yaw) in poses.items():
        assert x < BOUNDS[0] and z == pytest.approx(0.15)


def test_load_follow_unload_recycle():
    c = core()
    L1 = GRAPH.vertices["L1"]
    c.on_robot_pose("robot_1", L1.x, L1.y, math.pi, t=10.0)
    c.on_robot_state(state("robot_1", "LOADING", "o1"), t=10.0)
    ps = c.payload_state("robot_1")
    assert ps["loaded"] and ps["order_id"] == "o1" and ps["payload_type"] == "small"
    assert (ps["size_x"], ps["size_y"]) == pytest.approx((0.40, 0.40))
    assert len(ps["footprint"]) == 4 and max(abs(p[0]) for p in ps["footprint"]) == pytest.approx(0.35)
    pkg = c.package_of("robot_1")
    assert pkg == "pkg_00"
    upd = c.tick(10.05)
    assert upd[pkg] == pytest.approx((L1.x, L1.y, PLATFORM_Z, math.pi))
    c.on_robot_pose("robot_1", 5.0, 3.85, 0.0, t=20.0)
    assert c.tick(20.0)[pkg] == pytest.approx((5.0, 3.85, PLATFORM_Z, 0.0))      # follows the ground-truth pose
    D1 = GRAPH.vertices["D1"]
    c.on_robot_pose("robot_1", D1.x, D1.y, 0.0, t=40.0)
    c.on_robot_state(state("robot_1", "UNLOADING", "o1"), t=40.0)
    assert c.tick(41.0)[pkg][:2] == pytest.approx((D1.x, D1.y))                   # still on the robot during dwell
    up = c.tick(40.0 + api.LOAD_DWELL_S)
    x, y, z, yaw = up[pkg]
    assert (x, y) == pytest.approx((D1.x + 0.9, D1.y)) and z == pytest.approx(0.15)  # drop pose, on the floor
    assert not c.payload_state("robot_1")["loaded"]
    assert pkg not in c.tick(45.0)                                                    # no updates while lying there
    back = c.tick(40.0 + api.LOAD_DWELL_S + 10.0)
    assert back[pkg][0] < BOUNDS[0]                                                   # recycled after 10 s visible
    c.on_robot_state(state("robot_2", "LOADING", "o2"), t=60.0)
    assert c.package_of("robot_2") in [f"pkg_{i:02d}" for i in range(POOL_SIZE)]


def test_repeated_loading_state_does_not_take_second_package():
    c = core()
    c.on_robot_pose("robot_1", 1.3, 3.85, 0.0, t=0.0)
    for k in range(10):
        c.on_robot_state(state("robot_1", "LOADING", "o1"), t=0.2 * k)
    assert sum(1 for r in ("robot_1",) if c.package_of(r)) == 1
    assert len([n for n, p in c.tick(2.0).items()]) == 1


def test_cancelled_task_drops_package_back_to_pool():
    c = core()
    c.on_robot_pose("robot_1", 1.3, 3.85, 0.0, t=0.0)
    c.on_robot_state(state("robot_1", "LOADING", "o1"), t=0.0)
    pkg = c.package_of("robot_1")
    c.on_robot_state(state("robot_1", "STUCK", ""), t=30.0)       # task ended without unloading
    c.on_robot_state(state("robot_1", "IDLE", ""), t=31.0)
    up = c.tick(31.0)
    assert up[pkg][0] < BOUNDS[0] and c.package_of("robot_1") is None


def test_pool_exhaustion_is_reported_not_crashing():
    c = core()
    for i in range(POOL_SIZE + 2):
        rid = f"robot_{i + 1}"
        c.on_robot_pose(rid, 1.3, 3.85, 0.0, t=0.0)
        c.on_robot_state(state(rid, "LOADING", f"o{i}"), t=0.0)
    assert sum(1 for i in range(POOL_SIZE + 2) if c.package_of(f"robot_{i + 1}")) == POOL_SIZE
    assert c.warnings and "pool" in c.warnings[-1]
