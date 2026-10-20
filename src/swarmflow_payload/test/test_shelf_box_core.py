"""T066: boxes of v2 shelf orders appear on the robot at each pick and leave at the dock."""

from __future__ import annotations

import pytest

from swarmflow_payload.payload_core import PLATFORM_Z
from swarmflow_payload.shelf_box_core import BOX_H, ShelfBoxCore

BOUNDS = (0.0, 0.0, 20.0, 32.0)


def st(mode, order="o1", last="", x=1.0, y=2.0, yaw=0.5, rid="robot_1"):
    return {"robot_id": rid, "mode": mode, "order_id": order, "last_vertex": last, "x": x, "y": y, "yaw": yaw}


@pytest.fixture
def core():
    c = ShelfBoxCore(BOUNDS)
    c.on_order("o1", ["PK_1", "PK_2", "PK_3"])
    return c


def test_a_box_appears_on_the_deck_at_each_pick_stacked(core):
    core.on_robot_state(st("NAVIGATING", last="HM1"), 1.0)
    assert core.tick(1.0) == {} and core.boxes_of("robot_1") == []
    core.on_robot_state(st("NAVIGATING", last="PK_1"), 2.0)
    out = core.tick(2.0)
    assert out == {"pkg_00": pytest.approx((1.0, 2.0, PLATFORM_Z, 0.5))}
    core.on_robot_state(st("NAVIGATING", last="PK_1", x=3.0), 3.0)          # same pick again: no second box
    core.on_robot_state(st("NAVIGATING", last="PK_2", x=4.0), 4.0)
    core.on_robot_state(st("NAVIGATING", last="PK_3", x=5.0), 5.0)
    out = core.tick(5.0)
    assert [out[p][2] for p in core.boxes_of("robot_1")] == pytest.approx(
        [PLATFORM_Z, PLATFORM_Z + BOX_H, PLATFORM_Z + 2 * BOX_H])
    assert all(out[p][:2] == pytest.approx((5.0, 2.0)) for p in core.boxes_of("robot_1"))   # follows the robot
    assert core.payload_state("robot_1")["loaded"]


def test_no_box_for_other_vertices_or_unknown_orders(core):
    core.on_robot_state(st("NAVIGATING", order="zz", last="PK_1"), 1.0)
    core.on_robot_state(st("NAVIGATING", order="o1", last="X_1_5", rid="robot_2"), 1.0)
    assert core.tick(1.0) == {}


def test_boxes_leave_when_unloading_ends_and_stuck_keeps_them(core):
    core.on_robot_state(st("NAVIGATING", last="PK_1"), 1.0)
    core.on_robot_state(st("STUCK", last="PK_1"), 2.0)                        # blocked: boxes stay on board
    assert core.boxes_of("robot_1") == ["pkg_00"]
    core.on_robot_state(st("UNLOADING", last="DK1"), 10.0)
    assert core.boxes_of("robot_1") == ["pkg_00"]
    core.on_robot_state(st("IDLE", order="", last="DK1"), 15.0)
    assert core.boxes_of("robot_1") == []
    out = core.tick(15.0)
    assert out["pkg_00"][0] == pytest.approx(-3.0)                             # back in the pool
    assert not core.payload_state("robot_1")["loaded"]


def test_requeued_order_and_new_order_release_the_boxes(core):
    core.on_order("o2", ["PK_9"])
    core.on_robot_state(st("NAVIGATING", last="PK_1"), 1.0)
    core.on_order_state("o1", "IN_TRANSIT")
    assert core.boxes_of("robot_1") == ["pkg_00"]
    core.on_order_state("o1", "QUEUED")
    assert core.boxes_of("robot_1") == []
    core.on_robot_state(st("NAVIGATING", order="o2", last="PK_1"), 2.0)       # last_vertex still the old pick
    assert core.boxes_of("robot_1") == []
    core.on_robot_state(st("NAVIGATING", order="o2", last="PK_9"), 3.0)
    assert core.boxes_of("robot_1") == ["pkg_00"]


def test_idle_between_tasks_keeps_the_boxes(core):
    core.on_robot_state(st("NAVIGATING", last="PK_1"), 1.0)
    core.on_robot_state(st("IDLE", order="", last="PK_1"), 2.0)               # task replaced (dock line)
    core.on_robot_state(st("NAVIGATING", order="o1", last="PK_1"), 3.0)
    assert core.boxes_of("robot_1") == ["pkg_00"]


def test_pool_exhaustion_warns():
    c = ShelfBoxCore(BOUNDS, pool_size=1)
    c.on_order("o1", ["PK_1", "PK_2"])
    c.on_robot_state(st("NAVIGATING", last="PK_1"), 1.0)
    c.on_robot_state(st("NAVIGATING", last="PK_2"), 2.0)
    assert c.boxes_of("robot_1") == ["pkg_00"] and "exhausted" in c.warnings[0]
