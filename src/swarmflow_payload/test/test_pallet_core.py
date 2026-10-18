"""T041: pallet follower core (F-03, F-04 payload state)."""

from __future__ import annotations

import math

import pytest

from swarmflow_core import api
from swarmflow_payload.pallet_core import CARRY_FOOTPRINT, LIFT_Z, PalletCore

SPOTS = {"pallet_PL_W1": (2.5, 5.5, 0.0, math.pi / 2), "pallet_PL_E1": (14.5, 5.5, 0.0, math.pi / 2)}


def st(rid, mode, order="", x=0.0, y=0.0, yaw=0.0):
    return {"robot_id": rid, "mode": mode, "order_id": order, "x": x, "y": y, "yaw": yaw}


@pytest.fixture
def core():
    c = PalletCore(SPOTS)
    c.on_order("p1", api.ORDER_PALLET, "pallet_PL_W1")
    c.on_order("s1", api.ORDER_SHELF, "")
    return c


def test_initial_poses_are_the_stands(core):
    assert core.initial_poses() == dict(SPOTS)


def test_lifting_does_not_move_carrying_follows_raised(core):
    core.on_robot_state(st("robot_1", "LIFTING", "p1", 2.5, 5.5, math.pi / 2), 10.0)
    assert core.tick(10.0) == {}
    core.on_robot_state(st("robot_1", "CARRYING", "p1", 2.5, 5.6, math.pi / 2), 13.0)
    moved = core.tick(13.0)
    assert moved == {"pallet_PL_W1": pytest.approx((2.5, 5.6, LIFT_Z, math.pi / 2))}
    core.on_robot_state(st("robot_1", "CARRYING", "p1", 6.0, 9.0, 0.3), 20.0)
    assert core.tick(20.0)["pallet_PL_W1"] == pytest.approx((6.0, 9.0, LIFT_Z, 0.3))
    ps = core.payload_state("robot_1")
    assert ps["loaded"] and ps["payload_type"] == "pallet" and ps["order_id"] == "p1"
    assert ps["footprint"] == CARRY_FOOTPRINT


def test_set_down_at_the_dock_after_unloading(core):
    core.on_robot_state(st("robot_1", "CARRYING", "p1", 6.5, 30.0, math.pi / 2), 50.0)
    core.tick(50.0)
    core.on_robot_state(st("robot_1", "UNLOADING", "p1", 6.5, 30.6, math.pi / 2), 60.0)
    assert core.tick(60.0)["pallet_PL_W1"] == pytest.approx((6.5, 30.6, LIFT_Z, math.pi / 2))   # still lifted
    core.on_robot_state(st("robot_1", "IDLE", "", 6.5, 30.6, math.pi / 2), 70.5)
    assert core.tick(70.5)["pallet_PL_W1"] == pytest.approx((6.5, 30.6, 0.0, math.pi / 2))        # set down
    assert not core.payload_state("robot_1")["loaded"]
    core.on_robot_state(st("robot_1", "NAVIGATING", "", 6.5, 29.0, -math.pi / 2), 75.0)
    assert core.tick(75.0) == {}                                                                  # stays put


def test_shelf_orders_and_unknown_orders_move_nothing(core):
    core.on_robot_state(st("robot_2", "CARRYING", "s1", 1.0, 1.0), 5.0)
    core.on_robot_state(st("robot_3", "CARRYING", "zz", 1.0, 1.0), 5.0)
    assert core.tick(5.0) == {}
    assert not core.payload_state("robot_2")["loaded"]


def test_two_robots_two_pallets(core):
    core.on_order("p2", api.ORDER_PALLET, "pallet_PL_E1")
    core.on_robot_state(st("robot_1", "CARRYING", "p1", 3.0, 7.0, 0.0), 5.0)
    core.on_robot_state(st("robot_2", "CARRYING", "p2", 15.0, 7.0, 0.0), 5.0)
    moved = core.tick(5.0)
    assert set(moved) == {"pallet_PL_W1", "pallet_PL_E1"}
