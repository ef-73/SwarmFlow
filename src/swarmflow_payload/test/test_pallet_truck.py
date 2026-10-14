"""T047: a pallet set down at a dock is taken away by the truck after TRUCK_AFTER_S. Lead test: do not edit."""

from __future__ import annotations

import math

import pytest

from swarmflow_core import api
from swarmflow_payload.pallet_core import LIFT_Z, TRUCK_AFTER_S, PalletCore

SPOTS = {"pallet_PL_W1": (2.5, 5.5, 0.0, math.pi / 2), "pallet_PL_E1": (14.5, 5.5, 0.0, math.pi / 2)}


def st(rid, mode, order="", x=0.0, y=0.0, yaw=0.0):
    return {"robot_id": rid, "mode": mode, "order_id": order, "x": x, "y": y, "yaw": yaw}


def deliver(core, order, pallet, dock_xy, t0):
    core.on_order(order, api.ORDER_PALLET, pallet)
    core.on_robot_state(st("robot_1", "CARRYING", order, *dock_xy, math.pi / 2), t0)
    core.tick(t0)
    core.on_robot_state(st("robot_1", "UNLOADING", order, *dock_xy, math.pi / 2), t0 + 1)
    core.tick(t0 + 1)
    core.on_robot_state(st("robot_1", "IDLE", "", *dock_xy, math.pi / 2), t0 + 11)
    return core.tick(t0 + 11)


def test_truck_takes_the_pallet_after_the_delay():
    assert TRUCK_AFTER_S == 30.0
    core = PalletCore(SPOTS, pool_origin=(-3.0, 0.0))
    moved = deliver(core, "p1", "pallet_PL_W1", (2.0, 31.3), 100.0)
    assert moved["pallet_PL_W1"] == pytest.approx((2.0, 31.3, 0.0, math.pi / 2))      # set down at 111
    assert core.tick(111.0 + TRUCK_AFTER_S - 0.1) == {}
    gone = core.tick(111.0 + TRUCK_AFTER_S)
    x, y, z, _ = gone["pallet_PL_W1"]
    assert x == pytest.approx(-3.0) and z == pytest.approx(0.0)
    assert core.tick(111.0 + TRUCK_AFTER_S + 50.0) == {}                               # never moves again


def test_pool_spots_do_not_overlap():
    core = PalletCore(SPOTS, pool_origin=(-3.0, 0.0))
    deliver(core, "p1", "pallet_PL_W1", (2.0, 31.3), 0.0)
    a = core.tick(11.0 + TRUCK_AFTER_S)["pallet_PL_W1"]
    core.on_robot_state(st("robot_1", "NAVIGATING", "", 0.0, 0.0), 60.0)
    deliver(core, "p2", "pallet_PL_E1", (13.5, 31.3), 100.0)
    b = core.tick(111.0 + TRUCK_AFTER_S)["pallet_PL_E1"]
    assert math.hypot(a[0] - b[0], a[1] - b[1]) >= 1.5


def test_carried_pallet_is_never_taken():
    core = PalletCore(SPOTS)
    core.on_order("p1", api.ORDER_PALLET, "pallet_PL_W1")
    core.on_robot_state(st("robot_1", "CARRYING", "p1", 5.0, 9.0, 0.0), 0.0)
    for t in (10.0, 40.0, 80.0):
        core.on_robot_state(st("robot_1", "CARRYING", "p1", 5.0, 9.0, 0.0), t)
        assert core.tick(t)["pallet_PL_W1"][2] == pytest.approx(LIFT_Z)
