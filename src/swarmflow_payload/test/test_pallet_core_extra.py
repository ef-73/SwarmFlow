"""Extra PalletCore cases (T041)."""

import math

import pytest

from swarmflow_core import api
from swarmflow_payload.pallet_core import LIFT_Z, PalletCore

SPOTS = {"pallet_PL_W1": (2.5, 5.5, 0.0, math.pi / 2)}


def st(mode, order, x, y, yaw=0.0):
    return {"robot_id": "r1", "mode": mode, "order_id": order, "x": x, "y": y, "yaw": yaw}


def test_pallet_never_moves_again_after_set_down():
    c = PalletCore(SPOTS)
    c.on_order("p1", api.ORDER_PALLET, "pallet_PL_W1")
    c.on_robot_state(st("CARRYING", "p1", 1.0, 1.0), 1.0)
    c.on_robot_state(st("UNLOADING", "p1", 2.0, 2.0), 2.0)
    c.on_robot_state(st("LIFTING", "p1", 2.0, 2.0), 3.0)
    assert c.tick(3.0)["pallet_PL_W1"] == pytest.approx((2.0, 2.0, 0.0, 0.0))
    c.on_robot_state(st("CARRYING", "p1", 9.0, 9.0), 4.0)
    assert c.tick(4.0) == {}


def test_unchanged_pose_is_not_resent():
    c = PalletCore(SPOTS)
    c.on_order("p1", api.ORDER_PALLET, "pallet_PL_W1")
    c.on_robot_state(st("CARRYING", "p1", 1.0, 1.0), 1.0)
    assert LIFT_Z in c.tick(1.0)["pallet_PL_W1"]
    assert c.tick(1.1) == {}
