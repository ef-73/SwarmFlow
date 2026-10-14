"""T047 extra checks: pool origin and pool ordering."""

import math

import pytest

from swarmflow_core import api
from swarmflow_payload.pallet_core import TRUCK_AFTER_S, PalletCore

SPOTS = {"a": (1.0, 1.0, 0.0, 0.0), "b": (2.0, 2.0, 0.0, 0.0)}


def st(mode, order, x=5.0, y=6.0):
    return {"robot_id": "r", "mode": mode, "order_id": order, "x": x, "y": y, "yaw": 0.0}


def test_custom_pool_origin_and_order():
    core = PalletCore(SPOTS, pool_origin=(-9.0, 4.0))
    for i, p in enumerate(("a", "b")):
        t = 100.0 * i
        core.on_order(f"o{i}", api.ORDER_PALLET, p)
        core.on_robot_state(st("CARRYING", f"o{i}"), t)
        core.on_robot_state(st("UNLOADING", f"o{i}"), t + 1)
        core.on_robot_state(st("IDLE", ""), t + 2)
        core.tick(t + 2)
        out = core.tick(t + 2 + TRUCK_AFTER_S)
        assert out[p] == pytest.approx((-9.0, 4.0 + 1.5 * i, 0.0, 0.0))
