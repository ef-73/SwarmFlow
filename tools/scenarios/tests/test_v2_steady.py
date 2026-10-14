"""T040: steady-load v2 scenario for 4 robots (M15 / E-01). Lead test: do not edit."""

from __future__ import annotations

import collections
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "tools" / "scenarios"))

from scenarios import generate_orders, load_scenario  # noqa: E402
from swarmflow_core import api  # noqa: E402
from swarmflow_core.graph import load_layout  # noqa: E402


def test_v2_steady():
    s = load_scenario(ROOT / "scenarios" / "v2_steady.yaml")
    assert s.name == "v2_steady" and s.layout == "warehouse_v2" and s.policy == "assign"
    assert s.robots == ["robot_1", "robot_2", "robot_3", "robot_4"] and s.duration_s == 600 and s.seed == 2
    o = s.orders
    assert o["mode"] == "v2_rate" and sorted(o["docks"]) == ["DK1", "DK2", "DK3", "DK4"]
    assert o["shelf"]["rate_per_min"] == 1.6 and o["shelf"]["picks"] == [1, 2]
    assert o["shelf"]["popularity"] == "zipf" and o["pallet"]["rate_per_min"] == 0.25
    orders = generate_orders(s, load_layout(ROOT / "layouts" / "warehouse_v2" / "generated"))
    types = collections.Counter(x.order_type for x in orders)
    assert types[api.ORDER_SHELF] >= 6 and types[api.ORDER_PALLET] >= 1     # Poisson: seed 2 gives 8 at 1.6/min
    assert {x.dropoff_vertex for x in orders} == {"DK1", "DK2", "DK3", "DK4"}
