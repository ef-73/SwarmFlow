"""T043: manual orders from the command line (improvements_v2 O-07). Lead test: do not edit.

    python3 -m scenarios.order_cli shelf --picks PK_W1_S_01 PK_E2_N_03 --dock DK2 [--priority 2] [--id my_1]
    python3 -m scenarios.order_cli pallet --spot PL_W1 --dock DK1
    python3 -m scenarios.order_cli transport --pickup L1 --dropoff D3          (v1 layouts)
"""

from __future__ import annotations

import pathlib
import sys

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "tools" / "scenarios"))

from scenarios.order_cli import build_order, parse_args  # noqa: E402
from swarmflow_core import api  # noqa: E402
from swarmflow_core.graph import load_layout  # noqa: E402

V2 = load_layout(ROOT / "layouts" / "warehouse_v2" / "generated")
STD = load_layout(ROOT / "layouts" / "standard" / "generated")


def test_shelf_order():
    o = build_order(parse_args(["shelf", "--picks", "PK_W1_S_01", "PK_E2_N_03", "--dock", "DK2", "--priority", "2",
                                "--id", "m1"]), V2, now=12.5)
    assert o == api.OrderSpec("m1", 12.5, None, "", "DK2", "small", 2, order_type=api.ORDER_SHELF,
                              pick_vertices=("PK_W1_S_01", "PK_E2_N_03"))


def test_pallet_order_and_generated_id():
    o = build_order(parse_args(["pallet", "--spot", "PL_W1", "--dock", "DK1"]), V2, now=3.0)
    assert o.order_type == api.ORDER_PALLET and o.pickup_vertex == "PL_W1" and o.dropoff_vertex == "DK1"
    assert o.payload_type == "pallet" and o.pallet_id == "pallet_PL_W1"
    assert o.order_id.startswith("manual_") and o.release_t == 3.0


def test_transport_order_on_v1():
    o = build_order(parse_args(["transport", "--pickup", "L1", "--dropoff", "D3"]), STD, now=0.0)
    assert o.order_type == api.ORDER_TRANSPORT and (o.pickup_vertex, o.dropoff_vertex) == ("L1", "D3")


@pytest.mark.parametrize("argv,msg", [
    (["shelf", "--picks", "PK_W1_S_01", "--dock", "DK9"], "DK9"),
    (["shelf", "--picks", "DK1", "--dock", "DK2"], "pick"),
    (["shelf", "--picks", "PK_W1_S_01", "PK_W1_S_01", "--dock", "DK2"], "distinct"),
    (["pallet", "--spot", "PK_W1_S_01", "--dock", "DK1"], "pallet"),
])
def test_invalid_orders_rejected(argv, msg):
    with pytest.raises(ValueError, match=msg):
        build_order(parse_args(argv), V2, now=0.0)
