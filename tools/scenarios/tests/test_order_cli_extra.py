import pathlib
import sys

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "tools" / "scenarios"))

from scenarios.order_cli import build_order, parse_args  # noqa: E402
from swarmflow_core.graph import load_layout  # noqa: E402

V2 = load_layout(ROOT / "layouts" / "warehouse_v2" / "generated")


def test_defaults():
    a = parse_args(["pallet", "--spot", "PL_W1", "--dock", "DK1"])
    assert a.priority == 0 and a.layout_dir.endswith("/generated")


def test_unknown_vertex():
    with pytest.raises(ValueError, match="NOPE"):
        build_order(parse_args(["shelf", "--picks", "NOPE", "--dock", "DK1"]), V2, now=0.0)
