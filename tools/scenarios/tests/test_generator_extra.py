"""Extra generator tests (WS-F): validation errors and explicit/repeat semantics."""

from __future__ import annotations

import pathlib
import sys

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "tools" / "scenarios"))

from scenarios import generate_orders, load_scenario  # noqa: E402
from swarmflow_core.graph import load_layout  # noqa: E402

GRAPH = load_layout(ROOT / "layouts" / "standard" / "generated")
HEAD = "name: t\nseed: 1\nduration_s: 100\nlayout: standard\npolicy: fcfs\nrobots: [robot_1]\n"


def _load(tmp_path, body):
    p = tmp_path / "s.yaml"
    p.write_text(HEAD + body, encoding="utf-8")
    return load_scenario(p)


def test_explicit_repeat_and_ordering(tmp_path):
    s = _load(tmp_path, "orders: {mode: explicit, repeat_every_s: 40, list: [{t: 30, pickup: L1, dropoff: D1}, "
                        "{t: 5, pickup: L2, dropoff: D2}]}\n")
    ts = [o.release_t for o in generate_orders(s, GRAPH)]
    assert ts == [5.0, 30.0, 45.0, 70.0, 85.0]


def test_explicit_without_repeat_drops_late_orders(tmp_path):
    s = _load(tmp_path, "orders: {mode: explicit, list: [{t: 1, pickup: L1, dropoff: D1}, "
                        "{t: 100, pickup: L1, dropoff: D1}]}\n")
    assert len(generate_orders(s, GRAPH)) == 1


@pytest.mark.parametrize("body", [
    "orders: {mode: bogus}\n",
    "orders: {mode: rate, rate_per_min: 0, pickups: [L1], dropoffs: [D1]}\n",
    "orders: {mode: rate, rate_per_min: 2, pickups: [], dropoffs: [D1]}\n",
    "orders: {mode: explicit, list: []}\n",
    "orders: 5\n",
])
def test_bad_orders_rejected(tmp_path, body):
    with pytest.raises(ValueError):
        _load(tmp_path, body)


def test_unknown_vertex_and_missing_field(tmp_path):
    s = _load(tmp_path, "orders: {mode: rate, rate_per_min: 2, pickups: [L9], dropoffs: [D1]}\n")
    with pytest.raises(ValueError):
        generate_orders(s, GRAPH)
    p = tmp_path / "m.yaml"
    p.write_text("name: t\n", encoding="utf-8")
    with pytest.raises(ValueError):
        load_scenario(p)


def test_bad_policy_rejected(tmp_path):
    p = tmp_path / "p.yaml"
    p.write_text(HEAD.replace("fcfs", "magic") + "orders: {mode: rate, rate_per_min: 2, pickups: [L1], "
                 "dropoffs: [D1]}\n", encoding="utf-8")
    with pytest.raises(ValueError):
        load_scenario(p)
