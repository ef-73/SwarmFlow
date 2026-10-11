"""T027: v2 order generator (shelf and pallet orders, improvements_v2 P-02, F-06). Lead test: do not edit.

Scenario format (orders.mode = v2_rate):
    orders:
      mode: v2_rate
      docks: [DK1, DK2, DK3, DK4]
      shelf: {rate_per_min: 4.0, picks: [1, 3], slots: all, popularity: uniform | zipf, zipf_s: 1.0}
      pallet: {rate_per_min: 0.5}          # optional; each pallet spot is used at most once per scenario
"""

from __future__ import annotations

import collections
import dataclasses
import pathlib
import sys

import pytest
import yaml

ROOT = pathlib.Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "tools" / "scenarios"))

from scenarios import generate_orders, load_scenario  # noqa: E402
from swarmflow_core import api  # noqa: E402
from swarmflow_core.graph import load_layout  # noqa: E402

GRAPH = load_layout(ROOT / "layouts" / "warehouse_v2" / "generated")
DEMO = ROOT / "scenarios" / "v2_demo.yaml"
PICKS = set(GRAPH.vertices_of_kind("pick"))
DOCKS = set(GRAPH.vertices_of_kind("dock"))
PALLETS = set(GRAPH.vertices_of_kind("pallet"))


def scenario(tmp_path, orders, duration=3600, seed=5, name="t"):
    raw = {"name": name, "seed": seed, "duration_s": duration, "layout": "warehouse_v2", "policy": "assign",
           "robots": ["robot_1", "robot_2"], "orders": orders}
    p = tmp_path / f"{name}.yaml"
    p.write_text(yaml.safe_dump(raw), encoding="utf-8")
    return load_scenario(p)


BASE = {"mode": "v2_rate", "docks": ["DK1", "DK2", "DK3", "DK4"],
        "shelf": {"rate_per_min": 4.0, "picks": [1, 3], "slots": "all", "popularity": "uniform"}}


def test_v2_demo_file():
    s = load_scenario(DEMO)
    assert s.layout == "warehouse_v2" and s.policy == "assign" and len(s.robots) >= 4
    orders = generate_orders(s, GRAPH)
    assert orders and {o.order_type for o in orders} == {api.ORDER_SHELF, api.ORDER_PALLET}


def test_same_seed_identical_other_seed_different(tmp_path):
    a = generate_orders(scenario(tmp_path, BASE, seed=5, name="a"), GRAPH)
    b = generate_orders(scenario(tmp_path, BASE, seed=5, name="b"), GRAPH)
    c = generate_orders(scenario(tmp_path, BASE, seed=6, name="c"), GRAPH)
    key = lambda os_: [dataclasses.astuple(o) for o in os_]  # noqa: E731
    assert key(a) == key(b) and key(a) != key(c)


def test_shelf_orders_are_valid(tmp_path):
    orders = generate_orders(scenario(tmp_path, BASE), GRAPH)
    assert 0.7 * 240 <= len(orders) <= 1.3 * 240                  # 4/min for 60 min
    ids = [o.order_id for o in orders]
    assert len(set(ids)) == len(ids)
    assert [o.release_t for o in orders] == sorted(o.release_t for o in orders)
    sizes = collections.Counter()
    for o in orders:
        assert o.order_type == api.ORDER_SHELF and o.pickup_vertex == "" and o.pallet_id == ""
        assert 1 <= len(o.pick_vertices) <= 3 and len(set(o.pick_vertices)) == len(o.pick_vertices)
        assert set(o.pick_vertices) <= PICKS and o.dropoff_vertex in DOCKS
        assert o.payload_type == "small" and o.deadline_t is None
        sizes[len(o.pick_vertices)] += 1
    assert set(sizes) == {1, 2, 3}
    assert {o.dropoff_vertex for o in orders} == DOCKS


def test_zipf_popularity_concentrates_picks(tmp_path):
    zipf = dict(BASE, shelf=dict(BASE["shelf"], popularity="zipf", zipf_s=1.2))
    counts = collections.Counter(v for o in generate_orders(scenario(tmp_path, zipf), GRAPH) for v in o.pick_vertices)
    uniform = collections.Counter(v for o in generate_orders(scenario(tmp_path, BASE, name="u"), GRAPH)
                                  for v in o.pick_vertices)
    top = lambda c: sum(n for _, n in c.most_common(5)) / sum(c.values())  # noqa: E731
    assert top(counts) > 2 * top(uniform)


def test_pallet_orders_each_spot_once(tmp_path):
    spec = dict(BASE, pallet={"rate_per_min": 2.0})
    orders = generate_orders(scenario(tmp_path, spec), GRAPH)
    pallets = [o for o in orders if o.order_type == api.ORDER_PALLET]
    assert len(pallets) == len(PALLETS)                            # rate would give 120: capped by the spots
    assert {o.pickup_vertex for o in pallets} == PALLETS
    for o in pallets:
        assert o.dropoff_vertex in DOCKS and o.payload_type == "pallet" and o.pick_vertices == ()
        assert o.pallet_id == f"pallet_{o.pickup_vertex}"


def test_slot_subset(tmp_path):
    sub = sorted(PICKS)[:6]
    spec = dict(BASE, shelf=dict(BASE["shelf"], slots=sub))
    orders = generate_orders(scenario(tmp_path, spec), GRAPH)
    assert {v for o in orders for v in o.pick_vertices} <= set(sub)


@pytest.mark.parametrize("patch,msg", [
    ({"docks": ["DK9"]}, "DK9"),
    ({"docks": ["PK_W1_N_01"]}, "dock"),
    ({"shelf": {"rate_per_min": 4.0, "picks": [0, 2], "slots": "all", "popularity": "uniform"}}, "picks"),
    ({"shelf": {"rate_per_min": 4.0, "picks": [1, 2], "slots": "all", "popularity": "gauss"}}, "popularity"),
    ({"shelf": {"rate_per_min": 4.0, "picks": [1, 2], "slots": ["X_0_0"], "popularity": "uniform"}}, "pick"),
])
def test_invalid_v2_specs_rejected(tmp_path, patch, msg):
    with pytest.raises(ValueError, match=msg):
        s = scenario(tmp_path, dict(BASE, **patch))
        generate_orders(s, GRAPH)
