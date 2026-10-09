"""Graph loader tests (design §6.1). Lead test."""
import math
import pathlib

import pytest

from swarmflow_core import api
from swarmflow_core.graph import WarehouseGraph, load_layout, point_in_polygon

FIX = pathlib.Path(__file__).resolve().parents[3] / "tests" / "fixtures" / "standard"
G = load_layout(FIX)


def test_loads_all_vertices_edges_zones():
    assert len(G.vertices) == 34 and len(G.edges) == 74 and len(G.zones) == 3
    assert G.vertices["L1"].kind == "loading" and G.vertices["H_1_1"].kind == "hold"
    assert G.vertices["L1"].yaw == pytest.approx(math.pi, abs=1e-3) and G.vertices["X_0_0"].yaw is None
    z = G.zones["Z_aisle_2"]
    assert z.capacity == 1 and ("X_2_1", "X_2_3") in z.edges and z.entries[0] == api.ZoneEntry("X_2_1", "H_2_1")
    assert G.zone_of_edge("X_2_3", "X_2_1") == "Z_aisle_2" and G.zone_of_edge("L1", "X_1_0") is None
    assert G.edges[("X_1_1", "X_1_3")].length_m == pytest.approx(10.0)


def test_successors_sorted_and_one_way_lanes():
    assert list(G.successors("X_1_0")) == sorted(G.successors("X_1_0"))
    assert "X_1_1" not in G.successors("X_1_0")      # zone only via the hold
    assert "X_1_0" in G.successors("X_1_1")          # exit lane
    assert "H_1_1" not in G.successors("X_1_1")


@pytest.mark.parametrize("a,b,expected", [
    # equal-length alternatives (aisle 1 vs aisle 3) → tie broken by vertex-name order
    ("L1", "D3", ["L1", "X_1_0", "H_1_1", "X_1_1", "X_1_3", "X_1_4", "X_2_4", "X_3_4", "D3"]),
    ("D1", "L2", ["D1", "X_1_4", "H_1_3", "X_1_3", "X_1_1", "X_1_0", "X_2_0", "L2"]),
    ("L2", "D2", ["L2", "X_2_0", "H_2_1", "X_2_1", "X_2_3", "X_2_4", "D2"]),
    ("P1", "L1", ["P1", "X_0_0", "X_1_0", "L1"]),
])
def test_shortest_paths(a, b, expected):
    assert list(G.shortest_path(a, b)) == expected
    assert G.path_length(expected) == pytest.approx(sum(G.edges[(u, v)].length_m for u, v in zip(expected, expected[1:])))


def test_every_zone_route_passes_hold_first():
    for a in G.vertices:
        for b in ("L1", "L2", "L3", "D1", "D2", "D3", "P1", "P4"):
            if a == b:
                continue
            r = list(G.shortest_path(a, b))
            for i in range(1, len(r) - 1):
                z = G.zone_of_edge(r[i], r[i + 1])
                if z and G.zone_of_edge(r[i - 1], r[i]) != z and r[i - 1] not in G.zones[z].vertices:
                    assert any(e.entry == r[i] and e.hold == r[i - 1] for e in G.zones[z].entries), (a, b, r)


def test_errors_and_trivial():
    with pytest.raises(KeyError):
        G.shortest_path("L1", "nope")
    assert list(G.shortest_path("L1", "L1")) == ["L1"]
    lone = WarehouseGraph({"A": api.Vertex("A", 0, 0, "intersection"), "B": api.Vertex("B", 1, 0, "intersection")},
                          {}, {})
    with pytest.raises(ValueError):
        lone.shortest_path("A", "B")


def test_infeasible_payload_edges_skipped():
    v = {n: api.Vertex(n, x, 0, "intersection") for n, x in (("A", 0), ("B", 1), ("C", 2))}
    v["D"] = api.Vertex("D", 1, 1, "intersection")
    e = {("A", "C"): api.Edge("A", "C", 2.0, 1.0, False, feasible={"small": True, "wide": False}),
         ("A", "D"): api.Edge("A", "D", 1.5, 3.0, False, feasible={"small": True, "wide": True}),
         ("D", "C"): api.Edge("D", "C", 1.5, 3.0, False, feasible={"small": True, "wide": True})}
    g = WarehouseGraph(v, e, {})
    assert list(g.shortest_path("A", "C", "small")) == ["A", "C"]
    assert list(g.shortest_path("A", "C", "wide")) == ["A", "D", "C"]


def test_deterministic_tie_break():
    v = {n: api.Vertex(n, 0, 0, "intersection") for n in "SABT"}
    e = {k: api.Edge(k[0], k[1], 1.0, 2.0, False) for k in (("S", "B"), ("S", "A"), ("A", "T"), ("B", "T"))}
    assert list(WarehouseGraph(v, e, {}).shortest_path("S", "T")) == ["S", "A", "T"]


def test_zone_at_and_point_in_polygon():
    assert G.zone_at(11.0, 3.85) == "Z_aisle_1"
    assert G.zone_at(6.0, 3.85) == "Z_aisle_1"          # boundary counts as inside
    assert G.zone_at(5.9, 3.85) is None and G.zone_at(11.0, 5.0) is None
    sq = ((0, 0), (1, 0), (1, 1), (0, 1))
    assert point_in_polygon(0.5, 0.5, sq) and point_in_polygon(1.0, 0.5, sq) and not point_in_polygon(1.01, 0.5, sq)


def test_helpers():
    assert G.nearest_vertex(1.4, 1.0) == "P1"
    assert G.nearest_vertex(4.0, 3.0, kinds=("park",)) == "P1"
    assert G.vertices_of_kind("park") == ("P1", "P2", "P3", "P4")
    assert G.distance("P1", "L1") == pytest.approx(G.path_length(["P1", "X_0_0", "X_1_0", "L1"]))
    assert G.zone_entry("Z_aisle_1", "X_1_3") == api.ZoneEntry("X_1_3", "H_1_3")
    assert G.payloads["wide"]["padded_y"] == pytest.approx(1.10)
