"""Layout/sidecar schema contract tests (design §6.1, §7.2; milestone M2 verify). Lead test."""

from __future__ import annotations

import copy
import json
import math
import pathlib

import jsonschema
import pytest
import yaml

ROOT = pathlib.Path(__file__).resolve().parents[2]
SCHEMA = ROOT / "layouts" / "schema"
LAYOUT = ROOT / "layouts" / "standard" / "layout.yaml"
FIX = ROOT / "tests" / "fixtures" / "standard"


def load(path):
    return yaml.safe_load(path.read_text(encoding="utf-8"))


@pytest.fixture(scope="module")
def layout_schema():
    s = json.loads((SCHEMA / "layout.schema.json").read_text(encoding="utf-8"))
    jsonschema.Draft202012Validator.check_schema(s)
    return s


@pytest.fixture(scope="module")
def layout():
    return load(LAYOUT)


def vertex_names(lay):
    return ({s["name"] for s in lay["stations"]} | {i["name"] for i in lay["intersections"]}
            | {h["name"] for h in lay["holds"]})


def semantic_errors(lay):
    """Cross-reference checks JSON Schema cannot express (the generator enforces the same)."""
    errs = []
    names = vertex_names(lay)
    allv = [s["name"] for s in lay["stations"]] + [i["name"] for i in lay["intersections"]] + [h["name"] for h in lay["holds"]]
    if len(allv) != len(set(allv)):
        errs.append("duplicate vertex names")
    lanes = set()
    for ln in lay["lanes"]:
        for k in ("from", "to"):
            if ln[k] not in names:
                errs.append(f"lane references unknown vertex {ln[k]}")
        lanes.add((ln["from"], ln["to"]))
        if ln["bidirectional"]:
            lanes.add((ln["to"], ln["from"]))
    for z in lay["zones"]:
        for ln in z["lanes"]:
            if (ln["from"], ln["to"]) not in lanes:
                errs.append(f"zone {z['name']} references unknown lane {ln['from']}->{ln['to']}")
        for e in z["entries"]:
            if e["entry"] not in names or e["hold"] not in names:
                errs.append(f"zone {z['name']} entry/hold unknown")
    for s in lay["spawn"]:
        if s["vertex"] not in names:
            errs.append(f"spawn {s['robot_id']} at unknown vertex")
    return errs


def point_in_poly(x, y, poly):
    inside = False
    n = len(poly)
    for i in range(n):
        (x1, y1), (x2, y2) = poly[i], poly[(i + 1) % n]
        if (y1 > y) != (y2 > y) and x < x1 + (y - y1) * (x2 - x1) / (y2 - y1):
            inside = not inside
    return inside


def dist_to_poly(x, y, poly):
    best = math.inf
    for i in range(len(poly)):
        (x1, y1), (x2, y2) = poly[i], poly[(i + 1) % len(poly)]
        dx, dy = x2 - x1, y2 - y1
        t = max(0.0, min(1.0, ((x - x1) * dx + (y - y1) * dy) / (dx * dx + dy * dy)))
        best = min(best, math.hypot(x - (x1 + t * dx), y - (y1 + t * dy)))
    return best


def test_standard_layout_valid(layout_schema, layout):
    jsonschema.validate(layout, layout_schema)
    assert semantic_errors(layout) == []


@pytest.mark.parametrize("breaker", [
    "missing_racks", "bad_vertex_name", "zone_capacity_2",
])
def test_broken_layouts_rejected(layout_schema, layout, breaker):
    bad = copy.deepcopy(layout)
    if breaker == "missing_racks":
        del bad["racks"]
    elif breaker == "bad_vertex_name":
        bad["intersections"][0]["name"] = "junction-1"
    elif breaker == "zone_capacity_2":
        bad["zones"][0]["capacity"] = 2
    with pytest.raises(jsonschema.ValidationError):
        jsonschema.validate(bad, layout_schema)


def test_semantic_check_catches_dangling_lane(layout):
    bad = copy.deepcopy(layout)
    bad["lanes"][0]["to"] = "X_9_9"
    assert semantic_errors(bad)


def test_holds_outside_zone_and_back_from_entry(layout):
    poses = {h["name"]: (h["pose"]["x"], h["pose"]["y"]) for h in layout["holds"]}
    poses.update({i["name"]: (i["pose"]["x"], i["pose"]["y"]) for i in layout["intersections"]})
    for z in layout["zones"]:
        for e in z["entries"]:
            hx, hy = poses[e["hold"]]
            ex, ey = poses[e["entry"]]
            assert not point_in_poly(hx, hy, z["polygon"]), f"{e['hold']} inside {z['name']}"
            assert dist_to_poly(hx, hy, z["polygon"]) >= 1.0 - 1e-9
            assert math.hypot(hx - ex, hy - ey) >= 1.0


def test_zone_only_entered_via_hold(layout):
    """Every lane into a zone entry vertex from outside the zone starts at that entry's hold (design §6.3 item 2)."""
    directed = []
    for ln in layout["lanes"]:
        directed.append((ln["from"], ln["to"]))
        if ln["bidirectional"]:
            directed.append((ln["to"], ln["from"]))
    for z in layout["zones"]:
        inside = set(z["vertices"])
        holds = {e["entry"]: e["hold"] for e in z["entries"]}
        for a, b in directed:
            if b in inside and a not in inside:
                assert holds.get(b) == a, f"{a}->{b} enters {z['name']} bypassing its hold"


def test_standard_aisle_widths_match_design_7_2(layout):
    widths = {}
    for ln in layout["lanes"]:
        widths.setdefault(ln["clear_width_m"], 0)
        widths[ln["clear_width_m"]] += 1
    zone_lanes = {(ln["from"], ln["to"]) for z in layout["zones"] for ln in z["lanes"]}
    for ln in layout["lanes"]:
        if (ln["from"], ln["to"]) in zone_lanes:
            assert ln["clear_width_m"] == pytest.approx(1.30)
    # rack gaps = storage aisle clear width 1.30; cross aisles (rack face to wall) 2.20
    racks = sorted(layout["racks"], key=lambda r: r["y_min"])
    gaps = [round(b["y_min"] - a["y_max"], 3) for a, b in zip(racks, racks[1:])]
    assert gaps == [1.3] * (len(racks) - 1)
    assert round(racks[0]["y_min"] - layout["bounds"]["y_min"], 3) == 2.2
    assert round(layout["bounds"]["y_max"] - racks[-1]["y_max"], 3) == 2.2


def test_fixture_sidecar_valid_and_7_2_invariants():
    schema = json.loads((SCHEMA / "zones.schema.json").read_text(encoding="utf-8"))
    jsonschema.Draft202012Validator.check_schema(schema)
    side = load(FIX / "zones.yaml")
    jsonschema.validate(side, schema)
    pay = side["payloads"]
    # design §7.2 table
    assert (pay["unloaded"]["padded_x"], pay["unloaded"]["padded_y"]) == (0.70, 0.60)
    assert (pay["small"]["padded_x"], pay["small"]["padded_y"]) == (0.70, 0.60)
    assert (pay["wide"]["padded_x"], pay["wide"]["padded_y"]) == (0.75, 1.10)
    assert pay["long"]["rotation_diameter"] == pytest.approx(1.48, abs=0.005)
    for e in side["edges"]:
        if e["zone"]:
            # storage aisles: one-way for all loads, every load fits (standard)
            assert not any(e["two_way_ok"].values()), e
            assert all(e["feasible"].values()), e
        elif e["clear_width_m"] >= 2.2:
            for p in ("unloaded", "small", "medium"):
                assert e["two_way_ok"][p], e


def test_fixture_nav_graph_format():
    ng = load(FIX / "nav_graph.yaml")
    assert set(ng) == {"building_name", "doors", "lifts", "levels"}
    lvl = ng["levels"]["L1"]
    names = [v[2]["name"] for v in lvl["vertices"]]
    assert len(names) == len(set(names))
    for i, j, p in lvl["lanes"]:
        assert 0 <= i < len(names) and 0 <= j < len(names) and isinstance(p, dict)
    side = load(FIX / "zones.yaml")
    assert len(lvl["lanes"]) == len(side["edges"])
    assert set(names) == set(side["vertices"])


# ---- geometric clearance (G2 review findings 8, 9b) ------------------------------------------------------------

CIRCUMRADIUS = 0.47   # padded 0.70 x 0.60 footprint (design §7.2)


def seg_dist_to_rect(p, q, r):
    """Min distance between segment p-q and an axis-aligned rectangle (x_min, y_min, x_max, y_max)."""
    import itertools
    x0, y0, x1, y1 = r
    def inside(pt):
        return x0 <= pt[0] <= x1 and y0 <= pt[1] <= y1
    if inside(p) or inside(q):
        return 0.0
    corners = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
    edges = list(zip(corners, corners[1:] + corners[:1]))
    def pt_seg(pt, a, b):
        dx, dy = b[0] - a[0], b[1] - a[1]
        L = dx * dx + dy * dy
        t = 0.0 if L == 0 else max(0.0, min(1.0, ((pt[0] - a[0]) * dx + (pt[1] - a[1]) * dy) / L))
        return math.hypot(pt[0] - a[0] - t * dx, pt[1] - a[1] - t * dy)
    def cross(a, b, c, d):
        def o(p1, p2, p3):
            return (p2[0] - p1[0]) * (p3[1] - p1[1]) - (p2[1] - p1[1]) * (p3[0] - p1[0])
        return o(a, b, c) * o(a, b, d) < 0 and o(c, d, a) * o(c, d, b) < 0
    if any(cross(p, q, a, b) for a, b in edges):
        return 0.0
    return min([pt_seg(c, p, q) for c in corners] + [pt_seg(p, a, b) for a, b in edges] + [pt_seg(q, a, b) for a, b in edges])


def _poses(lay):
    d = {s["name"]: (s["pose"]["x"], s["pose"]["y"]) for s in lay["stations"]}
    d.update({i["name"]: (i["pose"]["x"], i["pose"]["y"]) for i in lay["intersections"]})
    d.update({h["name"]: (h["pose"]["x"], h["pose"]["y"]) for h in lay["holds"]})
    return d


def test_lanes_clear_racks_and_walls_by_circumradius(layout):
    poses = _poses(layout)
    b = layout["bounds"]
    for ln in layout["lanes"]:
        p, q = poses[ln["from"]], poses[ln["to"]]
        for r in layout["racks"]:
            d = seg_dist_to_rect(p, q, (r["x_min"], r["y_min"], r["x_max"], r["y_max"]))
            assert d >= CIRCUMRADIUS, f"{ln['from']}->{ln['to']} {d:.3f} m from {r['name']}"
        for x, y in (p, q):
            assert min(x - b["x_min"], b["x_max"] - x, y - b["y_min"], b["y_max"] - y) >= CIRCUMRADIUS


def test_holds_off_exit_paths(layout):
    """A robot waiting at a hold is >= 2 circumradii from every lane leaving a zone (review finding: exit path)."""
    poses = _poses(layout)
    zone_v = {v for z in layout["zones"] for v in z["vertices"]}
    exits = [(ln["from"], ln["to"]) for ln in layout["lanes"]
             if ln["from"] in zone_v and ln["to"] not in zone_v and not ln["bidirectional"]]
    assert exits
    for h in layout["holds"]:
        for a, b in exits:
            p, q = poses[a], poses[b]
            d = seg_dist_to_rect(p, q, (poses[h["name"]][0], poses[h["name"]][1]) * 2)
            assert d >= 2 * CIRCUMRADIUS, f"{h['name']} {d:.3f} m from exit lane {a}->{b}"
