"""T026: v2 warehouse layout (improvements_v2 L-01 ... L-06, decision 002 schema). Lead test: do not edit."""

from __future__ import annotations

import json
import math
import pathlib
import subprocess
import sys

import pytest
import yaml

ROOT = pathlib.Path(__file__).resolve().parents[3]
GEN = ROOT / "tools" / "layoutgen" / "generate.py"
V2 = ROOT / "layouts" / "warehouse_v2"
SCHEMA = ROOT / "layouts" / "schema"
sys.path.insert(0, str(ROOT / "src" / "swarmflow_core"))

TOL = 0.01
ROW_Y = [(8.0 + 3.2 * i, 9.0 + 3.2 * i) for i in range(5)]           # 5 rows, 1.0 m deep, 2.2 m aisles between


@pytest.fixture(scope="module")
def layout():
    return yaml.safe_load((V2 / "layout.yaml").read_text(encoding="utf-8"))


@pytest.fixture(scope="module")
def gen(tmp_path_factory):
    d = tmp_path_factory.mktemp("v2gen")
    r = subprocess.run([sys.executable, str(GEN), str(V2 / "layout.yaml"), "--out", str(d)],
                       capture_output=True, text=True)
    assert r.returncode == 0, r.stderr
    return d


def stations(layout, kind):
    return [s for s in layout["stations"] if s["type"] == kind]


def test_schema_v2_valid(layout):
    jsonschema = pytest.importorskip("jsonschema")
    jsonschema.validate(layout, json.loads((SCHEMA / "layout.schema.json").read_text(encoding="utf-8")))
    assert layout["schema_version"] == 2 and layout["name"] == "warehouse_v2"
    assert layout["zones"] == [] and layout["holds"] == []          # no traffic control by default


def test_building_and_racks(layout):
    b = layout["bounds"]
    assert (b["x_min"], b["y_min"], b["x_max"], b["y_max"]) == (0.0, 0.0, 20.0, 32.0)
    racks = layout["racks"]
    assert len(racks) == 10
    for side, (x0, x1) in (("W", (0.0, 8.0)), ("E", (12.0, 20.0))):
        rows = sorted((r for r in racks if r["name"].startswith(f"R_{side}")), key=lambda r: r["y_min"])
        assert len(rows) == 5
        for r, (y0, y1) in zip(rows, ROW_Y):
            assert abs(r["x_min"] - x0) < TOL and abs(r["x_max"] - x1) < TOL      # touches the outer wall
            assert abs(r["y_min"] - y0) < TOL and abs(r["y_max"] - y1) < TOL      # 1.0 m deep
    # central aisle 4.0 m between the blocks, storage aisles 2.2 m between rows
    assert abs(min(r["x_min"] for r in racks if r["name"].startswith("R_E"))
               - max(r["x_max"] for r in racks if r["name"].startswith("R_W")) - 4.0) < TOL
    for (_, a1), (b0, _) in zip(ROW_Y, ROW_Y[1:]):
        assert abs(b0 - a1 - 2.2) < TOL


def _aisle_of(y):
    for a, ((_, y1), (y2, _)) in enumerate(zip(ROW_Y, ROW_Y[1:]), start=1):
        if y1 < y < y2:
            return a, y1, y2
    return None


def test_pick_points_both_faces_every_2m(layout):
    picks = stations(layout, "pick")
    assert len(picks) == 64                        # 2 blocks x 4 aisles x 2 faces x 4 slots
    by_face = {}
    for p in picks:
        x, y = p["pose"]["x"], p["pose"]["y"]
        hit = _aisle_of(y)
        assert hit, p["name"]
        a, y1, y2 = hit
        face_d = min(y - y1, y2 - y)
        assert 0.40 - TOL <= face_d <= 0.50 + TOL, p["name"]        # off the aisle centre, towards a rack face
        side = "W" if x < 10.0 else "E"
        assert (0.5 <= x <= 7.5) if side == "W" else (12.5 <= x <= 19.5), p["name"]
        face = "S" if y - y1 < y2 - y else "N"
        assert p["name"].startswith(f"PK_{side}{a}_{face}_"), p["name"]
        by_face.setdefault((side, a, face), []).append(x)
    assert len(by_face) == 16
    for xs in by_face.values():
        xs.sort()
        assert len(xs) == 4 and all(abs((b - a) - 2.0) < TOL for a, b in zip(xs, xs[1:]))


def test_docks_with_queue_lanes(layout):
    docks = {s["name"]: s for s in stations(layout, "dock")}
    assert sorted(docks) == ["DK1", "DK2", "DK3", "DK4"]
    for d in docks.values():
        assert d["pose"]["y"] > 28.0                                 # north end, at the truck doors
    queues = stations(layout, "queue")
    assert len(queues) == 12
    for dk, d in docks.items():
        q = sorted((s for s in queues if s.get("queue_of") == dk), key=lambda s: s["queue_index"])
        assert [s["queue_index"] for s in q] == [1, 2, 3]
        prev = (d["pose"]["x"], d["pose"]["y"])
        for s in q:
            x, y = s["pose"]["x"], s["pose"]["y"]
            assert abs(x - prev[0]) < TOL and abs((prev[1] - y) - 1.6) < 0.05    # straight lane behind the dock
            assert not (8.0 < x < 12.0), "queue lanes stay off the central aisle"
            prev = (x, y)


def test_home_slots_pallets_spawn(layout):
    homes = stations(layout, "home")
    assert len(homes) >= 6 and all(h["pose"]["y"] < 3.0 for h in homes)        # south end
    pallets = stations(layout, "pallet")
    assert len(pallets) >= 4 and all(3.0 < p["pose"]["y"] < 8.0 for p in pallets)
    spawn = {s["robot_id"]: s["vertex"] for s in layout["spawn"]}
    assert len(spawn) >= 5 and set(spawn.values()) <= {h["name"] for h in homes}


def test_no_station_inside_a_rack(layout):
    for s in layout["stations"]:
        x, y = s["pose"]["x"], s["pose"]["y"]
        for r in layout["racks"]:
            assert not (r["x_min"] - 0.3 < x < r["x_max"] + 0.3 and r["y_min"] - 0.3 < y < r["y_max"] + 0.3), \
                (s["name"], r["name"])


def test_keep_right_aisles(layout):
    names = {a["name"]: a for a in layout["aisles"]}
    c = names["central"]
    assert c["axis"] == "y" and abs(c["x_min"] - 8.0) < TOL and abs(c["x_max"] - 12.0) < TOL
    storage = [a for a in layout["aisles"] if a["axis"] == "x" and 8.0 <= a["y_min"] and a["y_max"] <= 21.8]
    assert len(storage) == 8


def test_generated_graph_connected_and_kinds(gen, layout):
    from swarmflow_core.graph import load_layout
    g = load_layout(str(gen))
    kinds = {v.kind for v in g.vertices.values()}
    assert {"pick", "dock", "queue", "home", "pallet"} <= kinds
    names = [s["name"] for s in layout["stations"]]
    for n in names:
        assert g.shortest_path("HM1", n) and g.shortest_path(n, "HM1"), n
    # travel estimates are sane: path length within 1.6x the straight line for far pairs
    for a, b in (("HM1", "DK4"), ("PK_W4_N_01", "DK1"), ("PL_E2", "DK2")):
        va, vb = g.vertices[a], g.vertices[b]
        straight = math.hypot(va.x - vb.x, va.y - vb.y)
        assert straight <= g.path_length(g.shortest_path(a, b)) <= 1.6 * straight + 4.0, (a, b)
    side = yaml.safe_load((gen / "zones.yaml").read_text(encoding="utf-8"))
    assert side["schema_version"] == 2 and len(side["aisles"]) == len(layout["aisles"])
    nav = yaml.safe_load((gen / "nav_graph.yaml").read_text(encoding="utf-8"))
    homes = [v for v in nav["levels"]["L1"]["vertices"] if v[2]["name"].startswith("HM")]
    assert homes and all(v[2].get("is_parking_spot") for v in homes)


def test_committed_generated_files_are_fresh(gen):
    for f in ("world.sdf", "nav_graph.yaml", "zones.yaml", "map.pgm", "map.yaml", "sim2d.json"):
        assert (V2 / "generated" / f).read_bytes() == (gen / f).read_bytes(), f
