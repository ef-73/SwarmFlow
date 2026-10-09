"""Layout generator acceptance tests (design §6.1, §7.2, §14.2 WS-F). Lead test: do not edit.

CLI under test: ``python3 tools/layoutgen/generate.py <layout.yaml> --out <dir>``.
"""

from __future__ import annotations

import json
import math
import pathlib
import subprocess
import sys
import xml.etree.ElementTree as ET

import pytest
import yaml

ROOT = pathlib.Path(__file__).resolve().parents[3]
GEN = ROOT / "tools" / "layoutgen" / "generate.py"
LAYOUT = ROOT / "layouts" / "standard" / "layout.yaml"
FIX = ROOT / "tests" / "fixtures" / "standard"
FILES = ["world.sdf", "nav_graph.yaml", "zones.yaml", "map.pgm", "map.yaml", "sim2d.json"]


def run(layout, out):
    return subprocess.run([sys.executable, str(GEN), str(layout), "--out", str(out)], capture_output=True, text=True)


@pytest.fixture(scope="module")
def out(tmp_path_factory):
    d = tmp_path_factory.mktemp("gen")
    r = run(LAYOUT, d)
    assert r.returncode == 0, r.stderr
    return d


@pytest.fixture(scope="module")
def layout():
    return yaml.safe_load(LAYOUT.read_text(encoding="utf-8"))


def test_all_files_written(out):
    for f in FILES:
        assert (out / f).is_file(), f


@pytest.mark.parametrize("name", ["nav_graph.yaml", "zones.yaml"])
def test_graph_files_match_frozen_fixtures(out, name):
    got = yaml.safe_load((out / name).read_text(encoding="utf-8"))
    want = yaml.safe_load((FIX / name).read_text(encoding="utf-8"))
    assert got == want


def test_deterministic(tmp_path, out):
    r = run(LAYOUT, tmp_path)
    assert r.returncode == 0
    for f in FILES:
        assert (tmp_path / f).read_bytes() == (out / f).read_bytes(), f


# ---- Nav2 map -------------------------------------------------------------------------------------------------

def read_pgm(path):
    data = path.read_bytes()
    parts, i = [], 0
    while len(parts) < 4:                       # magic, width, height, maxval (comments allowed)
        while data[i:i + 1].isspace():
            i += 1
        if data[i:i + 1] == b"#":
            while data[i:i + 1] != b"\n":
                i += 1
            continue
        j = i
        while not data[j:j + 1].isspace():
            j += 1
        parts.append(data[i:j])
        i = j
    assert parts[0] == b"P5"
    w, h, maxval = int(parts[1]), int(parts[2]), int(parts[3])
    assert maxval == 255
    pix = data[i + 1:]
    assert len(pix) == w * h
    return w, h, pix


@pytest.fixture(scope="module")
def nav_map(out):
    meta = yaml.safe_load((out / "map.yaml").read_text(encoding="utf-8"))
    w, h, pix = read_pgm(out / meta["image"])
    return meta, w, h, pix


def px(meta, w, h, pix, x, y):
    res, (ox, oy, _) = meta["resolution"], meta["origin"]
    c, r = int(math.floor((x - ox) / res)), int(math.floor((y - oy) / res))
    assert 0 <= c < w and 0 <= r < h, (x, y)
    return pix[(h - 1 - r) * w + c]


def test_map_yaml(nav_map, layout):
    meta, w, h, _ = nav_map
    assert meta["image"] == "map.pgm" and meta["resolution"] == pytest.approx(layout["resolution_m"])
    assert meta["origin"] == pytest.approx([-0.5, -0.5, 0.0])
    assert meta["mode"] == "trinary" and meta["negate"] == 0
    assert meta["occupied_thresh"] == pytest.approx(0.65) and meta["free_thresh"] == pytest.approx(0.25)
    b = layout["bounds"]
    assert w == round((b["x_max"] - b["x_min"] + 1.0) / meta["resolution"])
    assert h == round((b["y_max"] - b["y_min"] + 1.0) / meta["resolution"])


def test_map_pixels(nav_map, layout):
    meta, w, h, pix = nav_map
    P = lambda x, y: px(meta, w, h, pix, x, y)  # noqa: E731
    for r in layout["racks"]:
        occ = tot = 0
        x = r["x_min"] + 0.025
        while x < r["x_max"]:
            y = r["y_min"] + 0.025
            while y < r["y_max"]:
                tot += 1
                occ += P(x, y) == 0
                y += 0.05
            x += 0.05
        assert occ / tot >= 0.95, r["name"]
    for s in layout["stations"]:
        assert P(s["pose"]["x"], s["pose"]["y"]) == 254, s["name"]
    for v in layout["intersections"] + layout["holds"]:
        assert P(v["pose"]["x"], v["pose"]["y"]) == 254, v["name"]
    assert P(11.0, 3.85) == 254 and P(11.0, 11.2) == 254          # aisle centres free
    b = layout["bounds"]
    assert P(b["x_min"] - 0.1, 5.0) == 0 and P(b["x_max"] + 0.1, 5.0) == 0     # walls
    assert P(5.0, b["y_min"] - 0.1) == 0 and P(5.0, b["y_max"] + 0.1) == 0
    assert set(pix) <= {0, 254}


# ---- Gazebo world -------------------------------------------------------------------------------------------

@pytest.fixture(scope="module")
def world(out):
    return ET.parse(out / "world.sdf").getroot()


def models(world):
    return {m.get("name"): m for m in world.iter("model")}


def test_world_plugins_and_physics(world, layout):
    w = world.find("world")
    assert w is not None and w.get("name") == layout["name"]
    files = {p.get("filename") for p in w.findall("plugin")}
    for f in ("gz-sim-physics-system", "gz-sim-user-commands-system", "gz-sim-scene-broadcaster-system",
              "gz-sim-sensors-system"):
        assert f in files, f
    sensors = [p for p in w.findall("plugin") if p.get("filename") == "gz-sim-sensors-system"][0]
    assert sensors.findtext("render_engine") == "ogre2"
    assert w.find("physics") is not None and float(w.find("physics").findtext("real_time_factor")) == 1.0
    assert "include" not in {c.tag for c in w}      # no Fuel downloads: everything inline


def box_size(geom_parent):
    return [float(v) for v in geom_parent.find("geometry/box/size").text.split()]


def test_racks_are_solid_static_boxes(world, layout):
    ms = models(world)
    for r in layout["racks"]:
        m = ms[f"rack_{r['name']}"]
        assert m.findtext("static").strip() == "true"
        link = m.find("link")
        col, vis = link.find("collision"), link.find("visual")
        assert col is not None and vis is not None
        sx, sy, sz = box_size(col)
        assert (sx, sy) == pytest.approx((r["x_max"] - r["x_min"], r["y_max"] - r["y_min"]))
        assert sz == pytest.approx(r.get("height_m", 2.0))
        x, y, z = [float(v) for v in m.findtext("pose").split()[:3]]
        assert (x, y, z) == pytest.approx(((r["x_min"] + r["x_max"]) / 2, (r["y_min"] + r["y_max"]) / 2, sz / 2))


def test_walls_ground_light(world):
    ms = models(world)
    assert "ground_plane" in ms
    walls = [n for n in ms if n.startswith("wall_")]
    assert len(walls) == 4
    for n in walls:
        assert ms[n].find("link/collision") is not None
    assert world.find("world/light") is not None


def test_stations_visual_only(world, layout):
    ms = models(world)
    for s in layout["stations"]:
        m = ms[f"station_{s['name']}"]
        assert m.find("link/visual") is not None and m.find("link/collision") is None


def test_package_pool(world, layout):
    ms = models(world)
    pk = sorted(n for n in ms if n.startswith("pkg_"))
    assert pk == [f"pkg_{i:02d}" for i in range(12)]
    b = layout["bounds"]
    for n in pk:
        m = ms[n]
        link = m.find("link")
        assert link.find("collision") is None                       # no contact physics (design §7.4)
        assert link.findtext("gravity").strip() == "false"
        assert m.findtext("static", "false").strip() == "false"
        sx, sy, sz = box_size(link.find("visual"))
        assert (sx, sy) == pytest.approx((0.40, 0.40))
        x, y = [float(v) for v in m.findtext("pose").split()[:2]]
        assert not (b["x_min"] <= x <= b["x_max"] and b["y_min"] <= y <= b["y_max"])  # parked outside


# ---- 2D sim layout ----------------------------------------------------------------------------------------

def test_sim2d_json(out):
    d = json.loads((out / "sim2d.json").read_text(encoding="utf-8"))
    for k in ("name", "bounds", "racks", "vertices", "edges", "zones", "spawn"):
        assert k in d
    side = yaml.safe_load((FIX / "zones.yaml").read_text(encoding="utf-8"))
    assert set(d["vertices"]) == set(side["vertices"])
    assert len(d["edges"]) == len(side["edges"])
    assert [z["name"] for z in d["zones"]] == [z["name"] for z in side["zones"]]
    assert d["spawn"][0] == {"robot_id": "robot_1", "vertex": "P1", "yaw": 0.0}


# ---- validation -------------------------------------------------------------------------------------------

def test_rejects_schema_invalid_layout(tmp_path, layout):
    bad = dict(layout)
    bad.pop("racks")
    p = tmp_path / "layout.yaml"
    p.write_text(yaml.safe_dump(bad), encoding="utf-8")
    r = run(p, tmp_path / "out")
    assert r.returncode != 0 and "racks" in (r.stderr + r.stdout)


def test_rejects_dangling_lane(tmp_path, layout):
    bad = yaml.safe_load(yaml.safe_dump(layout))
    bad["lanes"][0]["to"] = "X_9_9"
    p = tmp_path / "layout.yaml"
    p.write_text(yaml.safe_dump(bad), encoding="utf-8")
    r = run(p, tmp_path / "out")
    assert r.returncode != 0 and "X_9_9" in (r.stderr + r.stdout)


def test_rejects_zone_entry_without_hold_path(tmp_path, layout):
    """A lane into a zone entry from outside that does not start at its hold breaks design §6.3 item 2."""
    bad = yaml.safe_load(yaml.safe_dump(layout))
    bad["lanes"].append({"from": "X_1_0", "to": "X_1_1", "clear_width_m": 3.4, "bidirectional": False})
    p = tmp_path / "layout.yaml"
    p.write_text(yaml.safe_dump(bad), encoding="utf-8")
    r = run(p, tmp_path / "out")
    assert r.returncode != 0 and "X_1_1" in (r.stderr + r.stdout)
