"""T041: pallet-stand models in the v2 world (F-01)."""

from __future__ import annotations

import math
import pathlib
import subprocess
import sys
import xml.etree.ElementTree as ET

import pytest
import yaml

ROOT = pathlib.Path(__file__).resolve().parents[3]
GEN = ROOT / "tools" / "layoutgen" / "generate.py"
V2 = ROOT / "layouts" / "warehouse_v2"
ROBOT_BODY_W, ROBOT_TOP_Z = 0.50, 0.31          # swarmflow_bot.urdf.xacro body width / top plate


@pytest.fixture(scope="module")
def world(tmp_path_factory):
    d = tmp_path_factory.mktemp("v2w")
    r = subprocess.run([sys.executable, str(GEN), str(V2 / "layout.yaml"), "--out", str(d)], capture_output=True,
                       text=True)
    assert r.returncode == 0, r.stderr
    return ET.parse(d / "world.sdf").getroot()


def _floats(t):
    return [float(v) for v in t.split()]


def test_one_pallet_model_per_pallet_station(world):
    lay = yaml.safe_load((V2 / "layout.yaml").read_text(encoding="utf-8"))
    spots = {s["name"]: s["pose"] for s in lay["stations"] if s["type"] == "pallet"}
    models = {m.get("name"): m for m in world.iter("model") if m.get("name", "").startswith("pallet_")}
    assert set(models) == {f"pallet_{n}" for n in spots}
    for name, m in models.items():
        p = spots[name[len("pallet_"):]]
        x, y, _z, _r, _p, yaw = _floats(m.find("pose").text)
        assert math.hypot(x - p["x"], y - p["y"]) < 1e-6 and abs(yaw - p["yaw"]) < 1e-6
        assert m.find("static").text == "false"           # moved by the payload node while carried


def test_pallet_geometry_lets_the_robot_drive_under(world):
    for m in (m for m in world.iter("model") if m.get("name", "").startswith("pallet_")):
        boxes = []
        for link in m.iter("link"):
            lp = link.find("pose")
            lx, ly, lz = _floats(lp.text)[:3] if lp is not None else (0.0, 0.0, 0.0)
            for v in link.iter("visual"):
                vp = v.find("pose")
                vx, vy, vz = _floats(vp.text)[:3] if vp is not None else (0.0, 0.0, 0.0)
                sx, sy, sz = _floats(v.find("geometry/box/size").text)
                boxes.append((lx + vx, ly + vy, lz + vz, sx, sy, sz))
        legs = [b for b in boxes if b[5] > 0.2]
        decks = [b for b in boxes if b[3] > 1.0]
        assert len(legs) == 4 and len(decks) == 1, boxes
        deck = decks[0]
        assert deck[2] - deck[5] / 2 >= 0.35 - 1e-6                   # underside at least 0.35 m
        assert deck[2] - deck[5] / 2 > ROBOT_TOP_Z + 0.03
        assert abs(deck[3] - 1.20) < 1e-6 and abs(deck[4] - 1.00) < 1e-6
        inner_w = min(abs(b[1]) for b in legs) * 2 - legs[0][4]        # free width between the legs, across
        assert inner_w >= ROBOT_BODY_W + 0.30, inner_w                # at least 15 cm each side of the body
        assert all(abs(abs(b[0]) - abs(legs[0][0])) < 1e-6 for b in legs)   # symmetric
        # no collision (T046): with gravity off, a pallet set down on the floor was pushed out of the ground and
        # tumbled (M16 session p2); the payload node sets its pose and the GPU lidar sees the visuals
        assert not any(True for _ in m.iter("collision"))


def test_standard_world_has_no_pallets(tmp_path):
    r = subprocess.run([sys.executable, str(GEN), str(ROOT / "layouts" / "standard" / "layout.yaml"), "--out",
                        str(tmp_path)], capture_output=True, text=True)
    assert r.returncode == 0
    root = ET.parse(tmp_path / "world.sdf").getroot()
    assert not [m for m in root.iter("model") if m.get("name", "").startswith("pallet_")]
