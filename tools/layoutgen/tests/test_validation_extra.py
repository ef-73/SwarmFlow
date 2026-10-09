"""Extra semantic-validation tests for layoutgen (WS-F)."""

from __future__ import annotations

import pathlib
import subprocess
import sys

import yaml

ROOT = pathlib.Path(__file__).resolve().parents[3]
GEN = ROOT / "tools" / "layoutgen" / "generate.py"
LAYOUT = ROOT / "layouts" / "standard" / "layout.yaml"


def run_mut(tmp_path, mutate):
    layout = yaml.safe_load(LAYOUT.read_text(encoding="utf-8"))
    mutate(layout)
    p = tmp_path / "layout.yaml"
    p.write_text(yaml.safe_dump(layout), encoding="utf-8")
    return subprocess.run([sys.executable, str(GEN), str(p), "--out", str(tmp_path / "out")],
                          capture_output=True, text=True)


def test_hold_inside_zone_polygon_rejected(tmp_path):
    def m(layout):
        for h in layout["holds"]:
            if h["name"] == "H_1_1":
                h["pose"] = {"x": 8.0, "y": 3.85}
    r = run_mut(tmp_path, m)
    assert r.returncode == 1 and "H_1_1" in r.stderr


def test_duplicate_vertex_rejected(tmp_path):
    def m(layout):
        layout["holds"].append({"name": "H_1_1", "pose": {"x": 1.0, "y": 1.0}})
    r = run_mut(tmp_path, m)
    assert r.returncode == 1 and "H_1_1" in r.stderr


def test_unknown_zone_lane_rejected(tmp_path):
    def m(layout):
        layout["zones"][0]["lanes"].append({"from": "X_1_1", "to": "X_2_1"})
    r = run_mut(tmp_path, m)
    assert r.returncode == 1 and "X_2_1" in r.stderr
