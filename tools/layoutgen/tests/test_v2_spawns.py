"""T052: warehouse_v2 spawns 8 robots, one per home slot (light-mode scaling check). Lead test: do not edit."""

import pathlib

import yaml

ROOT = pathlib.Path(__file__).resolve().parents[3]


def test_eight_spawns_one_per_home_slot():
    lay = yaml.safe_load((ROOT / "layouts" / "warehouse_v2" / "layout.yaml").read_text(encoding="utf-8"))
    spawn = {s["robot_id"]: s["vertex"] for s in lay["spawn"]}
    assert spawn == {f"robot_{k}": f"HM{k}" for k in range(1, 9)}
