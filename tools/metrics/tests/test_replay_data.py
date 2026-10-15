"""replay_data.py: only the scenario window, times relative to its start, delivered events kept, layout copied."""

import importlib.util
import json
import pathlib

import yaml

HERE = pathlib.Path(__file__).resolve().parents[1]
REPO = HERE.parents[1]


def _mod():
    spec = importlib.util.spec_from_file_location("replay_data", HERE / "replay_data.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_window_and_events(tmp_path):
    (tmp_path / "config.yaml").write_text(yaml.safe_dump(
        {"scenario": {"duration_s": 10.0}, "start_time_sim_s": 100.0, "start_delay_s": 5.0}), encoding="utf-8")
    states = [{"t": t, "robot_id": "robot_1", "x": 1.0 + t / 100, "y": 2.0, "yaw": 0.0, "mode": m, "order_id": "o1"}
              for t, m in ((104.0, "IDLE"), (106.0, "NAVIGATING"), (110.0, "STUCK"), (116.0, "IDLE"))]
    (tmp_path / "robot_states.jsonl").write_text("".join(json.dumps(s) + "\n" for s in states), encoding="utf-8")
    orders = [{"t": 107.0, "order_id": "o1", "state": "DELIVERED", "robot_id": "robot_1"},
              {"t": 107.0, "order_id": "o1", "state": "ASSIGNED", "robot_id": "robot_1"},
              {"t": 130.0, "order_id": "o2", "state": "DELIVERED", "robot_id": "robot_1"}]
    (tmp_path / "orders.jsonl").write_text("".join(json.dumps(o) + "\n" for o in orders), encoding="utf-8")
    d = _mod().build(str(tmp_path), str(REPO / "layouts" / "warehouse_v2" / "layout.yaml"))
    samples = d["robots"]["robot_1"]
    assert [s[0] for s in samples] == [1.0, 5.0]                      # 104 and 116 are outside [105, 115]
    assert d["modes"][samples[1][4]] == "STUCK"
    assert d["events"] == [[2.0, "DELIVERED", "o1", "robot_1"]]
    assert len(d["racks"]) == 10 and any(s["type"] == "dock" for s in d["stations"])
