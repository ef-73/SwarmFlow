"""T069: the SWARMFLOW_SMOOTH profile applied to the merged v2 Nav2 parameters."""

import importlib.util
import pathlib

import pytest
import yaml

ROOT = pathlib.Path(__file__).resolve().parents[2]
NAV = ROOT / "src" / "swarmflow_nav"


def _launch():
    pytest.importorskip("launch_ros")
    pytest.importorskip("nav2_common")
    spec = importlib.util.spec_from_file_location("robot_launch", NAV / "launch" / "robot.launch.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _merged(mod):
    base = yaml.safe_load((NAV / "config" / "nav2_params.yaml").read_text(encoding="utf-8"))
    mod._deep_merge(base, yaml.safe_load((NAV / "config" / "nav2_params_v2.yaml").read_text(encoding="utf-8")))
    return base


def test_smoothing_limits_acceleration_in_controller_and_smoother_only():
    mod = _launch()
    base = _merged(mod)
    before = dict(base["controller_server"]["ros__parameters"]["FollowPath"])
    mod.apply_smoothing(base)
    fp = base["controller_server"]["ros__parameters"]["FollowPath"]
    a, aw = mod.SMOOTH_ACCEL
    assert (fp["ax_max"], fp["ax_min"], fp["az_max"]) == (a, -a, aw)
    assert a < before["ax_max"] and aw < before["az_max"] and fp["wz_std"] < before["wz_std"]
    assert fp["critics"] == before["critics"] and "TwirlingCritic" not in fp["critics"]   # s069_1: no U-turns
    vs = base["velocity_smoother"]["ros__parameters"]
    assert vs["max_accel"] == [a, 0.0, aw] and vs["max_decel"] == [-a, 0.0, -aw]
    assert base["planner_server"]["ros__parameters"]["GridBased"]["smoother"]["w_smooth"] == 0.3   # unchanged


def test_coordination_switches_are_on_unless_set_to_0(monkeypatch):
    mod = _launch()
    for name in ("SWARMFLOW_YIELD", "SWARMFLOW_UNJAM", "SWARMFLOW_SMOOTH"):
        monkeypatch.delenv(name, raising=False)
        assert mod.switch_on(name)
        for off in ("0", "false", "OFF"):
            monkeypatch.setenv(name, off)
            assert not mod.switch_on(name)
        monkeypatch.setenv(name, "1")
        assert mod.switch_on(name)
