"""video_cams.py: camera aim, pallet events from a run record, cut plan, sim-time frame resampling."""

import importlib.util
import math
import pathlib
import sys
import xml.etree.ElementTree as ET

HERE = pathlib.Path(__file__).resolve().parents[1]


def _mod():
    spec = importlib.util.spec_from_file_location("video_cams", HERE / "video_cams.py")
    mod = importlib.util.module_from_spec(spec)
    sys.modules["video_cams"] = mod      # dataclasses look their module up while the class is built
    spec.loader.exec_module(mod)
    return mod


vc = _mod()


def test_look_at_points_the_camera_x_axis_at_the_target():
    r, p, y = vc.look_at_rpy((0, 0, 10), (10, 10, 0))
    assert r == 0.0
    assert math.isclose(y, math.pi / 4)
    assert math.isclose(p, math.atan2(10, math.hypot(10, 10)))   # positive pitch = nose down in Gazebo
    _, p_up, y_back = vc.look_at_rpy((0, 0, 0), (-5, 0, 5))
    assert p_up < 0 and math.isclose(abs(y_back), math.pi)


def test_camera_sdf_is_valid_and_carries_the_settings():
    cam = vc.CAMERAS["wide"]
    root = ET.fromstring(vc.camera_sdf(cam, "/video/wide").split("?>", 1)[1])
    s = root.find("./model/link/sensor")
    assert root.find("./model/static").text == "true"
    assert s.get("type") == "camera"
    assert s.find("topic").text == "/video/wide"
    assert float(s.find("update_rate").text) == cam.rate_hz
    assert int(s.find("./camera/image/width").text) == cam.width


def test_every_camera_sits_inside_or_above_the_warehouse_and_looks_into_it():
    for cam in vc.CAMERAS.values():
        assert cam.eye[2] > 2.5                                   # above the walls (2.5 m)
        assert 0 <= cam.target[0] <= 20 and 0 <= cam.target[1] <= 32


def _states(rows):
    return [{"t": t, "robot_id": r, "order_id": o, "mode": m} for t, r, o, m in rows]


def test_pallet_moves_finds_pickup_and_setdown():
    states = _states([
        (10, "robot_2", "p1", "NAVIGATING"), (20, "robot_2", "p1", "LIFTING"), (24, "robot_2", "p1", "LIFTING"),
        (26, "robot_2", "p1", "CARRYING"), (60, "robot_2", "p1", "CARRYING"), (62, "robot_2", "p1", "LIFTING"),
        (66, "robot_2", "p1", "LIFTING"), (68, "robot_2", "", "IDLE"),
        (21, "robot_1", "p1", "LIFTING"),                         # another robot's stale row: ignored
        (30, "robot_3", "s1", "UNLOADING"),
    ])
    orders = [{"order_id": "p1", "order_type": "pallet", "state": "DELIVERED", "robot_id": "robot_2", "t": 67},
              {"order_id": "s1", "order_type": "shelf", "state": "DELIVERED", "robot_id": "robot_3", "t": 31}]
    (m,) = vc.pallet_moves(states, orders)
    assert (m.order_id, m.robot_id) == ("p1", "robot_2")
    assert (m.lift_start, m.lift_end, m.setdown_start, m.setdown_end, m.delivered) == (20, 26, 62, 66, 67)


def test_undelivered_pallet_is_not_a_move():
    states = _states([(20, "robot_2", "p1", "LIFTING"), (26, "robot_2", "p1", "CARRYING")])
    orders = [{"order_id": "p1", "order_type": "pallet", "state": "IN_TRANSIT", "robot_id": "robot_2", "t": 26}]
    assert vc.pallet_moves(states, orders) == []


def test_cut_plan_covers_the_run_without_gaps_and_shows_the_pallet_close_up():
    m = vc.PalletMove("p1", "robot_2", 40, 46, 120, 126, 127)
    cuts = vc.plan_cuts(m, 5, 365)
    assert cuts[0].t0 == 5 and cuts[-1].t1 == 365
    assert all(a.t1 == b.t0 for a, b in zip(cuts, cuts[1:]))
    cams = [c.cam for c in cuts]
    assert cams == ["wide", "pallets", "wide", "docks", "wide"]
    pick = cuts[1]
    assert pick.t0 <= m.lift_start and pick.t1 >= m.lift_end and pick.speed < cuts[0].speed
    drop = cuts[3]
    assert drop.t0 <= m.setdown_start and drop.t1 >= m.setdown_end


def test_cut_plan_without_a_pallet_move_is_one_wide_shot():
    assert [c.cam for c in vc.plan_cuts(None, 0, 100)] == ["wide"]


def test_frames_for_picks_the_nearest_frame_in_sim_time_and_repeats_over_gaps():
    stamps = [0.0, 0.1, 0.2, 0.3, 0.7, 0.8]                     # frames 0.4-0.6 dropped
    idx = vc.frames_for(stamps, 0.0, 0.8, 1.0, 10)
    assert len(idx) == 8
    assert idx[:4] == [0, 1, 2, 3]
    assert idx[4] == 3 and idx[6] == 4                          # time does not shift after the gap
    assert idx == sorted(idx)


def test_frames_for_speed_up_skips_frames():
    stamps = [i / 10 for i in range(100)]
    idx = vc.frames_for(stamps, 0.0, 9.0, 3.0, 10)
    assert len(idx) == 30 and idx[1] == 3 and idx[-1] == 87
