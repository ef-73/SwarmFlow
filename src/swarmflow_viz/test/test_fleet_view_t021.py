"""T021: world parser, frame prefixing, robot overlay markers, goals and dashboard."""

from __future__ import annotations

import math
import pathlib

import pytest

pytest.importorskip("visualization_msgs")
from geometry_msgs.msg import TransformStamped  # noqa: E402
from visualization_msgs.msg import Marker  # noqa: E402

from swarmflow_viz import fleet_view as fv  # noqa: E402
from swarmflow_viz.tf_relay import finite_ranges, prefix_transforms, prefixed  # noqa: E402
from swarmflow_viz.world_scene import compose, load_world, parse_world  # noqa: E402

WORLD = pathlib.Path(__file__).resolve().parents[3] / "layouts" / "standard" / "generated" / "world.sdf"


# ---- world_scene --------------------------------------------------------------------------------------------------

def test_standard_world_shapes_match_the_sdf():
    w = load_world(str(WORLD))
    assert w.name == "standard"
    models = {s.model for s in w.static}
    assert {"wall_north", "wall_south", "wall_east", "wall_west"} <= models
    assert {f"rack_R{i}" for i in range(1, 5)} <= models
    assert "ground_plane" not in models                       # planes are not drawn
    assert len([m for m in models if m.startswith("station_")]) == 10
    assert len(w.dynamic) == 12 and all(s.model.startswith("pkg_") for s in w.dynamic)
    r1 = next(s for s in w.static if s.model == "rack_R1")
    assert r1.kind == "box" and r1.size == (10.0, 1.0, 2.0)
    assert r1.pose[:3] == (11.0, 2.7, 1.0)
    l1 = next(s for s in w.static if s.model == "station_L1")
    assert l1.rgba == pytest.approx((0.1, 0.7, 0.2, 1.0))


def test_poses_compose_with_model_yaw_and_cylinders_parse():
    sdf = """<sdf version="1.9"><world name="w"><model name="m"><pose>1 2 0 0 0 1.5707963</pose>
      <link name="l"><pose>1 0 0.5 0 0 0</pose><visual name="v"><geometry><cylinder><radius>0.2</radius>
      <length>0.4</length></cylinder></geometry><material><diffuse>1 0 0</diffuse></material></visual></link>
      </model></world></sdf>"""
    s = parse_world(sdf).static[0]
    assert s.kind == "cylinder" and s.size == (0.4, 0.4, 0.4)
    assert s.pose[0] == pytest.approx(1.0) and s.pose[1] == pytest.approx(3.0) and s.pose[2] == 0.5
    assert s.rgba == (1.0, 0.0, 0.0, 1.0)
    assert compose((0, 0, 0, math.pi), (1, 0, 0, 0))[0] == pytest.approx(-1.0)


# ---- tf_relay -----------------------------------------------------------------------------------------------------

def test_frames_get_the_robot_prefix_but_map_stays_shared():
    assert prefixed("base_link", "robot_2") == "robot_2/base_link"
    assert prefixed("/odom", "robot_2") == "robot_2/odom"
    assert prefixed("map", "robot_2") == "map"
    assert prefixed("robot_2/odom", "robot_2") == "robot_2/odom"
    t = TransformStamped()
    t.header.frame_id, t.child_frame_id = "map", "odom"
    out = prefix_transforms([t], "robot_1")
    assert (out[0].header.frame_id, out[0].child_frame_id) == ("map", "robot_1/odom")
    assert t.child_frame_id == "odom"                         # input untouched
    assert finite_ranges([1.0, float("inf"), float("nan"), 2.5]) == [1.0, 0.0, 0.0, 2.5]


# ---- robots, paths, goals -----------------------------------------------------------------------------------------

def test_robot_markers_are_locked_to_the_robot_frame_in_its_colour():
    ms = fv.robot_markers("robot_2", 1, 5.0, "NAVIGATING", fv.DEFAULT_FOOTPRINT)
    assert {m.header.frame_id for m in ms} == {"robot_2/base_footprint"}
    assert all(m.frame_locked for m in ms)
    body = next(m for m in ms if m.ns == "robot_body")
    assert (body.color.r, body.color.g, body.color.b) == pytest.approx(fv.ROBOT_RGBA["robot_2"][:3])
    fp = next(m for m in ms if m.ns == "robot_footprint")
    assert fp.type == Marker.LINE_STRIP and len(fp.points) == 5
    assert (fp.points[0].x, fp.points[0].y) == (fp.points[-1].x, fp.points[-1].y)   # closed outline
    label = next(m for m in ms if m.ns == "robot_label")
    assert "robot_2" in label.text and "NAVIGATING" in label.text


def test_path_is_thinned_keeps_ends_and_deletes_when_empty():
    pts = [(i * 0.01, 0.0) for i in range(101)]               # 1 m in 1 cm steps
    m = fv.path_marker("robot_1", 0, 1.0, pts)
    assert m.action == Marker.ADD and m.points[0].x == 0.0 and m.points[-1].x == pytest.approx(1.0)
    assert len(m.points) <= 12
    assert fv.path_marker("robot_1", 0, 1.0, []).action == Marker.DELETE


def test_goal_is_pickup_before_and_dropoff_after_picking():
    order = {"pickup_vertex": "L1", "dropoff_vertex": "D2"}
    s = {"mode": "NAVIGATING", "order_id": "o1", "task_id": "task_o1", "remaining_route": ("X_1_0", "L1", "D2")}
    assert fv.current_goal(s, order, "ASSIGNED") == ("L1", "pickup o1")
    assert fv.current_goal(dict(s, mode="LOADING"), order, "PICKING_UP") == ("L1", "pickup o1")
    assert fv.current_goal(s, order, "IN_TRANSIT") == ("D2", "drop-off o1")
    park = {"mode": "NAVIGATING", "order_id": "", "task_id": "park_robot_1_1", "remaining_route": ("X_0_0", "P1")}
    assert fv.current_goal(park, None, "") == ("P1", "home")
    assert fv.current_goal({"mode": "IDLE", "task_id": "", "remaining_route": ()}, None, "") is None
    kinds = {"X_1_0": "intersection", "L1": "loading", "D2": "delivery"}
    assert fv.current_goal(s, None, "", kinds) == ("L1", "pickup o1")             # order not seen: from the route
    ms = fv.goal_markers("robot_1", 0, 1.0, None, "")
    assert all(m.action == Marker.DELETE for m in ms)


def test_scene_has_floor_world_and_place_symbols():
    w = load_world(str(WORLD))
    places = [("L1", "loading", 1.3, 3.85), ("D1", "delivery", 20.7, 3.85), ("P1", "park", 1.3, 1.1),
              ("H_1_1", "hold", 4.4, 2.7)]
    ma = fv.scene_markers(w.static, (0.0, 0.0, 22.0, 12.3), places, 0.0)
    ns = [m.ns for m in ma.markers]
    assert ns.count("floor") == 1 and ns.count("world") == len(w.static)
    assert ns.count("place_symbols") == 4 and ns.count("place_labels") == 3      # holds: symbol, no label
    labels = {m.text for m in ma.markers if m.ns == "place_labels"}
    assert {"L1 pickup", "D1 drop-off", "P1 home"} == labels


# ---- dashboard ----------------------------------------------------------------------------------------------------

def test_dashboard_levels_and_stale_robots():
    states = {"robot_1": {"mode": "NAVIGATING", "stamp": 99.5, "goal": "D2 drop-off o1", "order_id": "o1",
                          "loaded": True, "speed": 0.42},
              "robot_2": {"mode": "STUCK", "stamp": 100.0, "fault_reason": "NAV_FAILED_AT:X_1_1"},
              "robot_3": {"mode": "IDLE", "stamp": 96.0}}
    arr = fv.dashboard(100.0, states, ["robot_1", "robot_2", "robot_3", "robot_4"])
    by = {s.name: s for s in arr.status}
    assert [fv.level_of(by[r]) for r in ("robot_1", "robot_2", "robot_3", "robot_4")] == [0, 2, 1, 2]
    assert "D2 drop-off o1" in by["robot_1"].message and "loaded" in by["robot_1"].message
    assert "no update for 4s" in by["robot_3"].message
    assert by["robot_4"].message == "no state received"
    vals = {kv.key: kv.value for kv in by["robot_1"].values}
    assert vals["speed m/s"] == "0.42" and vals["loaded"] == "yes"


# ---- Gazebo overlay -----------------------------------------------------------------------------------------------

def test_gazebo_overlay_status_rings_paths_goals_and_board():
    import json

    from swarmflow_viz.gz_overlay import LEVEL_RGBA, board_json, build_overlay
    states = {"robot_1": {"x": 1.0, "y": 2.0, "mode": "NAVIGATING", "task_id": "t1"},
              "robot_2": {"x": 5.0, "y": 2.0, "mode": "STUCK", "task_id": ""}}
    out = build_overlay(["robot_1", "robot_2", "robot_3"], states, {"robot_1": 0, "robot_2": 2},
                        {"robot_1": [(1, 2), (3, 2)]}, {"robot_1": (3.0, 2.0)})
    by = {d["id"]: d for d in out}
    assert by[0]["type"] == "cylinder" and by[0]["pose"][:2] == (1.0, 2.0) and by[0]["rgba"] == LEVEL_RGBA[0]
    assert by[10]["rgba"] == LEVEL_RGBA[2]                                     # stuck robot: red ring
    assert by[1]["type"] == "line_strip" and len(by[1]["points"]) == 2
    assert by[1]["rgba"] == fv.ROBOT_RGBA["robot_1"]
    assert by[2]["type"] == "cylinder" and by[2]["pose"][:2] == (3.0, 2.0)
    assert by[11]["type"] == "delete" and by[12]["type"] == "delete"          # no task: no path, no goal
    assert [by[20 + k]["type"] for k in range(3)] == ["delete"] * 3            # robot without state
    dash = fv.dashboard(10.0, {"robot_1": {"mode": "NAVIGATING", "stamp": 9.5, "goal": "L1 pickup o1",
                                           "order_id": "o1", "order_state": "ASSIGNED", "speed": 0.4}}, ["robot_1"])
    board = json.loads(board_json(10.0, dash, {"robot_1": {"speed": 0.4}}))
    assert board["t"] == 10.0
    assert board["robots"] == [{"name": "robot_1", "color": "#1a73d9", "level": 0, "mode": "NAVIGATING",
                                "goal": "L1 pickup o1", "order": "o1", "order_state": "ASSIGNED", "loaded": False,
                                "speed": 0.4, "age": 0.5, "fault": ""}]
