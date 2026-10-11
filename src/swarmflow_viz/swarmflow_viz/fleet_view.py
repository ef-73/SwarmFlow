"""Marker and dashboard builders for the Foxglove fleet view (T021). Pure functions: no node, no clock; time is an
argument. Robot geometry follows ``swarmflow_description`` (body 0.60 × 0.50 × 0.25 m), colours follow
``swarmflow_gazebo/launch/sim.launch.py`` so Foxglove and Gazebo show the same robot in the same colour.

Robot shapes are attached to the robot's own TF frames (``<robot>/base_footprint``, published by ``tf_relay``) with
``frame_locked``, so they move as smoothly as TF (30 Hz) rather than at the 5 Hz of ``RobotState``.
"""

from __future__ import annotations

import math
from typing import Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

from builtin_interfaces.msg import Duration, Time
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

from .world_scene import Shape

MAP = "map"
RGBA = Tuple[float, float, float, float]

#: same values as swarmflow_gazebo/launch/sim.launch.py COLORS
ROBOT_RGBA: Dict[str, RGBA] = {"robot_1": (0.10, 0.45, 0.85, 1.0), "robot_2": (0.95, 0.55, 0.10, 1.0),
                               "robot_3": (0.20, 0.70, 0.30, 1.0), "robot_4": (0.75, 0.20, 0.70, 1.0)}
GREY: RGBA = (0.5, 0.5, 0.5, 1.0)
WHITE: RGBA = (1.0, 1.0, 1.0, 1.0)
DARK: RGBA = (0.12, 0.12, 0.12, 1.0)

#: Nav2 footprint (src/swarmflow_nav/config/nav2_params.yaml), base_footprint frame
DEFAULT_FOOTPRINT: Tuple[Tuple[float, float], ...] = ((0.35, 0.30), (0.35, -0.30), (-0.35, -0.30), (-0.35, 0.30))

BODY_L, BODY_W, BODY_H, BODY_Z0 = 0.60, 0.50, 0.25, 0.05    # body bottom 0.05 m above the floor
WHEEL_R, WHEEL_W, TRACK = 0.075, 0.04, 0.42
LIDAR_X, LIDAR_Z = BODY_L / 2 + 0.035, 0.20

#: symbol per station kind: (label suffix, colour)
PLACE_STYLE: Dict[str, Tuple[str, RGBA]] = {"loading": ("pickup", (0.10, 0.70, 0.20, 1.0)),
                                            "delivery": ("drop-off", (0.10, 0.30, 0.90, 1.0)),
                                            "park": ("home", (0.45, 0.45, 0.45, 1.0)),
                                            "hold": ("wait", (0.95, 0.80, 0.10, 1.0))}

OK, WARN, ERROR = 0, 1, 2                                    # DiagnosticStatus levels
MODE_LEVEL = {"IDLE": OK, "NAVIGATING": OK, "LOADING": OK, "UNLOADING": OK, "WAITING_RESERVATION": WARN,
              "STUCK": ERROR, "FAULT": ERROR}
STALE_WARN_S = 2.0
STALE_ERROR_S = 5.0


def robot_rgba(robot_id: str) -> RGBA:
    return ROBOT_RGBA.get(robot_id, GREY)


def stamp(sec: float) -> Time:
    sec = max(0.0, float(sec))
    s = int(sec)
    return Time(sec=s, nanosec=min(int((sec - s) * 1e9), 999_999_999))


def marker(ns: str, mid: int, mtype: int, frame: str, t: float, rgba: RGBA, locked: bool = False) -> Marker:
    m = Marker()
    m.header.frame_id = frame
    m.header.stamp = stamp(t)
    m.ns, m.id, m.type, m.action = ns, mid, mtype, Marker.ADD
    m.pose.orientation.w = 1.0
    m.color.r, m.color.g, m.color.b, m.color.a = (float(c) for c in rgba)
    m.frame_locked = locked
    return m


def delete(ns: str, mid: int, frame: str, t: float) -> Marker:
    m = marker(ns, mid, Marker.CUBE, frame, t, WHITE)
    m.action = Marker.DELETE
    return m


def _set_pose(m: Marker, x: float, y: float, z: float, yaw: float = 0.0, roll: float = 0.0) -> None:
    m.pose.position.x, m.pose.position.y, m.pose.position.z = float(x), float(y), float(z)
    # quaternion of R = Rz(yaw) · Rx(roll)
    cr, sr, cy, sy = math.cos(roll / 2), math.sin(roll / 2), math.cos(yaw / 2), math.sin(yaw / 2)
    m.pose.orientation.x = cy * sr
    m.pose.orientation.y = sy * sr
    m.pose.orientation.z = sy * cr
    m.pose.orientation.w = cy * cr


def _text(ns: str, mid: int, frame: str, t: float, x: float, y: float, z: float, text: str, h: float,
          rgba: RGBA = WHITE, locked: bool = False) -> Marker:
    m = marker(ns, mid, Marker.TEXT_VIEW_FACING, frame, t, rgba, locked)
    _set_pose(m, x, y, z)
    m.scale.z = h
    m.text = text
    return m


# ---- static scene -------------------------------------------------------------------------------------------------

def shape_marker(ns: str, mid: int, s: Shape, t: float) -> Marker:
    m = marker(ns, mid, Marker.CUBE if s.kind == "box" else Marker.CYLINDER, MAP, t, s.rgba)
    _set_pose(m, *s.pose)
    m.scale.x, m.scale.y, m.scale.z = s.size
    return m


def scene_markers(shapes: Sequence[Shape], bounds: Optional[Tuple[float, float, float, float]],
                  places: Iterable[Tuple[str, str, float, float]], t: float) -> MarkerArray:
    """World shapes + floor + place symbols and labels. ``places`` = (name, kind, x, y)."""
    ma = MarkerArray()
    if bounds is not None:                                     # floor slab, slightly below z = 0
        x0, y0, x1, y1 = bounds
        fl = marker("floor", 0, Marker.CUBE, MAP, t, (0.82, 0.82, 0.80, 1.0))
        _set_pose(fl, (x0 + x1) / 2, (y0 + y1) / 2, -0.011)
        fl.scale.x, fl.scale.y, fl.scale.z = x1 - x0, y1 - y0, 0.02
        ma.markers.append(fl)
    for i, s in enumerate(shapes):
        ma.markers.append(shape_marker("world", i, s, t))
    for i, (name, kind, x, y) in enumerate(sorted(places)):
        label, rgba = PLACE_STYLE.get(kind, (kind, GREY))
        if kind == "loading":                                   # pickup: arrow up (the package goes onto the robot)
            sym = marker("place_symbols", i, Marker.ARROW, MAP, t, rgba)
            sym.points = [Point(x=x, y=y, z=0.05), Point(x=x, y=y, z=0.55)]
            sym.scale.x, sym.scale.y, sym.scale.z = 0.08, 0.2, 0.2
        elif kind == "delivery":                                # drop-off: arrow down
            sym = marker("place_symbols", i, Marker.ARROW, MAP, t, rgba)
            sym.points = [Point(x=x, y=y, z=0.6), Point(x=x, y=y, z=0.08)]
            sym.scale.x, sym.scale.y, sym.scale.z = 0.08, 0.2, 0.2
        elif kind == "hold":                                    # waiting spot: small flat disc
            sym = marker("place_symbols", i, Marker.CYLINDER, MAP, t, (*rgba[:3], 0.7))
            _set_pose(sym, x, y, 0.006)
            sym.scale.x = sym.scale.y = 0.30
            sym.scale.z = 0.01
        else:                                                   # home: big "H" on the pad
            sym = _text("place_symbols", i, MAP, t, x, y, 0.12, "H", 0.5, (0.95, 0.95, 0.95, 1.0))
        ma.markers.append(sym)
        if kind != "hold":
            ma.markers.append(_text("place_labels", i, MAP, t, x, y, 0.85, f"{name} {label}", 0.25))
    return ma


def package_markers(poses: Mapping[str, Tuple[float, float, float, float]], shape_of: Mapping[str, Shape],
                    t: float) -> MarkerArray:
    """Packages at their current Gazebo poses (name → x, y, z, yaw)."""
    ma = MarkerArray()
    for i, name in enumerate(sorted(poses)):
        s = shape_of.get(name)
        if s is None:
            continue
        ma.markers.append(shape_marker("packages", i, Shape(name, s.kind, poses[name], s.size, s.rgba), t))
    return ma


# ---- robots -------------------------------------------------------------------------------------------------------

def robot_markers(robot_id: str, idx: int, t: float, mode: str, footprint: Sequence[Tuple[float, float]],
                  prefix: str = "") -> List[Marker]:
    """Body, wheels, lidar, footprint outline and label, attached to ``<robot>/base_footprint``."""
    frame = f"{prefix or robot_id}/base_footprint"
    rgba = robot_rgba(robot_id)
    out: List[Marker] = []
    body = marker("robot_body", idx, Marker.CUBE, frame, t, rgba, True)
    _set_pose(body, 0.0, 0.0, BODY_Z0 + BODY_H / 2)
    body.scale.x, body.scale.y, body.scale.z = BODY_L, BODY_W, BODY_H
    out.append(body)
    top = marker("robot_top", idx, Marker.CUBE, frame, t, tuple(min(1.0, c + 0.25) for c in rgba[:3]) + (1.0,), True)
    _set_pose(top, 0.0, 0.0, BODY_Z0 + BODY_H + 0.005)
    top.scale.x, top.scale.y, top.scale.z = BODY_L - 0.02, BODY_W - 0.02, 0.01
    out.append(top)
    for k, side in enumerate((1.0, -1.0)):
        w = marker("robot_wheels", idx * 2 + k, Marker.CYLINDER, frame, t, DARK, True)
        _set_pose(w, 0.0, side * TRACK / 2, WHEEL_R, roll=math.pi / 2)
        w.scale.x = w.scale.y = 2 * WHEEL_R
        w.scale.z = WHEEL_W
        out.append(w)
    lid = marker("robot_lidar", idx, Marker.CYLINDER, frame, t, DARK, True)
    _set_pose(lid, LIDAR_X, 0.0, LIDAR_Z)
    lid.scale.x = lid.scale.y = 0.06
    lid.scale.z = 0.04
    out.append(lid)
    fp = marker("robot_footprint", idx, Marker.LINE_STRIP, frame, t, (*rgba[:3], 0.9), True)
    pts = list(footprint) + [footprint[0]]
    fp.points = [Point(x=float(x), y=float(y), z=0.015) for x, y in pts]
    fp.scale.x = 0.03
    out.append(fp)
    out.append(_text("robot_label", idx, frame, t, 0.0, 0.0, 0.85, f"{robot_id}\n{mode}", 0.22, rgba, True))
    return out


def path_marker(robot_id: str, idx: int, t: float, points: Sequence[Tuple[float, float]],
                min_step: float = 0.1) -> Marker:
    """The robot's current Nav2 plan, drawn on the floor in its colour; DELETE when there is no plan."""
    if len(points) < 2:
        return delete("robot_path", idx, MAP, t)
    kept = [points[0]]
    for p in points[1:-1]:
        if math.hypot(p[0] - kept[-1][0], p[1] - kept[-1][1]) >= min_step:
            kept.append(p)
    kept.append(points[-1])
    m = marker("robot_path", idx, Marker.LINE_STRIP, MAP, t, (*robot_rgba(robot_id)[:3], 0.85))
    m.points = [Point(x=float(x), y=float(y), z=0.02) for x, y in kept]
    m.scale.x = 0.07
    return m


def goal_markers(robot_id: str, idx: int, t: float, goal: Optional[Tuple[float, float]], label: str) -> List[Marker]:
    """A ring on the floor and a pin above the robot's current goal, in the robot's colour."""
    if goal is None:
        return [delete("robot_goal_ring", idx, MAP, t), delete("robot_goal_pin", idx, MAP, t),
                delete("robot_goal_label", idx, MAP, t)]
    rgba = robot_rgba(robot_id)
    x, y = goal
    ring = marker("robot_goal_ring", idx, Marker.CYLINDER, MAP, t, (*rgba[:3], 0.45))
    _set_pose(ring, x, y, 0.012)
    ring.scale.x = ring.scale.y = 1.0
    ring.scale.z = 0.01
    pin = marker("robot_goal_pin", idx, Marker.ARROW, MAP, t, rgba)
    pin.points = [Point(x=x, y=y, z=1.6), Point(x=x, y=y, z=0.7)]
    pin.scale.x, pin.scale.y, pin.scale.z = 0.07, 0.18, 0.2
    lab = _text("robot_goal_label", idx, MAP, t, x, y, 1.8, label, 0.22, rgba)
    return [ring, pin, lab]


# ---- dashboard ----------------------------------------------------------------------------------------------------

def _level(n: int):
    """DiagnosticStatus.level is a ROS ``byte`` (``bytes`` in rclpy)."""
    return bytes([n]) if isinstance(DiagnosticStatus.OK, bytes) else n


def level_of(status) -> int:
    lv = status.level
    return lv[0] if isinstance(lv, (bytes, bytearray)) else int(lv)


def dashboard(t: float, states: Mapping[str, Mapping], expected: Sequence[str]) -> DiagnosticArray:
    """One DiagnosticStatus per robot. ``states[robot]``: mode, task_id, order_id, order_state, goal, next_vertex,
    speed, loaded, leases, fault_reason, stamp (sim s). Missing or stale robots are WARN / ERROR."""
    arr = DiagnosticArray()
    arr.header.stamp = stamp(t)
    for rid in sorted(set(expected) | set(states)):
        st = DiagnosticStatus()
        st.name = rid
        st.hardware_id = rid
        s = states.get(rid)
        if s is None:
            st.level = _level(ERROR)
            st.message = "no state received"
            arr.status.append(st)
            continue
        mode = str(s.get("mode", ""))
        age = max(0.0, t - float(s.get("stamp", t)))
        level = MODE_LEVEL.get(mode, WARN)
        if age >= STALE_ERROR_S:
            level = ERROR
        elif age >= STALE_WARN_S:
            level = max(level, WARN)
        st.level = _level(level)
        parts = [mode]
        if s.get("goal"):
            parts.append(f"→ {s['goal']}")
        if s.get("order_id"):
            parts.append(f"order {s['order_id']}")
        if s.get("loaded"):
            parts.append("loaded")
        if s.get("fault_reason"):
            parts.append(str(s["fault_reason"]))
        if age >= STALE_WARN_S:
            parts.append(f"no update for {age:.0f}s")
        st.message = " · ".join(parts)
        values = [("mode", mode), ("task", s.get("task_id", "")), ("order", s.get("order_id", "")),
                  ("order state", s.get("order_state", "")), ("goal", s.get("goal", "")),
                  ("next vertex", s.get("next_vertex", "")), ("speed m/s", f"{float(s.get('speed', 0.0)):.2f}"),
                  ("loaded", "yes" if s.get("loaded") else "no"), ("leases", ", ".join(s.get("leases", ()))),
                  ("fault", s.get("fault_reason", "")), ("last update s ago", f"{age:.1f}")]
        st.values = [KeyValue(key=k, value=str(v)) for k, v in values]
        arr.status.append(st)
    return arr


# ---- goals --------------------------------------------------------------------------------------------------------

def current_goal(state: Mapping, order: Optional[Mapping], order_state: str,
                 kinds: Optional[Mapping[str, str]] = None) -> Optional[Tuple[str, str]]:
    """(vertex, label) of where the robot is heading now, or None.

    ``state``: mode, task_id, order_id, remaining_route; ``order``: pickup_vertex, dropoff_vertex (None if the order
    was published before this node started); ``kinds``: vertex → kind. Before pickup the goal is the pickup station,
    after it the drop-off; a move-only (park) task heads for its last vertex. Without the order, the next station on
    the remaining route is the goal."""
    route = list(state.get("remaining_route", ()))
    oid = state.get("order_id", "")
    if oid and order is not None:
        picked = order_state in ("PICKING_UP", "IN_TRANSIT") or state.get("mode") == "UNLOADING"
        if state.get("mode") == "LOADING" or not picked:
            return order["pickup_vertex"], f"pickup {oid}"
        return order["dropoff_vertex"], f"drop-off {oid}"
    if not state.get("task_id") or not route:
        return None
    if str(state["task_id"]).startswith("park_"):
        return route[-1], "home"
    names = {"loading": "pickup", "delivery": "drop-off", "park": "home"}
    for v in route:
        kind = (kinds or {}).get(v, "")
        if kind in names:
            return v, f"{names[kind]} {oid}".strip()
    return route[-1], "goal"
