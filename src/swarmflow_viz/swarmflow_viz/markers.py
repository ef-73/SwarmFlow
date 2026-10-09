"""Pure MarkerArray builders for Foxglove (design 11.1). Frame ``map``; time is an argument."""

from __future__ import annotations

import math
import re
from typing import Iterable, Mapping

from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

from swarmflow_core.api import LeaseState

FRAME = "map"
ZONE_COLORS = {"FREE": (0.2, 0.8, 0.2, 0.35), "GRANTED": (1.0, 0.6, 0.0, 0.45),
               "OCCUPIED_UNKNOWN": (0.9, 0.1, 0.1, 0.55)}
ROBOT_COLORS = {1: (0.15, 0.35, 0.95, 1.0), 2: (1.0, 0.55, 0.05, 1.0), 3: (0.15, 0.75, 0.25, 1.0)}
_DEFAULT_ROBOT_COLOR = (0.6, 0.6, 0.6, 1.0)
STATION_COLORS = {"loading": (0.3, 0.6, 1.0, 0.8), "delivery": (0.8, 0.3, 0.9, 0.8), "park": (0.7, 0.7, 0.7, 0.8)}


def _stamp(sec: float) -> Time:
    sec = max(0.0, float(sec))
    s = int(sec)
    return Time(sec=s, nanosec=min(int((sec - s) * 1e9), 999_999_999))


def _marker(ns: str, mid: int, mtype: int, stamp_sec: float, color) -> Marker:
    m = Marker()
    m.header.frame_id = FRAME
    m.header.stamp = _stamp(stamp_sec)
    m.ns = ns
    m.id = mid
    m.type = mtype
    m.action = Marker.ADD
    m.pose.orientation.w = 1.0
    m.color.r, m.color.g, m.color.b, m.color.a = (float(c) for c in color)
    return m


def robot_number(robot_id: str) -> int:
    mt = re.search(r"(\d+)\s*$", robot_id)
    return int(mt.group(1)) if mt else 0


def robot_markers(states: Iterable[Mapping], stamp_sec: float) -> MarkerArray:
    ma = MarkerArray()
    for s in sorted(states, key=lambda s: s["robot_id"]):
        rid = s["robot_id"]
        n = robot_number(rid)
        body = _marker("robots", n, Marker.CUBE, stamp_sec, ROBOT_COLORS.get(n, _DEFAULT_ROBOT_COLOR))
        body.pose.position.x, body.pose.position.y, body.pose.position.z = float(s["x"]), float(s["y"]), 0.175
        yaw = float(s.get("yaw", 0.0))
        body.pose.orientation.z, body.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        body.scale.x, body.scale.y, body.scale.z = 0.60, 0.50, 0.25
        ma.markers.append(body)
        text = f"{rid} {s.get('mode', '')}"
        if s.get("order_id"):
            text += f" {s['order_id']}"
        lab = _marker("robot_labels", n, Marker.TEXT_VIEW_FACING, stamp_sec, (1.0, 1.0, 1.0, 1.0))
        lab.pose.position.x, lab.pose.position.y, lab.pose.position.z = float(s["x"]), float(s["y"]), 0.8
        lab.scale.z = 0.3
        lab.text = text
        ma.markers.append(lab)
    return ma


def _zone_state_name(state) -> str:
    name = state.value if isinstance(state, LeaseState) else str(state)
    return name if name in ("GRANTED", "OCCUPIED_UNKNOWN") else "FREE"


def _pt(x: float, y: float, z: float = 0.02) -> Point:
    return Point(x=float(x), y=float(y), z=float(z))


def zone_markers(graph, lease_states: Mapping[str, LeaseState], stamp_sec: float) -> MarkerArray:
    ma = MarkerArray()
    for i, name in enumerate(sorted(graph.zones)):
        poly = graph.zones[name].polygon
        st = _zone_state_name(lease_states.get(name, LeaseState.RELEASED))
        m = _marker("zones", i, Marker.TRIANGLE_LIST, stamp_sec, ZONE_COLORS[st])
        m.scale.x = m.scale.y = m.scale.z = 1.0
        for k in range(1, len(poly) - 1):                       # triangle fan (polygons are convex rectangles)
            for p in (poly[0], poly[k], poly[k + 1]):
                m.points.append(_pt(*p))
        ma.markers.append(m)
        cx = sum(p[0] for p in poly) / len(poly)
        cy = sum(p[1] for p in poly) / len(poly)
        lab = _marker("zone_labels", i, Marker.TEXT_VIEW_FACING, stamp_sec, (1.0, 1.0, 1.0, 1.0))
        lab.pose.position.x, lab.pose.position.y, lab.pose.position.z = cx, cy, 0.3
        lab.scale.z = 0.35
        lab.text = f"{name} {st}"
        ma.markers.append(lab)
    return ma


def station_markers(graph, stamp_sec: float) -> MarkerArray:
    ma = MarkerArray()
    i = 0
    for name in sorted(graph.vertices):
        v = graph.vertices[name]
        if v.kind not in STATION_COLORS:
            continue
        pad = _marker("stations", i, Marker.CYLINDER, stamp_sec, STATION_COLORS[v.kind])
        pad.pose.position.x, pad.pose.position.y, pad.pose.position.z = float(v.x), float(v.y), 0.025
        pad.scale.x = pad.scale.y = 0.8
        pad.scale.z = 0.05
        ma.markers.append(pad)
        lab = _marker("station_labels", i, Marker.TEXT_VIEW_FACING, stamp_sec, (1.0, 1.0, 1.0, 1.0))
        lab.pose.position.x, lab.pose.position.y, lab.pose.position.z = float(v.x), float(v.y), 0.5
        lab.scale.z = 0.35
        lab.text = name
        ma.markers.append(lab)
        i += 1
    return ma
