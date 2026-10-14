"""Zone overlay for Foxglove (``/viz/zones``): one polygon per conflict zone, coloured by lease state. Frame ``map``;
time is an argument. (v1 robot/station markers removed in X-02.)"""

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
