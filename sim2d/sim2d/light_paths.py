"""Light mode stand-in for Nav2's planner: expand a goal pose into a path that follows the nav graph (T036).

Pure Python, no ROS. ``graph`` is a ``swarmflow_core.graph.WarehouseGraph`` (only ``vertices``, ``shortest_path`` and
``nearest_vertex`` are used).
"""

from __future__ import annotations

import math
from typing import List, Tuple

Point = Tuple[float, float]

SHORT_HOP_M = 1.0
_ON_SEGMENT_M = 0.35     # a point this close to a path segment counts as already being on it


def _seg(p: Point, a: Point, b: Point) -> Tuple[float, float]:
    """(t, distance) of point p against segment a-b; t is the unclamped projection parameter."""
    dx, dy = b[0] - a[0], b[1] - a[1]
    n2 = dx * dx + dy * dy
    if n2 < 1e-12:
        return 0.0, math.hypot(p[0] - a[0], p[1] - a[1])
    t = ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / n2
    tc = min(1.0, max(0.0, t))
    return t, math.hypot(p[0] - (a[0] + tc * dx), p[1] - (a[1] + tc * dy))


def expand(graph, start_xy: Point, goal_xy: Point) -> List[Point]:
    """Waypoints from ``start_xy`` to ``goal_xy`` (the start itself is not included; the last point is the goal)."""
    goal = (float(goal_xy[0]), float(goal_xy[1]))
    start = (float(start_xy[0]), float(start_xy[1]))
    if math.hypot(goal[0] - start[0], goal[1] - start[1]) <= SHORT_HOP_M:
        return [goal]
    try:
        v0 = graph.nearest_vertex(*start)
        v1 = graph.nearest_vertex(*goal)
        names = list(graph.shortest_path(v0, v1))
    except (ValueError, KeyError):
        return [goal]
    pts = [(graph.vertices[n].x, graph.vertices[n].y) for n in names]
    # skip vertices already behind the start / past the goal
    while len(pts) > 1:
        t, d = _seg(start, pts[0], pts[1])
        if t > 0.0 and d <= _ON_SEGMENT_M:
            pts.pop(0)
        else:
            break
    while len(pts) > 1:
        t, d = _seg(goal, pts[-2], pts[-1])
        if t < 1.0 and d <= _ON_SEGMENT_M:
            pts.pop()
        else:
            break
    if math.hypot(pts[-1][0] - goal[0], pts[-1][1] - goal[1]) < 1e-6:
        pts.pop()
    return pts + [goal]
