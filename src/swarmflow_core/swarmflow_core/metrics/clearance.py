"""Clearance metric (design §13.1, C20): padded-footprint distance and violation episodes. Pure stdlib."""

from __future__ import annotations

import math
from typing import Dict, List, Mapping, Tuple

Pose = Tuple[float, float, float]
Point = Tuple[float, float]

FOOTPRINT_LENGTH_M = 0.70
FOOTPRINT_WIDTH_M = 0.60


def _corners(p: Pose) -> List[Point]:
    x, y, yaw = p
    c, s = math.cos(yaw), math.sin(yaw)
    hl, hw = FOOTPRINT_LENGTH_M / 2.0, FOOTPRINT_WIDTH_M / 2.0
    return [(x + c * dx - s * dy, y + s * dx + c * dy) for dx, dy in ((hl, hw), (-hl, hw), (-hl, -hw), (hl, -hw))]


def _axes(p: Pose) -> List[Point]:
    yaw = p[2]
    return [(math.cos(yaw), math.sin(yaw)), (-math.sin(yaw), math.cos(yaw))]


def _project(corners: List[Point], axis: Point) -> Tuple[float, float]:
    vals = [cx * axis[0] + cy * axis[1] for cx, cy in corners]
    return min(vals), max(vals)


def _point_segment(p: Point, a: Point, b: Point) -> float:
    abx, aby = b[0] - a[0], b[1] - a[1]
    denom = abx * abx + aby * aby
    u = 0.0 if denom == 0.0 else max(0.0, min(1.0, ((p[0] - a[0]) * abx + (p[1] - a[1]) * aby) / denom))
    return math.hypot(p[0] - (a[0] + u * abx), p[1] - (a[1] + u * aby))


def footprint_distance(pose_a: Pose, pose_b: Pose) -> float:
    """Minimum distance between the two padded footprints; negative (minus penetration depth) when overlapping."""
    ca, cb = _corners(pose_a), _corners(pose_b)
    min_overlap = math.inf
    for axis in _axes(pose_a) + _axes(pose_b):
        a_lo, a_hi = _project(ca, axis)
        b_lo, b_hi = _project(cb, axis)
        overlap = min(a_hi, b_hi) - max(a_lo, b_lo)
        if overlap <= 0.0:
            min_overlap = -1.0           # separated: compute the true distance below
            break
        min_overlap = min(min_overlap, overlap)
    if min_overlap > 0.0:
        return -min_overlap
    best = math.inf
    for pts, other in ((ca, cb), (cb, ca)):
        for i in range(4):
            a, b = other[i], other[(i + 1) % 4]
            for p in pts:
                best = min(best, _point_segment(p, a, b))
    return best


def count_violations(samples: Mapping[float, Mapping[str, Pose]]) -> int:
    """Number of overlap episodes: per robot pair, contiguous samples with distance < 0 count once."""
    active: Dict[Tuple[str, str], bool] = {}
    episodes = 0
    for t in sorted(samples):
        poses = samples[t]
        ids = sorted(poses)
        for i, ra in enumerate(ids):
            for rb in ids[i + 1:]:
                violating = footprint_distance(poses[ra], poses[rb]) < 0.0
                if violating and not active.get((ra, rb), False):
                    episodes += 1
                active[(ra, rb)] = violating
    return episodes
