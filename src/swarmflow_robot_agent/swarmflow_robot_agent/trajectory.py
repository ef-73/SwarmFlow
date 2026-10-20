"""Reserved trajectories (T068): where each robot will be, and when. Pure Python: no ROS; time is always an argument.

Every robot publishes its reservation: its Nav2 plan timed at a nominal speed (one point every ``DT_S`` up to
``HORIZON_S``), stamped with the time it started its current goal (the reservation time). A robot yields only to a
robot whose reservation meets its own at a similar time (``first_conflict``): robots whose paths cross at different
times, or that are far apart, do not conflict.

Priority is a total order (``priority_key``): pallet carriers first, then the older reservation, then the lower
robot id. A robot only ever waits for a robot ranked above it, so "waits for" can never form a cycle, whatever the
number of robots. Cooperative A* / WHCA* (Silver 2005) reserves space-time the same way; as there, it is fast but not
complete, and the deadlock breaker (deadlock.py) is the backstop.

The waiting robot waits *out of the way* (``wait_index``): at the last point of its own path, before the conflict,
that keeps ``CLEAR_M`` from the other robot's whole reservation and is not inside a dead-end storage aisle. If there
is no such point it does not stop at all: standing in the other robot's path is what jammed the stop-in-place yield
(T067, run y167_1).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Callable, Optional, Sequence, Tuple

DT_S = 0.5              #: trajectory sample spacing
HORIZON_S = 15.0        #: reservations reach this far ahead
NOMINAL_MPS = 0.5       #: planned speed used to time a plan (the robots cruise at 0.5-0.6 m/s)
MEET_M = 1.0            #: two robots closer than this (centres) meet: footprints 0.7 x 0.6 plus a margin
TIME_TOL_S = 1.5        #: "at a similar time": |t_mine - t_theirs| within this ...
TIME_TOL_GROWTH = 0.15  #: ... plus this fraction of how far ahead the meeting is (timing grows less certain)
YIELD_WITHIN_S = 12.0   #: only conflicts this soon count
CLEAR_M = 1.3           #: a wait point keeps this far from every point of the other robot's reservation

Point = Tuple[float, float, float]      #: (t, x, y), t absolute (sim seconds)


@dataclass(frozen=True)
class Reservation:
    robot_id: str
    reserved_t: float                    #: when the robot started its current goal
    pallet: bool                         #: carrying a pallet: right of way
    points: Tuple[Point, ...]            #: timed path; one point = standing there


def priority_key(r: Reservation) -> Tuple[int, float, str]:
    """Smaller = higher priority."""
    return (0 if r.pallet else 1, r.reserved_t, r.robot_id)


def outranks(a: Reservation, b: Reservation) -> bool:
    return priority_key(a) < priority_key(b)


def time_parameterise(path: Sequence[Tuple[float, float]], x: float, y: float, t_now: float,
                      speed: float = NOMINAL_MPS, dt: float = DT_S, horizon: float = HORIZON_S) -> Tuple[Point, ...]:
    """The plan ``path`` (map x, y) from the robot's position, one point every ``dt`` at ``speed``, up to
    ``horizon`` or the end of the path. Starts at the robot (t_now, x, y). An empty path: standing here."""
    if not path:
        return ((t_now, x, y),)
    # continue from the path point nearest the robot (a Nav2 plan starts at the robot, an older one behind it)
    k = min(range(len(path)), key=lambda i: math.hypot(path[i][0] - x, path[i][1] - y))
    pts = [(x, y)] + [(float(a), float(b)) for a, b in path[k:]]
    cum = [0.0]
    for (ax, ay), (bx, by) in zip(pts, pts[1:]):
        cum.append(cum[-1] + math.hypot(bx - ax, by - ay))
    total = min(cum[-1], speed * horizon)
    out = []
    seg = 0
    n = 0
    while True:
        s = min(n * speed * dt, total)
        while seg + 1 < len(cum) - 1 and cum[seg + 1] < s:
            seg += 1
        a, b = pts[seg], pts[min(seg + 1, len(pts) - 1)]
        span = cum[min(seg + 1, len(cum) - 1)] - cum[seg]
        f = 0.0 if span <= 1e-9 else (s - cum[seg]) / span
        out.append((t_now + s / speed, a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f))
        if s >= total:
            return tuple(out)
        n += 1


def first_conflict(mine: Sequence[Point], theirs: Sequence[Point], t_now: float, meet: float = MEET_M,
                   within: float = YIELD_WITHIN_S) -> Optional[int]:
    """Index of my first point (within ``within`` s) that meets a point of ``theirs`` at a similar time, or None.
    A one-point reservation (a robot standing) is there at every time."""
    standing = len(theirs) == 1
    for i, (ti, xi, yi) in enumerate(mine):
        if ti - t_now > within:
            break
        for tj, xj, yj in theirs:
            if math.hypot(xi - xj, yi - yj) >= meet:
                continue
            if standing:
                return i
            ahead = max(0.0, min(ti, tj) - t_now)
            if abs(ti - tj) <= TIME_TOL_S + TIME_TOL_GROWTH * ahead:
                return i
    return None


def wait_index(mine: Sequence[Point], conflict: int, theirs: Sequence[Point],
               in_aisle: Callable[[float, float], bool] = lambda x, y: False, clear: float = CLEAR_M) -> Optional[int]:
    """The last index of ``mine`` before ``conflict`` that keeps ``clear`` from every point of ``theirs`` and is not in
    a dead-end aisle, or None (then the robot must not stop: it would stand in the other robot's way)."""
    for i in range(conflict - 1, -1, -1):
        _, x, y = mine[i]
        if in_aisle(x, y):
            continue
        if all(math.hypot(x - xj, y - yj) >= clear for _, xj, yj in theirs):
            return i
    return None


def corridor_clear(corridor: Sequence[Tuple[float, float]], theirs: Sequence[Point], clear: float = MEET_M) -> bool:
    """True when no point of ``theirs`` (its position now and its reservation) comes within ``clear`` of the
    corridor (the path the waiting robot will take next): the other robot has passed or no longer heads there."""
    return all(math.hypot(cx - xj, cy - yj) >= clear for cx, cy in corridor for _, xj, yj in theirs)
