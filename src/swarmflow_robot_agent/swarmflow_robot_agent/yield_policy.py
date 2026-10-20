"""Yield to another robot (T067, switch ``SWARMFLOW_YIELD``). Pure Python: no ROS; time is always an argument.

Every robot sees the others on ``/fleet/robot_states`` (pose, heading, speed, mode). When two moving robots are about
to meet head-on or crossing, the one that would reach the meeting point later stops and waits, and says so: it reports
mode ``WAITING_RESERVATION`` with fault reason ``YIELD:<other>`` and zero speed. The other robot then knows this one
will not move: it never yields back to a waiting robot (no mutual yields), and its costmap sees a static robot.

Rules (``should_yield``): never while carrying a pallet (pallets have right of way), never to a robot that is
waiting, stuck or standing still (that is an obstacle for the planner, not an encounter), never to a robot driving the
same way (following). The wait ends (``keep_yielding``) when the other robot no longer closes in on this one, stops,
or after ``YIELD_MAX_S``; then ``YIELD_COOLDOWN_S`` before yielding to the same robot again.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Tuple

YIELD_HORIZON_S = 4.0       #: look this far ahead (constant velocity)
YIELD_MEET_M = 1.0          #: centres closer than this at the closest approach: an encounter (footprints 0.7 m long)
YIELD_RANGE_M = 5.0         #: only robots this close count
YIELD_MIN_ANGLE = math.pi / 4   #: headings differing less than this: following, never an encounter
YIELD_MIN_SPEED = 0.10      #: the other robot must be driving at least this fast
YIELD_MARGIN_S = 0.3        #: arrival times closer than this: the higher robot id yields
YIELD_MAX_S = 6.0           #: wait at most this long
YIELD_COOLDOWN_S = 10.0     #: then do not yield to the same robot for this long
YIELD_CLEAR_M = 1.5         #: the wait ends once the other robot's closest approach is beyond this


@dataclass(frozen=True)
class Body:
    robot_id: str
    x: float
    y: float
    yaw: float
    speed: float            #: m/s along the heading (negative = reversing)
    mode: str = "NAVIGATING"

    def velocity(self) -> Tuple[float, float]:
        return self.speed * math.cos(self.yaw), self.speed * math.sin(self.yaw)


def _wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def closest_approach(a: Body, b: Body, horizon: float = YIELD_HORIZON_S) -> Tuple[float, float, bool]:
    """(distance, time, closing) of the closest approach of ``a`` and ``b`` within ``horizon``, constant velocity."""
    px, py = b.x - a.x, b.y - a.y
    (avx, avy), (bvx, bvy) = a.velocity(), b.velocity()
    vx, vy = bvx - avx, bvy - avy
    vv = vx * vx + vy * vy
    closing = px * vx + py * vy < 0.0
    t = 0.0 if vv < 1e-9 else max(0.0, min(horizon, -(px * vx + py * vy) / vv))
    return math.hypot(px + vx * t, py + vy * t), t, closing


def _arrival(body: Body, mx: float, my: float) -> float:
    return math.hypot(mx - body.x, my - body.y) / max(abs(body.speed), YIELD_MIN_SPEED)


def should_yield(me: Body, other: Body, me_has_pallet: bool = False) -> Optional[str]:
    """The reason to yield to ``other`` now, or None."""
    if me_has_pallet or other.mode in ("WAITING_RESERVATION", "STUCK", "FAULT", "LIFTING", "UNLOADING", "LOADING"):
        return None
    if abs(other.speed) < YIELD_MIN_SPEED or math.hypot(other.x - me.x, other.y - me.y) > YIELD_RANGE_M:
        return None
    if abs(_wrap(other.yaw - me.yaw)) < YIELD_MIN_ANGLE:
        return None                                              # same direction: following, not an encounter
    d, t, closing = closest_approach(me, other)
    if not closing or d >= YIELD_MEET_M:
        return None
    if other.mode == "CARRYING":
        return "pallet"                                          # a pallet carrier has right of way
    (ax, ay), (bx, by) = me.velocity(), other.velocity()
    mx = (me.x + ax * t + other.x + bx * t) / 2.0                # meeting point
    my = (me.y + ay * t + other.y + by * t) / 2.0
    mine, theirs = _arrival(me, mx, my), _arrival(other, mx, my)
    if mine > theirs + YIELD_MARGIN_S:
        return "later"
    if abs(mine - theirs) <= YIELD_MARGIN_S and me.robot_id > other.robot_id:
        return "tie"
    return None


def keep_yielding(me: Body, other: Optional[Body], since: float, t: float) -> bool:
    """While yielding (``me`` standing still): wait on while ``other`` still drives towards this robot."""
    if other is None or t - since >= YIELD_MAX_S:
        return False
    if other.mode in ("WAITING_RESERVATION", "STUCK", "FAULT") or abs(other.speed) < YIELD_MIN_SPEED:
        return False
    still = Body(me.robot_id, me.x, me.y, me.yaw, 0.0, me.mode)
    d, _, closing = closest_approach(still, other)
    return closing and d < YIELD_CLEAR_M
