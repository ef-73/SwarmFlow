"""Light mode kinematic world (T025): unicycle robots with simple local avoidance. Pure Python, no ROS, no clock.

Avoidance has two layers:

* soft: a robot slows to a stop when another footprint lies in the area swept ahead of it within ``stop_dist``
  (state ``blocked``). A lower id ignores a higher id robot that is crossing (not head-on) so a symmetric crossing
  cannot deadlock.
* hard: robots are updated in id order and a robot never moves into an overlap with another footprint that its
  current pose does not already overlap. This is what guarantees zero clearance violations.

A head-on standoff is broken deterministically: after ``standoff_s`` of mutual blocking the higher id robot
side-steps ``sidestep_m`` to its right, waits until the other robot has passed, then continues.
"""

from __future__ import annotations

import math
from typing import Dict, List, Optional, Sequence, Tuple

Goal = Tuple[float, float, Optional[float]]

_STATES = ("idle", "moving", "blocked", "arrived")


def _wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def _clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else hi if v > hi else v


class _Robot:
    __slots__ = ("rid", "x", "y", "yaw", "v", "w", "goals", "state", "final_yaw", "blocker", "blocked_t",
                 "detour", "hold_for", "hold_t", "wait_id")

    def __init__(self, rid: str, x: float, y: float, yaw: float):
        self.rid, self.x, self.y, self.yaw = rid, float(x), float(y), float(yaw)
        self.v = 0.0
        self.w = 0.0
        self.goals: List[Goal] = []
        self.state = "idle"
        self.final_yaw: Optional[float] = None     # pending in-place turn after the last position is reached
        self.blocker: Optional[str] = None         # robot soft-blocking this one in the latest step
        self.blocked_t = 0.0                       # contiguous soft-blocked time by the same blocker
        self.detour: Optional[Tuple[float, float]] = None
        self.hold_for: Optional[str] = None        # waiting at the side for this robot to pass
        self.hold_t = 0.0
        self.wait_id: Optional[str] = None


def _overlap(ax: float, ay: float, ayaw: float, bx: float, by: float, byaw: float,
             hl: float, hw: float, margin: float) -> bool:
    """True if box A (half extents inflated by ``margin``) intersects box B (separating axis test)."""
    ca, sa, cb, sb = math.cos(ayaw), math.sin(ayaw), math.cos(byaw), math.sin(byaw)
    dx, dy = bx - ax, by - ay
    la, wa = hl + margin, hw + margin
    for ux, uy in ((ca, sa), (-sa, ca), (cb, sb), (-sb, cb)):
        ra = la * abs(ux * ca + uy * sa) + wa * abs(-ux * sa + uy * ca)
        rb = hl * abs(ux * cb + uy * sb) + hw * abs(-ux * sb + uy * cb)
        if abs(dx * ux + dy * uy) > ra + rb:
            return False
    return True


class LightWorld:
    def __init__(self, max_speed: float, max_turn: float, footprint: Tuple[float, float] = (0.70, 0.60),
                 stop_dist: float = 0.6, accel: float = 1.0, wp_radius: float = 0.12, soft_margin: float = 0.05,
                 hard_margin: float = 0.01, standoff_s: float = 0.5, sidestep_m: float = 0.85):
        self.max_speed, self.max_turn = float(max_speed), float(max_turn)
        self.hl, self.hw = footprint[0] / 2.0, footprint[1] / 2.0
        self.stop_dist, self.accel = float(stop_dist), float(accel)
        self.wp_radius, self.soft_margin, self.hard_margin = float(wp_radius), float(soft_margin), float(hard_margin)
        self.standoff_s, self.sidestep_m = float(standoff_s), float(sidestep_m)
        self._r: Dict[str, _Robot] = {}
        self._order: List[str] = []
        self._reach = 2.0 * math.hypot(self.hl, self.hw) + self.stop_dist + 2.0 * self.hl

    # -- public API ----------------------------------------------------------------------------------------------
    def add_robot(self, rid: str, x: float, y: float, yaw: float) -> None:
        self._r[rid] = _Robot(rid, x, y, yaw)
        self._order = sorted(self._r)

    def robot_ids(self) -> List[str]:
        return list(self._order)

    def set_goals(self, rid: str, goals: Sequence[Goal]) -> None:
        r = self._r[rid]
        r.goals = [(float(x), float(y), None if yaw is None else float(yaw)) for x, y, yaw in goals]
        r.final_yaw = None
        r.detour = r.hold_for = None
        r.blocker, r.blocked_t = None, 0.0
        r.state = "moving" if r.goals else "idle"

    def cancel(self, rid: str) -> None:
        r = self._r[rid]
        r.goals, r.final_yaw, r.detour, r.hold_for = [], None, None, None
        r.v = r.w = 0.0
        r.blocker, r.blocked_t = None, 0.0
        r.state = "idle"

    def pose(self, rid: str) -> Tuple[float, float, float]:
        r = self._r[rid]
        return r.x, r.y, r.yaw

    def speed(self, rid: str) -> float:
        return self._r[rid].v

    def state(self, rid: str) -> str:
        return self._r[rid].state

    def remaining_goals(self, rid: str) -> List[Goal]:
        r = self._r[rid]
        return list(r.goals)

    def distance_remaining(self, rid: str) -> float:
        r = self._r[rid]
        d, px, py = 0.0, r.x, r.y
        for gx, gy, _ in r.goals:
            d += math.hypot(gx - px, gy - py)
            px, py = gx, gy
        return d

    # -- stepping ------------------------------------------------------------------------------------------------
    def step(self, dt: float) -> None:
        for rid in self._order:
            self._step_robot(self._r[rid], dt)

    def _near(self, r: _Robot):
        reach2 = self._reach * self._reach
        for oid in self._order:
            if oid == r.rid:
                continue
            o = self._r[oid]
            if (o.x - r.x) ** 2 + (o.y - r.y) ** 2 <= reach2:
                yield o

    def _soft_blocker(self, r: _Robot, path_left: float) -> Optional[_Robot]:
        c, s = math.cos(r.yaw), math.sin(r.yaw)
        reach = min(self.stop_dist, path_left)         # nothing beyond the end of the path matters
        for o in self._near(r):
            if (r.rid < o.rid and abs(math.cos(o.yaw - r.yaw)) < 0.7 and o.detour is None
                    and o.hold_for is None):
                continue                         # crossing: the lower id has right of way (hard layer still guards)
            m = self.soft_margin + (0.15 if o.detour is not None else 0.0)    # give a side-stepping robot room
            for f in (0.5, 1.0):
                d = reach * f
                if _overlap(r.x + c * d, r.y + s * d, r.yaw, o.x, o.y, o.yaw, self.hl, self.hw, m):
                    return o
        return None

    def _hard_ok(self, r: _Robot, nx: float, ny: float, nyaw: float) -> bool:
        for o in self._near(r):
            if _overlap(nx, ny, nyaw, o.x, o.y, o.yaw, self.hl, self.hw, self.hard_margin) and not _overlap(
                    r.x, r.y, r.yaw, o.x, o.y, o.yaw, self.hl, self.hw, 0.0):
                return False
        return True

    def _target(self, r: _Robot):
        """(tx, ty, is_final) of the current target or None."""
        if r.detour is not None:
            return r.detour[0], r.detour[1], True
        if r.goals:
            g = r.goals[0]
            return g[0], g[1], len(r.goals) == 1
        return None

    def _step_robot(self, r: _Robot, dt: float) -> None:
        a = self.accel
        v_t = w_t = 0.0
        soft: Optional[_Robot] = None
        snap: Optional[Tuple[float, float]] = None     # final target to snap onto when the step reaches it
        holding = False

        if r.hold_for is not None:               # parked beside the lane until the other robot has passed
            r.hold_t += dt
            o = self._r.get(r.hold_for)
            gone = o is None or r.hold_t > 30.0
            if not gone and r.goals:
                gx, gy, _ = r.goals[0]
                n = math.hypot(gx - r.x, gy - r.y) or 1.0
                along = ((o.x - r.x) * (gx - r.x) + (o.y - r.y) * (gy - r.y)) / n
                gone = along < -0.5 or math.hypot(o.x - r.x, o.y - r.y) > 3.0
            if gone:
                r.hold_for, r.hold_t = None, 0.0
            else:
                holding = True
        if holding:
            pass
        elif r.final_yaw is not None:
            err = _wrap(r.final_yaw - r.yaw)
            w_t = _clamp(err / dt, -self.max_turn, self.max_turn)
        elif r.detour is not None or r.goals:
            if r.detour is not None:
                tx, ty = r.detour
                final = True
            else:
                tx, ty, _ = r.goals[0]
                final = len(r.goals) == 1
                if not final and math.hypot(tx - r.x, ty - r.y) <= self.wp_radius:
                    r.goals.pop(0)
                    tx, ty, _ = r.goals[0]
                    final = len(r.goals) == 1
            dx, dy = tx - r.x, ty - r.y
            dist = math.hypot(dx, dy)
            err = _wrap(math.atan2(dy, dx) - r.yaw) if dist > 1e-9 else 0.0
            w_t = _clamp(3.0 * err, -self.max_turn, self.max_turn)
            if abs(err) <= 0.5:
                v_t = self.max_speed * math.cos(err)
                if final:
                    v_t = min(v_t, math.sqrt(2.0 * 0.8 * a * dist))
                    snap = (tx, ty)
            if v_t > 0.0:
                soft = self._soft_blocker(r, dist if r.detour is not None else self.distance_remaining(r.rid))
                if soft is not None:
                    v_t = 0.0

        self._blocked_bookkeeping(r, soft, dt)

        # velocity update with acceleration limits (braking is stronger than accelerating)
        r.v = min(v_t, r.v + a * dt) if v_t > r.v else max(v_t, r.v - 1.5 * a * dt)
        r.w = w_t

        nyaw = _wrap(r.yaw + r.w * dt)
        mid = r.yaw + 0.5 * r.w * dt
        nx, ny = r.x + r.v * dt * math.cos(mid), r.y + r.v * dt * math.sin(mid)
        reached = False
        if snap is not None:
            sd = math.hypot(snap[0] - r.x, snap[1] - r.y)
            if sd <= 0.005 or r.v * dt >= sd:
                nx, ny, reached = snap[0], snap[1], True
        hard = False
        if nx != r.x or ny != r.y or nyaw != r.yaw:
            if not self._hard_ok(r, nx, ny, nyaw):
                hard = True
                reached = False
                r.v = 0.0
                nx, ny = r.x, r.y
                if not self._hard_ok(r, nx, ny, nyaw):
                    nyaw, r.w = r.yaw, 0.0
        r.x, r.y, r.yaw = nx, ny, nyaw

        done = False
        if reached:
            r.v = 0.0
            if r.detour is not None:
                r.detour = None
                r.hold_for, r.hold_t = r.wait_id, 0.0
            else:
                g = r.goals.pop(0)
                if g[2] is not None:
                    r.final_yaw = g[2]
                else:
                    done = not r.goals
        elif r.final_yaw is not None and not hard and abs(_wrap(r.final_yaw - r.yaw)) < 1e-6:
            r.yaw, r.final_yaw, r.w = r.final_yaw, None, 0.0
            done = not r.goals

        if done:
            r.state = "arrived"
        elif holding or soft is not None or hard:
            r.state = "blocked"
        elif r.goals or r.detour is not None or r.final_yaw is not None or r.hold_for is not None:
            r.state = "moving"
        elif r.state != "arrived":
            r.state = "idle"

    def _blocked_bookkeeping(self, r: _Robot, soft: Optional[_Robot], dt: float) -> None:
        if soft is None:
            r.blocker, r.blocked_t = None, 0.0
            return
        if r.blocker == soft.rid:
            r.blocked_t += dt
        else:
            r.blocker, r.blocked_t = soft.rid, dt
        busy = bool(soft.goals) or soft.detour is not None
        head_on = math.cos(soft.yaw - r.yaw) < -0.7
        trigger = (head_on and soft.blocker == r.rid and r.blocked_t >= self.standoff_s) or (
            busy and r.blocked_t >= 8.0)
        if trigger and r.rid > soft.rid and r.detour is None and r.hold_for is None:
            hx, hy = math.cos(r.yaw), math.sin(r.yaw)
            r.detour = (r.x + hy * self.sidestep_m, r.y - hx * self.sidestep_m)     # to the right of the heading
            r.wait_id = soft.rid
            r.blocked_t = 0.0
