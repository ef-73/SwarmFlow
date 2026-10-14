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
                 "detour", "hold_for", "hold_t", "wait_id", "detour_t", "attempts", "settle", "crab", "tabu")

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
        self.detour_t = 0.0                        # time spent on the current detour
        self.attempts = 0                          # detours tried for the current goals (alternates the side)
        self.tabu: List[Tuple[float, float]] = []  # recent detour targets (not revisited if there is an alternative)
        self.crab = False                          # detour turn is jammed by a neighbour: slide to the detour point
        self.settle = False                        # goal is occupied by a parked robot: accept the position


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
                 hard_margin: float = 0.01, crab_speed: float = 0.3, standoff_s: float = 0.5, sidestep_m: float = 0.85):
        self.max_speed, self.max_turn = float(max_speed), float(max_turn)
        self.hl, self.hw = footprint[0] / 2.0, footprint[1] / 2.0
        self.stop_dist, self.accel = float(stop_dist), float(accel)
        self.wp_radius, self.soft_margin, self.hard_margin = float(wp_radius), float(soft_margin), float(hard_margin)
        self.crab_speed = float(crab_speed)
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
        r.crab = False
        r.blocker, r.blocked_t = None, 0.0
        r.attempts, r.settle, r.tabu = 0, False, []
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
                    and o.hold_for is None and (o.goals or o.final_yaw is not None)):
                continue                         # crossing: the lower id has right of way (hard layer still guards)
            m = self.soft_margin + (0.15 if o.detour is not None else 0.0)    # give a side-stepping robot room
            for f in (0.5, 1.0):
                d = reach * f
                if _overlap(r.x + c * d, r.y + s * d, r.yaw, o.x, o.y, o.yaw, self.hl, self.hw, m):
                    return o
        return None

    def _hard_ok(self, r: _Robot, nx: float, ny: float, nyaw: float) -> bool:
        return self._hard_culprit(r, nx, ny, nyaw) is None

    def _hard_culprit(self, r: _Robot, nx: float, ny: float, nyaw: float) -> Optional[_Robot]:
        for o in self._near(r):
            if _overlap(nx, ny, nyaw, o.x, o.y, o.yaw, self.hl, self.hw, self.hard_margin) and not _overlap(
                    r.x, r.y, r.yaw, o.x, o.y, o.yaw, self.hl, self.hw, 0.0):
                return o
        return None

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

        # velocity update with acceleration limits (braking is stronger than accelerating)
        r.v = min(v_t, r.v + a * dt) if v_t > r.v else max(v_t, r.v - 1.5 * a * dt)
        r.w = w_t

        nyaw = _wrap(r.yaw + r.w * dt)
        mid = r.yaw + 0.5 * r.w * dt
        nx, ny = r.x + r.v * dt * math.cos(mid), r.y + r.v * dt * math.sin(mid)
        reached = False
        if r.crab and r.detour is not None and not holding:
            tx, ty = r.detour
            d = math.hypot(tx - r.x, ty - r.y)
            step = self.crab_speed * dt
            nyaw, r.w, r.v = r.yaw, 0.0, self.crab_speed
            if d <= step:
                nx, ny, reached = tx, ty, True
            else:
                nx, ny = r.x + (tx - r.x) / d * step, r.y + (ty - r.y) / d * step
        elif snap is not None:
            sd = math.hypot(snap[0] - r.x, snap[1] - r.y)
            if sd <= 0.005 or r.v * dt >= sd:
                nx, ny, reached = snap[0], snap[1], True
        hard = False
        culprit: Optional[_Robot] = None
        if nx != r.x or ny != r.y or nyaw != r.yaw:
            culprit = self._hard_culprit(r, nx, ny, nyaw)
            if culprit is not None:
                hard = True
                reached = False
                r.v = 0.0
                if r.detour is not None and not r.crab and v_t == 0.0 and w_t != 0.0:
                    r.crab = True
                nx, ny = r.x, r.y
                if not self._hard_ok(r, nx, ny, nyaw):
                    nyaw, r.w = r.yaw, 0.0
        r.x, r.y, r.yaw = nx, ny, nyaw

        done = False
        if reached:
            r.v = 0.0
            if r.detour is not None:
                r.detour, r.crab = None, False
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

        self._blocked_bookkeeping(r, soft if soft is not None else culprit, dt)
        if r.detour is not None:
            r.detour_t += dt
            if r.detour_t > 12.0 or (r.detour_t > 6.0 and soft is not None):    # jammed: drop it, retry later
                r.detour, r.detour_t, r.crab = None, 0.0, False
                r.hold_for = None
                r.blocker, r.blocked_t = None, 0.0
        if r.settle:                              # the goal lies under a parked robot: stay where we are
            r.settle, r.goals, r.final_yaw, r.detour, r.hold_for = False, [], None, None, None
            r.v = r.w = 0.0
            r.state = "arrived"
            return

        if done:
            r.state = "arrived"
        elif holding or soft is not None or hard:
            r.state = "blocked"
        elif r.goals or r.detour is not None or r.final_yaw is not None or r.hold_for is not None:
            r.state = "moving"
        elif r.state != "arrived":
            r.state = "idle"

    def _blocked_bookkeeping(self, r: _Robot, soft: Optional[_Robot], dt: float) -> None:
        """Deadlock breaker. Rules, in order (all deterministic):

        * blocked by a *parked* robot (idle / arrived, it cannot move): after 2 s the blocked robot goes around it
          (see :meth:`_start_detour`); if its final goal is within 1 m of the parked robot it settles where it is after
          1.5 s (reports arrived).
        * blocked by a *busy* robot that is stopped or blocked in turn: after 4 s the higher id of the two yields with
          a side-step and waits for the other to pass. Head-on pairs yield after ``standoff_s``.
        * hard-layer contact counts as being blocked by the robot touched (the soft layer may have exempted it).
        """
        if soft is None:
            r.blocker, r.blocked_t = None, 0.0
            return
        if r.blocker == soft.rid:
            r.blocked_t += dt
        else:
            r.blocker, r.blocked_t = soft.rid, dt
        busy = bool(soft.goals) or soft.detour is not None or soft.hold_for is not None or soft.final_yaw is not None
        if not busy and r.goals and len(r.goals) == 1 and r.blocked_t >= 1.5:
            if math.hypot(r.goals[0][0] - soft.x, r.goals[0][1] - soft.y) < 1.0:
                r.settle = True
                return
        if r.detour is not None or r.hold_for is not None:
            return
        head_on = math.cos(soft.yaw - r.yaw) < -0.7
        if not busy:
            if r.goals and r.blocked_t >= 2.0:
                self._start_detour(r, soft, None)
            return
        trigger = (head_on and soft.blocker == r.rid and r.blocked_t >= self.standoff_s) or (
            r.blocked_t >= 4.0 and (soft.blocker is not None or soft.v < 0.02))
        if trigger and r.rid > soft.rid:
            self._start_detour(r, soft, soft.rid)

    def _detour_free(self, r: _Robot, tx: float, ty: float, fx: Optional[float] = None,
                     fy: Optional[float] = None, parked_only: bool = False) -> bool:
        """True if driving straight from (fx, fy) (default: ``r``) to (tx, ty) keeps clear of every other robot."""
        fx, fy = (r.x, r.y) if fx is None or fy is None else (fx, fy)
        d = math.hypot(tx - fx, ty - fy)
        yaw = math.atan2(ty - fy, tx - fx)
        n = max(1, int(d / 0.2))
        for k in range(1, n + 1):
            px, py = fx + (tx - fx) * k / n, fy + (ty - fy) * k / n
            for oid in self._order:
                if oid != r.rid:
                    o = self._r[oid]
                    if parked_only and (o.goals or o.detour is not None or o.final_yaw is not None):
                        continue
                    if _overlap(px, py, yaw, o.x, o.y, o.yaw, self.hl, self.hw, 0.1):
                        return False
        return True

    def _start_detour(self, r: _Robot, blocker: _Robot, wait_id: Optional[str]) -> None:
        """Side-step (or back off) away from ``blocker`` to a spot reachable without touching anyone else.

        Candidates are tried in a fixed order; the one that ends nearest the goal (or, with no goal, nearest the
        current spot) wins. Around a parked robot the candidates also move forward past it. No free candidate: stay
        put and look again after ``retry_s`` (the jam is then cleared by whoever can move).
        """
        r.blocked_t = 0.0
        hx, hy = math.cos(r.yaw), math.sin(r.yaw)
        goal = None
        if r.goals:
            goal = r.goals[-1][:2]
            gx, gy, _ = r.goals[0]
            n = math.hypot(gx - r.x, gy - r.y)
            if wait_id is None and n > 1e-6:
                hx, hy = (gx - r.x) / n, (gy - r.y) / n
        db = math.hypot(blocker.x - r.x, blocker.y - r.y)
        fwds = (db + 0.7, 0.0, -0.8) if wait_id is None else (0.0, -0.8)
        best, best_score = None, math.inf
        away = math.atan2(-(blocker.y - r.y), -(blocker.x - r.x))
        cands = []
        for side in ((-1.0, 1.0) if (hx * (blocker.y - r.y) - hy * (blocker.x - r.x)) > 0.0 else (1.0, -1.0)):
            for off in (self.sidestep_m, 1.3, 1.8, 2.4, 3.0):
                for fwd in fwds:
                    cands.append((r.x + hx * fwd - hy * side * off, r.y + hy * fwd + hx * side * off))
        if wait_id is None:                           # also a ring of spots all around, for a pocket with one exit
            for dist in (1.2, 2.0, 3.0):
                for k in range(12):
                    ang = away + math.pi * (k + 0.5) / 6.0
                    cands.append((r.x + dist * math.cos(ang), r.y + dist * math.sin(ang)))
        for tx, ty in cands:
            if not self._detour_free(r, tx, ty):
                continue
            score = math.hypot(goal[0] - tx, goal[1] - ty) if goal else 0.0
            if goal and not self._detour_free(r, goal[0], goal[1], tx, ty, parked_only=True):
                score += 50.0                    # still walled in from there: prefer a spot with a clear way on
            if any(math.hypot(tx - px, ty - py) < 0.8 for px, py in r.tabu):
                score += 20.0                    # been there recently
            if score < best_score - 1e-9:
                best, best_score = (tx, ty), score
        r.attempts += 1
        if best is None:
            return
        r.detour = best
        r.tabu = (r.tabu + [best])[-8:]
        r.detour_t, r.crab = 0.0, False
        r.wait_id = wait_id
