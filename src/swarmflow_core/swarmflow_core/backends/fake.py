"""In-memory fake backend (design §6.7) with a deterministic run loop.

Robots follow their routes in straight lines at a constant speed, with no collisions. With a reservation authority the
robots behave like the real agent (§6.5): request a lease at the hold vertex before a zone entry, retry while denied,
heartbeat, and release once ``RELEASE_MARGIN_M`` outside the zone polygon.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

from .. import api
from ..api import (Assignment, Decision, Direction, FleetSnapshot, OrderSpec, OrderStatusChange, ReleaseReason,
                   ReservationRequest, RobotMode, RobotSnapshot, TaskResult)
from ..decisions import reservation_decision
from ..graph import point_in_polygon

_EPS = 1e-9


def _dist_to_polygon(x: float, y: float, poly: Sequence[Tuple[float, float]]) -> float:
    """Distance from a point to the polygon (0 if inside or on the boundary)."""
    if point_in_polygon(x, y, poly):
        return 0.0
    best = math.inf
    for i in range(len(poly)):
        (x1, y1), (x2, y2) = poly[i], poly[(i + 1) % len(poly)]
        dx, dy = x2 - x1, y2 - y1
        L2 = dx * dx + dy * dy
        u = 0.0 if L2 == 0 else max(0.0, min(1.0, ((x - x1) * dx + (y - y1) * dy) / L2))
        best = min(best, math.hypot(x - (x1 + u * dx), y - (y1 + u * dy)))
    return best


@dataclass
class _Robot:
    robot_id: str
    x: float
    y: float
    last_vertex: str
    yaw: float = 0.0
    task: Optional[Assignment] = None
    route: List[str] = field(default_factory=list)
    idx: int = 0                      #: index in route of the last vertex reached
    heading: str = ""                 #: vertex currently moving towards ("" when standing at a vertex)
    returning: bool = False           #: first returning to route[0] (task started mid-lane)
    hook_pending: bool = False
    mode: RobotMode = RobotMode.IDLE
    payload: str = ""
    dwell_end: Optional[float] = None
    dwell_kind: str = ""
    pickup_idx: int = -1
    leases: Dict[str, str] = field(default_factory=dict)       #: zone -> lease id
    entered: Dict[str, bool] = field(default_factory=dict)     #: zone -> has been inside since grant
    next_request_t: float = 0.0
    next_heartbeat_t: float = 0.0
    req_seq: int = 0
    speed: float = 0.0


class FakeBackend:
    def __init__(self, graph, spawn: Dict[str, str], authority=None, speed_mps: float = 0.5):
        self.graph = graph
        self.authority = authority
        self.speed_mps = speed_mps
        self._robots: Dict[str, _Robot] = {}
        for rid, vname in sorted(spawn.items()):
            v = graph.vertices[vname]
            self._robots[rid] = _Robot(rid, v.x, v.y, vname)
        self._t = 0.0
        self._started = False
        self._results: List[TaskResult] = []
        self._orders: Dict[str, OrderSpec] = {}
        self.reservation_log: List[Tuple[float, ReservationRequest, api.ReservationDecision]] = []

    # ---- api.Backend ------------------------------------------------------------------------------------------

    def register_orders(self, orders: Sequence[OrderSpec]) -> None:
        for o in orders:
            self._orders[o.order_id] = o

    def snapshot(self) -> FleetSnapshot:
        return FleetSnapshot(t=self._t, robots=[self._snap(r) for _, r in sorted(self._robots.items())],
                             open_orders=())

    def dispatch(self, assignment: Assignment) -> None:
        r = self._robots[assignment.robot_id]
        route = list(assignment.route)
        r.task, r.route, r.idx = assignment, route, 0
        r.dwell_end, r.dwell_kind = None, ""
        r.payload = ""
        r.pickup_idx = -1
        if assignment.order_id:
            order = self._orders.get(assignment.order_id)
            pickup = order.pickup_vertex if order else next(
                (v for v in route if self.graph.vertices[v].kind == "loading"), "")
            r.pickup_idx = route.index(pickup) if pickup in route else -1
        v0 = self.graph.vertices[route[0]]
        at_v0 = math.hypot(r.x - v0.x, r.y - v0.y) < 1e-6
        r.returning = False
        r.hook_pending = False
        if at_v0:
            r.x, r.y, r.heading, r.hook_pending = v0.x, v0.y, "", True
            r.last_vertex = route[0]
        elif not (len(route) > 1 and route[1] == r.heading):
            r.returning, r.heading = True, route[0]
        # leases the new route will not use and the robot has not entered are given back
        used = {self.graph.zone_of_edge(a, b) for a, b in zip(route, route[1:])} - {None}
        for zone in sorted(r.leases):
            if zone not in used and not r.entered.get(zone, False):
                self._release(r, zone, self._t, ReleaseReason.TASK_CANCELLED)
        r.mode = RobotMode.NAVIGATING

    def cancel(self, task_id: str) -> None:
        for r in self._robots.values():
            if r.task is not None and r.task.task_id == task_id:
                r.task, r.route, r.idx = None, [], 0
                r.dwell_end, r.dwell_kind, r.payload = None, "", ""
                r.returning, r.hook_pending = False, False
                r.mode, r.speed = RobotMode.IDLE, 0.0

    def poll_results(self) -> Sequence[TaskResult]:
        res, self._results = self._results, []
        return res

    def pop_reservation_log(self):
        log, self.reservation_log = self.reservation_log, []
        return log

    # ---- simulation -------------------------------------------------------------------------------------------

    def step(self, t: float) -> None:
        dt = t - self._t if self._started else 0.0
        self._started = True
        self._t = t
        for _, r in sorted(self._robots.items()):
            self._step_robot(r, t, max(0.0, dt))

    def _snap(self, r: _Robot) -> RobotSnapshot:
        remaining = tuple(r.route[r.idx + 1:]) if r.task else ()
        return RobotSnapshot(
            robot_id=r.robot_id, x=r.x, y=r.y, yaw=r.yaw, mode=r.mode, last_vertex=r.last_vertex,
            task_id=r.task.task_id if r.task else "", order_id=r.task.order_id if r.task else "",
            payload_type=r.payload, held_lease_ids=tuple(r.leases[z] for z in sorted(r.leases)),
            linear_speed=r.speed, next_vertex=remaining[0] if remaining else "", remaining_route=remaining)

    def _release(self, r: _Robot, zone: str, t: float, reason: ReleaseReason) -> None:
        lease = r.leases.pop(zone, None)
        r.entered.pop(zone, None)
        if lease is not None and self.authority is not None:
            self.authority.release(r.robot_id, lease, t, reason)

    def _complete(self, r: _Robot, t: float) -> None:
        task = r.task
        for zone in sorted(r.leases):
            self._release(r, zone, t, ReleaseReason.EXITED)
        self._results.append(TaskResult(task.task_id, r.robot_id, True, "", t))
        r.task, r.route, r.idx, r.payload = None, [], 0, ""
        r.mode, r.speed = RobotMode.IDLE, 0.0

    def _zone_need(self, r: _Robot) -> Optional[Tuple[str, str, str, str]]:
        """If the robot is about to enter an unleased zone: (zone, entry, exit, requesting_from_vertex)."""
        route, i = r.route, r.idx
        if i + 1 >= len(route):
            return None
        zone_ab = self.graph.zone_of_edge(route[i], route[i + 1])
        start = i
        if zone_ab is None:
            if i + 2 >= len(route):
                return None
            zone_bc = self.graph.zone_of_edge(route[i + 1], route[i + 2])
            if zone_bc is None:
                return None
            zone, entry, start = zone_bc, route[i + 1], i + 1
        else:
            zone, entry = zone_ab, route[i]
        if zone in r.leases:
            return None
        j = start
        while j + 1 < len(route) and self.graph.zone_of_edge(route[j], route[j + 1]) == zone:
            j += 1
        return zone, entry, route[j], route[i]

    def _step_robot(self, r: _Robot, t: float, dt: float) -> None:
        g = self.graph
        budget = self.speed_mps * dt
        moved = False
        while r.task is not None:
            if r.dwell_end is not None:
                if t + _EPS < r.dwell_end:
                    break
                kind = r.dwell_kind
                r.dwell_end, r.dwell_kind = None, ""
                if kind == "unload":
                    self._complete(r, t)
                    break
                order = self._orders.get(r.task.order_id)
                r.payload = order.payload_type if order else "small"
                r.mode = RobotMode.NAVIGATING
            if r.hook_pending:
                r.hook_pending = False
                r.last_vertex = r.route[r.idx]
                if r.task.order_id and r.idx == r.pickup_idx and r.payload == "":
                    r.dwell_end, r.dwell_kind, r.mode = t + api.LOAD_DWELL_S, "load", RobotMode.LOADING
                    r.speed = 0.0
                    continue
                if r.idx == len(r.route) - 1:
                    if r.task.order_id:
                        r.dwell_end, r.dwell_kind, r.mode = t + api.LOAD_DWELL_S, "unload", RobotMode.UNLOADING
                        r.speed = 0.0
                        continue
                    self._complete(r, t)
                    break
            if r.returning:
                target = r.route[0]
            else:
                if r.idx >= len(r.route) - 1:
                    self._complete(r, t)
                    break
                target = r.route[r.idx + 1]
                if r.heading == "" and self.authority is not None:
                    need = self._zone_need(r)
                    if need is not None:
                        zone, entry, exit_v, _ = need
                        if t + _EPS < r.next_request_t:
                            r.mode, r.speed = RobotMode.WAITING_RESERVATION, 0.0
                            break
                        if not self._request(r, zone, entry, exit_v, t):
                            r.mode, r.speed = RobotMode.WAITING_RESERVATION, 0.0
                            break
            r.mode = RobotMode.NAVIGATING
            tv = g.vertices[target]
            dist = math.hypot(tv.x - r.x, tv.y - r.y)
            r.heading = target
            if dist > _EPS:
                r.yaw = math.atan2(tv.y - r.y, tv.x - r.x)
            step = min(budget, dist)
            if dist > _EPS:
                r.x += (tv.x - r.x) * step / dist
                r.y += (tv.y - r.y) * step / dist
            budget -= step
            moved = moved or step > 0
            if dist - step > 1e-7:
                break
            r.x, r.y, r.heading, r.hook_pending = tv.x, tv.y, "", True
            if r.returning:
                r.returning = False
            else:
                r.idx += 1
            if budget <= 1e-12 and not r.hook_pending:
                break
        r.speed = self.speed_mps if (moved and r.task is not None and r.mode == RobotMode.NAVIGATING) else 0.0
        if self.authority is not None:
            self._maintain_leases(r, t)

    def _request(self, r: _Robot, zone: str, entry: str, exit_v: str, t: float) -> bool:
        z = self.graph.zones[zone]
        entries = [e.entry for e in z.entries]
        direction = Direction.FORWARD if entry == entries[0] else Direction.REVERSE
        i = r.route.index(entry, r.idx)
        j = r.route.index(exit_v, i)
        v = self.speed_mps
        earliest = t + self.graph.path_length(r.route[r.idx:i + 1]) / v
        expected_exit = t + self.graph.path_length(r.route[r.idx:j + 1]) / v
        r.req_seq += 1
        req = ReservationRequest(request_id=f"{r.robot_id}_{zone}_{r.req_seq}", robot_id=r.robot_id, zone_id=zone,
                                 entry_vertex=entry, exit_vertex=exit_v, direction=direction,
                                 earliest_entry_t=earliest, expected_exit_t=expected_exit)
        dec = self.authority.request(req, t)
        self.reservation_log.append((t, req, dec))
        if dec.granted:
            r.leases[zone] = dec.lease_id
            r.entered[zone] = False
            r.next_heartbeat_t = t + api.HEARTBEAT_PERIOD_S
            return True
        r.next_request_t = t + max(dec.retry_after_s, api.RETRY_AFTER_S)
        return False

    def _maintain_leases(self, r: _Robot, t: float) -> None:
        for zone in sorted(r.leases):
            poly = self.graph.zones[zone].polygon
            d = _dist_to_polygon(r.x, r.y, poly)
            if d == 0.0:
                r.entered[zone] = True
            elif r.entered.get(zone, False) and d >= api.RELEASE_MARGIN_M:
                self._release(r, zone, t, ReleaseReason.EXITED)
        if r.leases and t + _EPS >= r.next_heartbeat_t:
            self.authority.heartbeat(r.robot_id, [r.leases[z] for z in sorted(r.leases)], t)
            r.next_heartbeat_t = t + api.HEARTBEAT_PERIOD_S


@dataclass
class FakeRunResult:
    order_ids: List[str]
    decisions: List[Decision]
    max_robots_in_zone: int
    finish_t: float
    status_changes: List[OrderStatusChange] = field(default_factory=list)


_TERMINAL = (api.OrderState.DELIVERED, api.OrderState.FAILED)


def run_fake(core, backend: FakeBackend, orders: Sequence[OrderSpec], until_t: float, dt: float = 0.1) -> FakeRunResult:
    backend.register_orders(orders)
    for o in orders:
        core.add_order(o)
    authority = core.authority
    if authority is not None:
        authority.recover(backend.snapshot().robots, 0.0)
    decisions: List[Decision] = []
    changes: List[OrderStatusChange] = []
    order_ids = [o.order_id for o in orders]
    max_in_zone = 0
    finish_t = until_t
    steps = int(math.floor(until_t / dt + 1e-9))
    for i in range(steps + 1):
        t = round(i * dt, 9)
        backend.step(t)
        robots = backend.snapshot().robots
        counts: Dict[str, int] = {}
        for s in robots:
            core.update_robot(s)
            if authority is not None:
                authority.observe_robot(s.robot_id, s.x, s.y, t)
            zone = backend.graph.zone_at(s.x, s.y)
            if zone is not None:
                counts[zone] = counts.get(zone, 0) + 1
        max_in_zone = max([max_in_zone] + list(counts.values()))
        for (rt, req, dec) in backend.pop_reservation_log():
            decisions.append(reservation_decision(rt, core.policy.name, req, dec))
        for res in backend.poll_results():
            core.task_result(res.task_id, res.success, res.failure_reason, res.t)
        out = core.tick(t)
        for task_id in out.cancels:
            backend.cancel(task_id)
        for a in out.dispatches:
            backend.dispatch(a)
        decisions.extend(out.decisions)
        changes.extend(out.status_changes)
        if all(core.order_state(oid) in _TERMINAL for oid in order_ids):
            finish_t = t
            break
    return FakeRunResult(order_ids, decisions, max_in_zone, finish_t, changes)
