"""Robot agent state machine (design §6.3, §6.5 rules 3-6, §13.5) — pure Python, no ROS imports.

All event methods take the current sim time ``t`` (seconds) and return a list of *actions* (the dataclasses below) for
the caller (ROS node or 2D sim) to execute. Nothing in here sleeps, reads a clock or does I/O.

M9 review hardening: S1 - ``on_lease_event`` drops a lease the authority no longer honours (expired, revoked,
released, occupied-unknown) and, if the robot has not yet entered the zone, cancels navigation and re-requests, so a
granted robot never drives into a zone it no longer holds. S4 - pose validity: before the first pose the state is
``FAULT/NO_POSE`` (cannot accept tasks); a pose older than ``POSE_STALE_S`` shows ``FAULT/STALE_POSE`` (overlay only).

Route bookkeeping uses **indices into the route**, not vertex names, because a route may visit a vertex twice
(e.g. ``X_1_0`` before and after the pickup).
"""

from __future__ import annotations

import math
import random
from collections import deque
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Set, Tuple

from swarmflow_core import api
from swarmflow_core.api import ReleaseReason, RobotMode
from swarmflow_core.graph import WarehouseGraph, point_in_polygon

ZONE_SPEED_MPS = 0.5          #: assumed speed through a zone when estimating ``expected_exit_t``
POSE_STALE_S = 1.0            #: a pose older than this makes ``state()`` report FAULT / STALE_POSE
REACH_RADIUS_M = 0.6          #: pose within this of a route vertex counts as having reached it


# ---------------------------------------------------------------------------------------------------------------
# Actions
# ---------------------------------------------------------------------------------------------------------------


@dataclass(frozen=True)
class Navigate:
    vertices: Tuple[str, ...]
    poses: Tuple[Tuple[float, float, float], ...]  #: (x, y, yaw) per vertex


@dataclass(frozen=True)
class CancelNav:
    pass


@dataclass(frozen=True)
class RequestReservation:
    request: api.ReservationRequest


@dataclass(frozen=True)
class Release:
    lease_id: str
    reason: ReleaseReason


@dataclass(frozen=True)
class TaskFinished:
    task_id: str
    success: bool
    failure_reason: str = ""


@dataclass
class _Lease:
    lease_id: str
    zone: str
    exit_idx: Optional[int]      #: route index of the zone exit while the issuing task is active
    hold_idx: Optional[int] = None
    entered: bool = False


@dataclass(frozen=True)
class _Hold:
    zone: str
    entry: str
    exit_vertex: str
    direction: api.Direction
    zone_path_m: float


def _outside_distance(x: float, y: float, poly: Sequence[Tuple[float, float]]) -> float:
    """0 inside (or on) the polygon, else the distance to its boundary."""
    if point_in_polygon(x, y, poly):
        return 0.0
    best = math.inf
    n = len(poly)
    for i in range(n):
        (x1, y1), (x2, y2) = poly[i], poly[(i + 1) % n]
        dx, dy = x2 - x1, y2 - y1
        L2 = dx * dx + dy * dy
        u = 0.0 if L2 == 0 else max(0.0, min(1.0, ((x - x1) * dx + (y - y1) * dy) / L2))
        best = min(best, math.hypot(x - (x1 + u * dx), y - (y1 + u * dy)))
    return best


class AgentCore:
    def __init__(self, robot_id: str, graph: WarehouseGraph, traffic_control: bool = True, rng_seed: int = 0):
        self.robot_id = robot_id
        self.graph = graph
        self.traffic_control = traffic_control
        self._rng = random.Random(rng_seed)
        # pose
        self._x = self._y = self._yaw = self._speed = 0.0
        self._have_pose = False
        self._pose_t = 0.0
        self._stale = False
        self._last_vertex = ""
        # mode
        self._mode = RobotMode.IDLE
        self._fault = ""                       #: orchestrator-timeout overlay (only while WAITING_RESERVATION)
        self._fault_reason_stuck = ""
        # task
        self._task_id = ""
        self._order_id = ""
        self._route: List[str] = []
        self._reach = 0                        #: index of the last route vertex reached
        self._pickup_idx: Optional[int] = None
        self._end_idx = 0
        self._holds: Dict[int, _Hold] = {}
        self._handled: Set[int] = set()        #: route indices whose pickup dwell is done
        self._granted: Set[int] = set()        #: route indices whose hold has been granted
        self._has_order = False
        # segment
        self._seg_end = 0
        self._attempts = 0
        self._ref_d: Optional[float] = None
        self._ref_t = 0.0
        # dwell
        self._dwell_until = 0.0
        # reservations
        self._req_n = 0
        self._pending: Optional[Tuple[str, float, str]] = None   #: (request_id, sent_t, zone)
        self._abandoned: deque = deque(maxlen=32)   #: request ids whose grant must be released on arrival
        self._epoch = ""
        self._retry_at: Optional[float] = None
        self._wait_since = 0.0
        self._last_contact: Optional[float] = None
        self._leases: List[_Lease] = []

    # -- queries -------------------------------------------------------------------------------------------------
    def can_accept(self) -> bool:
        return self._have_pose and self._mode in (RobotMode.IDLE, RobotMode.STUCK)

    def has_pose(self) -> bool:
        return self._have_pose

    def pose_time(self) -> float:
        return self._pose_t

    def pending_request_id(self) -> str:
        return self._pending[0] if self._pending else ""

    def heartbeat_lease_ids(self) -> List[str]:
        return [l.lease_id for l in self._leases]

    def waiting_zone(self) -> str:
        if self._mode == RobotMode.WAITING_RESERVATION and self._reach in self._holds:
            return self._holds[self._reach].zone
        return ""

    def state(self) -> api.RobotSnapshot:
        active = bool(self._route)
        mode = RobotMode.FAULT if self._fault else self._mode
        overlay = ""
        if not self._have_pose:
            mode, overlay = RobotMode.FAULT, "NO_POSE"
        elif self._stale and not self._fault:
            mode, overlay = RobotMode.FAULT, "STALE_POSE"
        return api.RobotSnapshot(
            robot_id=self.robot_id, x=self._x, y=self._y, yaw=self._yaw, mode=mode,
            last_vertex=self._route[self._reach] if active else self._last_vertex,
            task_id=self._task_id, order_id=self._order_id,
            held_lease_ids=tuple(self.heartbeat_lease_ids()), linear_speed=self._speed,
            fault_reason=overlay or self._fault or (self._fault_reason_stuck if self._mode == RobotMode.STUCK else ""),
            next_vertex=self._route[self._reach + 1] if active and self._reach + 1 < len(self._route) else "",
            remaining_route=tuple(self._route[self._reach + 1:]) if active else ())

    # -- events --------------------------------------------------------------------------------------------------
    def start_task(self, task_id: str, order_id: str, route: Sequence[str], t: float, pickup: str = "",
                   dropoff: str = "") -> List[object]:
        if self._mode not in (RobotMode.IDLE, RobotMode.STUCK):
            raise RuntimeError(f"{self.robot_id} is busy ({self._mode.value})")
        route = list(route)
        if not route:
            raise ValueError("empty route")
        for v in route:
            if v not in self.graph.vertices:
                raise ValueError(f"unknown vertex {v!r}")
        for a, b in zip(route, route[1:]):
            if (a, b) not in self.graph.edges:
                raise ValueError(f"route step {a}->{b} is not a graph edge")
        self._init_time(t)
        self._task_id, self._order_id, self._route = task_id, order_id, route
        self._reach = 0
        self._handled = set()
        self._granted = set()
        self._holds = {}
        self._has_order = bool(order_id)
        self._fault = ""
        self._fault_reason_stuck = ""
        self._pending = None
        self._retry_at = None
        self._mode = RobotMode.NAVIGATING
        n = len(route)
        self._pickup_idx = None
        self._end_idx = n - 1
        if self._has_order:
            if not pickup and not dropoff:
                loading = [i for i, v in enumerate(route) if self.graph.vertices[v].kind == "loading"]
                pickup_idx = loading[0] if loading else None
                drop_idx = n - 1
            else:
                pickup_idx = route.index(pickup) if pickup in route else None
                drop_idx = max((i for i, v in enumerate(route) if v == dropoff), default=n - 1)
            if pickup_idx is not None and pickup_idx < drop_idx:
                self._pickup_idx = pickup_idx
            self._end_idx = drop_idx
        if self.traffic_control:
            self._holds = self._find_holds(route, self._end_idx)
        return self._arrive(t)

    def on_pose(self, x: float, y: float, yaw: float, speed: float, t: float) -> List[object]:
        self._x, self._y, self._yaw, self._speed = x, y, yaw, speed
        self._have_pose = True
        self._pose_t = t
        self._stale = False
        if self._route:
            if self._mode == RobotMode.NAVIGATING:
                for k in range(self._seg_end, self._reach, -1):
                    v = self.graph.vertices[self._route[k]]
                    if math.hypot(v.x - x, v.y - y) <= REACH_RADIUS_M:
                        self._reach = k
                        self._ref_d = None        # passed a vertex: visible progress, restart the stuck window
                        break
        else:
            near = self.graph.nearest_vertex(x, y)
            v = self.graph.vertices[near]
            if math.hypot(v.x - x, v.y - y) <= REACH_RADIUS_M:
                self._last_vertex = near
        return []

    def on_nav_result(self, success: bool, t: float) -> List[object]:
        if self._mode != RobotMode.NAVIGATING or not self._route:
            return []
        if success:
            self._reach = self._seg_end
            return self._arrive(t)
        self._attempts += 1
        if self._attempts >= 2:
            acts: List[object] = self._release_all(ReleaseReason.FAULT)
            acts += self._fail(api.FAILURE_NAV_PREFIX + self._route[self._seg_end], RobotMode.STUCK)
            return acts
        return [self._send_segment(t)]

    def on_reservation_response(self, request_id: str, decision: Optional[api.ReservationDecision],
                                t: float) -> List[object]:
        if self._pending is None or request_id != self._pending[0]:
            if request_id in self._abandoned:           # orphaned request: nobody wants a grant any more
                self._abandoned.remove(request_id)
                if decision is not None and decision.granted and decision.lease_id:
                    return [Release(decision.lease_id, ReleaseReason.TASK_CANCELLED)]
            return []
        self._pending = None
        if decision is None:                          # service call failed: no contact information
            self._retry_at = t + api.RETRY_AFTER_S * self._jitter()
            return []
        self._last_contact = t
        if self._fault:
            self._fault = ""
            self._wait_since = t
        if decision.granted:
            hold = self._holds.get(self._reach)
            zone = hold.zone if hold else ""
            exit_idx = self._exit_index(self._reach) if hold else None
            self._leases.append(_Lease(decision.lease_id, zone, exit_idx, self._reach))
            self._granted.add(self._reach)
            self._retry_at = None
            return self._arrive(t)
        self._retry_at = t + (decision.retry_after_s or api.RETRY_AFTER_S) * self._jitter()
        return []

    def on_orchestrator_heartbeat(self, t: float, epoch: str = "") -> List[object]:
        """``epoch`` identifies the orchestrator instance ("" = unknown, treated as unchanged)."""
        self._last_contact = t
        acts: List[object] = []
        reissue = False
        if self._fault and self._mode == RobotMode.WAITING_RESERVATION:
            self._fault = ""
            self._wait_since = t
            reissue = True
        if epoch:
            changed = bool(self._epoch) and epoch != self._epoch
            self._epoch = epoch
            if changed:
                reissue |= self._on_new_epoch(acts, t)
        if reissue and self._mode == RobotMode.WAITING_RESERVATION and self._reach in self._holds:
            acts += self._issue_request(t)
        return acts

    def on_lease_event(self, lease_id: str, state: api.LeaseState, t: float) -> List[object]:
        """Authority-side lease state change (``/fleet/reservations``). A held lease that is no longer valid is
        dropped; if the robot has not entered the zone it stops and asks again. Inside the zone it keeps driving out
        (the lease is kept so the EXITED release is still sent; the authority blocks the zone meanwhile)."""
        if state not in (api.LeaseState.OCCUPIED_UNKNOWN, api.LeaseState.REVOKED, api.LeaseState.RELEASED,
                         api.LeaseState.EXPIRED):
            return []
        lease = next((l for l in self._leases if l.lease_id == lease_id), None)
        if lease is None:
            return []
        zone = self.graph.zones.get(lease.zone)
        inside = lease.entered or (zone is not None and self._have_pose
                                   and point_in_polygon(self._x, self._y, zone.polygon))
        if inside:
            lease.entered = True
            return []
        self._leases.remove(lease)
        acts: List[object] = []
        if (self._mode == RobotMode.NAVIGATING and self._route and lease.exit_idx is not None
                and self._reach < lease.exit_idx):
            acts.append(CancelNav())
            self._mode = RobotMode.WAITING_RESERVATION
            self._wait_since = t
            self._retry_at = None
            if self._reach not in self._holds and not self._ensure_zone_hold(self._reach):
                if lease.hold_idx is not None:
                    self._reach = lease.hold_idx
            self._granted.discard(self._reach)
            if self._reach in self._holds:
                acts += self._issue_request(t)
        return acts

    def _on_new_epoch(self, acts: List[object], t: float) -> bool:
        """New orchestrator instance: it does not know our leases. Leases of zones we are not inside are dropped
        silently; inside ones are kept (it blocks that zone by recovery) and still released with EXITED later.
        Returns True if the robot has to (re-)request a reservation."""
        again = self._mode == RobotMode.WAITING_RESERVATION
        for lease in list(self._leases):
            zone = self.graph.zones.get(lease.zone)
            if zone is not None and point_in_polygon(self._x, self._y, zone.polygon):
                continue
            self._leases.remove(lease)
            if (self._mode == RobotMode.NAVIGATING and self._route and lease.exit_idx is not None
                    and self._reach < lease.exit_idx):
                acts.append(CancelNav())                  # was about to enter on a grant that no longer exists
                self._mode = RobotMode.WAITING_RESERVATION
                self._wait_since = t
                self._retry_at = None
                if self._reach not in self._holds and not self._ensure_zone_hold(self._reach):
                    if lease.hold_idx is not None:
                        self._reach = lease.hold_idx      # fall back to the hold the grant was issued for
                self._granted.discard(self._reach)
                again = True
        return again

    def tick(self, t: float) -> List[object]:
        self._init_time(t)
        # Stationary dwell/wait modes are exempt (position cannot drift while loading or holding for a grant); the
        # mode is judged before this tick's transitions.
        self._stale = (self._have_pose and t - self._pose_t > POSE_STALE_S
                       and self._mode in (RobotMode.IDLE, RobotMode.STUCK, RobotMode.NAVIGATING))
        acts = self._check_release()
        if self._mode in (RobotMode.LOADING, RobotMode.UNLOADING):
            if t >= self._dwell_until:
                if self._mode == RobotMode.LOADING:
                    self._handled.add(self._reach)
                    acts += self._arrive(t)
                else:
                    acts += self._finish_ok()
        elif self._mode == RobotMode.NAVIGATING:
            acts += self._check_stuck(t)
        elif self._mode == RobotMode.WAITING_RESERVATION:
            if self._pending is not None and t - self._pending[1] >= api.ORCH_RESPONSE_TIMEOUT_S:
                self._pending = None                  # orchestrator did not answer: ask again with a new id
                acts += self._issue_request(t)
            elif self._pending is None and self._retry_at is not None and t >= self._retry_at:
                acts += self._issue_request(t)
            if not self._fault and self.traffic_control:
                ref = max(self._last_contact if self._last_contact is not None else t, self._wait_since)
                if t - ref >= api.ORCHESTRATOR_TIMEOUT_S:
                    self._fault = api.FAULT_ORCHESTRATOR_TIMEOUT
        return acts

    def cancel(self, t: float) -> List[object]:
        if not self._route:
            return []
        acts: List[object] = []
        if self._mode == RobotMode.NAVIGATING:
            acts.append(CancelNav())
        acts += self._release_all(ReleaseReason.TASK_CANCELLED)
        task = self._task_id
        self._clear_task(RobotMode.IDLE)
        acts.append(TaskFinished(task, False, api.FAILURE_CANCELLED))
        return acts

    # -- internals -----------------------------------------------------------------------------------------------
    def _init_time(self, t: float) -> None:
        if self._last_contact is None:
            self._last_contact = t

    def _jitter(self) -> float:
        return self._rng.uniform(1.0 - api.RETRY_JITTER, 1.0 + api.RETRY_JITTER)

    def _find_holds(self, route: List[str], end_idx: int) -> Dict[int, _Hold]:
        """Hold stops: ``route[i]`` is just before a zone entry ``route[i+1]`` that leads into the zone."""
        holds: Dict[int, _Hold] = {}
        g = self.graph
        for i in range(min(end_idx, len(route) - 2)):
            entry = route[i + 1]
            zone = g.zone_of_edge(entry, route[i + 2])
            if not zone or g.zone_entry(zone, entry) is None or route[i] in g.zones[zone].vertices:
                continue
            holds[i] = self._make_hold(route, i, i + 1, zone, entry)
        return holds

    def _make_hold(self, route: List[str], at: int, run_start: int, zone: str, entry: str) -> _Hold:
        """Hold at ``route[at]``; the in-zone run starts at ``route[run_start]``."""
        g = self.graph
        j = run_start
        while j + 1 < len(route) and g.zone_of_edge(route[j], route[j + 1]) == zone:
            j += 1
        entries = g.zones[zone].entries
        if route[j] != entry and g.zone_entry(zone, route[j]) is not None:
            exit_vertex = route[j]
        else:
            exit_vertex = next((e.entry for e in entries if e.entry != entry), entry)
        direction = api.Direction.FORWARD if entries[0].entry == entry else api.Direction.REVERSE
        return _Hold(zone, entry, exit_vertex, direction, g.path_length(route[run_start:j + 1]))

    def _ensure_zone_hold(self, i: int) -> bool:
        """The next edge of ``route[i]`` is a zone edge and no lease covers that zone (route starts at an entry or
        inside a zone, or a grant was lost): create a hold here so the robot requests before moving."""
        if not self.traffic_control or i + 1 >= len(self._route) or i >= self._end_idx:
            return False
        g = self.graph
        zone = g.zone_of_edge(self._route[i], self._route[i + 1])
        if not zone or any(l.zone == zone for l in self._leases):
            return False
        if g.zone_entry(zone, self._route[i]) is not None:
            entry = self._route[i]
        else:                                             # interior start: nearest zone entry to the robot
            ref = (self._x, self._y) if self._have_pose else (g.vertices[self._route[i]].x, g.vertices[self._route[i]].y)
            entry = min((e.entry for e in g.zones[zone].entries),
                        key=lambda n: math.hypot(g.vertices[n].x - ref[0], g.vertices[n].y - ref[1]))
        self._holds[i] = self._make_hold(self._route, i, i, zone, entry)
        return True

    def _exit_index(self, hold_idx: int) -> int:
        zone = self._holds[hold_idx].zone
        j = hold_idx + 1
        while j + 1 < len(self._route) and self.graph.zone_of_edge(self._route[j], self._route[j + 1]) == zone:
            j += 1
        return j

    def _heading(self, k: int) -> float:
        r, g = self._route, self.graph.vertices
        a = g[r[k]]
        if k + 1 < len(r):
            b = g[r[k + 1]]
            return math.atan2(b.y - a.y, b.x - a.x)
        if a.yaw is not None:
            return a.yaw
        if k > 0:
            p = g[r[k - 1]]
            return math.atan2(a.y - p.y, a.x - p.x)
        return 0.0

    def _next_stop(self) -> int:
        stops = [self._end_idx]
        if self._pickup_idx is not None and self._pickup_idx > self._reach and self._pickup_idx not in self._handled:
            stops.append(self._pickup_idx)
        stops += [i for i in self._holds if i > self._reach and i not in self._granted]
        return min(stops)

    def _send_segment(self, t: float) -> Navigate:
        """Navigate to the next stop; a retry (``_attempts`` > 0) re-sends the same segment."""
        if self._attempts == 0:
            self._seg_end = self._next_stop()
        first_idx = min(self._reach + 1, self._seg_end)
        idxs = range(first_idx, self._seg_end + 1)
        g = self.graph.vertices
        nav = Navigate(tuple(self._route[k] for k in idxs),
                       tuple((g[self._route[k]].x, g[self._route[k]].y, self._heading(k)) for k in idxs))
        self._mode = RobotMode.NAVIGATING
        self._ref_t = t
        self._ref_d = self._dist_to_target() if self._have_pose else None
        return nav

    def _dist_to_target(self) -> float:
        """Remaining route distance to the segment end: pose to the next unreached vertex plus the edges after it."""
        nxt = min(self._reach + 1, self._seg_end)
        v = self.graph.vertices[self._route[nxt]]
        d = math.hypot(v.x - self._x, v.y - self._y)
        return d + self.graph.path_length(self._route[nxt:self._seg_end + 1])

    def _arrive(self, t: float) -> List[object]:
        """The robot stands at ``route[self._reach]``: do whatever that vertex calls for."""
        i = self._reach
        if i >= self._end_idx:
            if self._has_order:
                self._mode = RobotMode.UNLOADING
                self._dwell_until = t + api.LOAD_DWELL_S
                return []
            return self._finish_ok()
        if i == self._pickup_idx and i not in self._handled:
            self._mode = RobotMode.LOADING
            self._dwell_until = t + api.LOAD_DWELL_S
            return []
        if i not in self._granted and (i in self._holds or self._ensure_zone_hold(i)):
            self._mode = RobotMode.WAITING_RESERVATION
            self._wait_since = t
            self._retry_at = None
            return self._issue_request(t)
        self._attempts = 0
        return [self._send_segment(t)]

    def _issue_request(self, t: float) -> List[object]:
        hold = self._holds[self._reach]
        self._req_n += 1
        rid = f"{self.robot_id}-{self._req_n}"
        if self._pending is not None and self._pending[2] != hold.zone:
            self._abandoned.append(self._pending[0])   # superseded by a request for another zone
        self._pending = (rid, t, hold.zone)
        self._retry_at = None
        req = api.ReservationRequest(
            request_id=rid, robot_id=self.robot_id, zone_id=hold.zone, entry_vertex=hold.entry,
            exit_vertex=hold.exit_vertex, direction=hold.direction, earliest_entry_t=t,
            expected_exit_t=t + hold.zone_path_m / ZONE_SPEED_MPS)
        return [RequestReservation(req)]

    def _check_release(self) -> List[object]:
        acts: List[object] = []
        for lease in list(self._leases):
            if lease.zone not in self.graph.zones:
                continue
            d = _outside_distance(self._x, self._y, self.graph.zones[lease.zone].polygon)
            if d == 0.0:
                lease.entered = True
            past_exit = lease.exit_idx is not None and self._route and self._reach >= lease.exit_idx
            if d >= api.RELEASE_MARGIN_M and (lease.entered or past_exit):
                acts.append(Release(lease.lease_id, ReleaseReason.EXITED))
                self._leases.remove(lease)
        return acts

    def _abandon_pending(self) -> None:
        if self._pending is not None:
            self._abandoned.append(self._pending[0])
        self._pending = None

    def _release_all(self, reason: ReleaseReason) -> List[object]:
        acts = [Release(l.lease_id, reason) for l in self._leases]
        self._leases = []
        return acts

    def _check_stuck(self, t: float) -> List[object]:
        if not self._have_pose:
            return []
        d = self._dist_to_target()
        if self._ref_d is None or self._ref_d - d >= api.STUCK_MIN_PROGRESS_M:
            self._ref_d, self._ref_t = d, t
            return []
        if t - self._ref_t >= api.STUCK_WINDOW_S:
            acts: List[object] = [CancelNav()]
            acts += self._release_all(ReleaseReason.FAULT)
            acts += self._fail(api.FAILURE_STUCK_TIMEOUT, RobotMode.STUCK)
            return acts
        return []

    def _fail(self, reason: str, mode: RobotMode) -> List[object]:
        task = self._task_id
        self._clear_task(mode)
        self._fault_reason_stuck = reason
        return [TaskFinished(task, False, reason)]

    def _finish_ok(self) -> List[object]:
        task = self._task_id
        self._clear_task(RobotMode.IDLE)
        return [TaskFinished(task, True, "")]

    def _clear_task(self, mode: RobotMode) -> None:
        if self._route:
            self._last_vertex = self._route[self._reach]
        for lease in self._leases:
            lease.exit_idx = None
        self._route = []
        self._holds = {}
        self._handled = set()
        self._granted = set()
        self._task_id = self._order_id = ""
        self._abandon_pending()
        self._retry_at = None
        self._fault = ""
        self._fault_reason_stuck = ""
        self._reach = 0
        self._mode = mode
