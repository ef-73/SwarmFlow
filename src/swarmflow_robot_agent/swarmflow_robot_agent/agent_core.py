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
ENTERED_MARGIN_M = 0.2        #: "entered" needs the pose this far inside the polygon (entry vertices lie on it)
REACH_RADIUS_M = 0.6          #: pose within this of a route vertex counts as having reached it
FAILURE_PALLET_ON_BOARD = "PALLET_ON_BOARD"   #: start_task refused: a pallet is on board (T042)
PALLET_LIFT_S = 3.0           #: v2 goal mode: dwell (mode LIFTING) when picking up a pallet
GOAL_RETRY_MIN_S = 1.0        #: v2 goal mode: a failed Navigate is re-sent after a uniform delay in this range
GOAL_RETRY_MAX_S = 3.0
GOAL_PROGRESS_MOVE_M = 0.5    #: v2 goal mode: displacement this far from the last reference also counts as progress
PREDOCK_M = 1.2               #: pallet docking (T042): Nav2 goal this far in front of the pallet, along its yaw
DOCK_AHEAD_M = 0.7            #: deep drop: drive this far past the dock vertex along the dock yaw
UNDOCK_M = 0.9                #: back out this far (along the dock yaw) after setting the pallet down
DOCK_TOL_LAT_M = 0.03         #: dock drive reached: across-track tolerance
DOCK_TOL_LON_M = 0.02         #: dock drive reached: along-track tolerance
DOCK_TIMEOUT_S = 30.0         #: dock drive without STUCK_MIN_PROGRESS_M progress this long -> task failure


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
class DockDrive:
    """Straight closed-loop drive to ``target`` (x, y, yaw) along the target heading; ``reverse`` backs up."""
    target: Tuple[float, float, float]
    reverse: bool = False


@dataclass(frozen=True)
class StopDrive:
    pass


@dataclass(frozen=True)
class SetFootprint:
    kind: str                    #: "carry" (padded for a pallet on board) | "normal"


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


def _inside_margin(x: float, y: float, poly: Sequence[Tuple[float, float]]) -> float:
    """Distance to the nearest polygon edge if the point is inside (or on) it, else -1."""
    if not point_in_polygon(x, y, poly):
        return -1.0
    best = math.inf
    n = len(poly)
    for i in range(n):
        (x1, y1), (x2, y2) = poly[i], poly[(i + 1) % n]
        dx, dy = x2 - x1, y2 - y1
        L2 = dx * dx + dy * dy
        u = 0.0 if L2 == 0 else max(0.0, min(1.0, ((x - x1) * dx + (y - y1) * dy) / L2))
        best = min(best, math.hypot(x - (x1 + u * dx), y - (y1 + u * dy)))
    return best


# Pallet dock drive (T042): direct velocity commands, hard-bounded
DRIVE_HZ = 20.0
DRIVE_MAX_MPS = 0.15            #: forward / reverse speed limit
DRIVE_MIN_MPS = 0.03            #: creep speed near the target (below this the base would not move reliably)
DRIVE_MAX_RADPS = 0.5           #: yaw rate limit
DRIVE_SLOW_GAIN = 0.5           #: speed = gain * remaining distance (1/s), clamped to [MIN, MAX]
DRIVE_STOP_M = 0.01             #: command zero speed this close to the target (core stops at 2 cm)
DRIVE_LOOKAHEAD_M = 0.3         #: pure pursuit look-ahead on the target line
DRIVE_STEER_MAX_RAD = 0.5       #: steering toward the line is limited to this angle
DRIVE_HEADING_GAIN = 1.5
DRIVE_TURN_ONLY_RAD = 0.6       #: heading error above this: rotate in place, no translation
DRIVE_POSE_MAX_AGE_S = 0.5      #: a TF older than this stops the robot


class FootprintSync:
    """Retry bookkeeping for the costmap footprint (T042): the wanted footprint kind is remembered per costmap and
    re-sent every ``period_s`` until a call for that kind reports success. Pure, time is an argument."""

    def __init__(self, names: Sequence[str], period_s: float = 1.0):
        self.period_s = period_s
        self.want: Dict[str, Optional[str]] = {n: None for n in names}
        self.applied: Dict[str, Optional[str]] = {n: None for n in names}
        self._last: Dict[str, float] = {}

    def set_wanted(self, kind: str) -> None:
        for n in self.want:
            self.want[n] = kind
            self._last.pop(n, None)                       # send at the next pump

    def due(self, now: float) -> List[Tuple[str, str]]:
        """(costmap, kind) pairs to send now; callers report each with :meth:`sent`."""
        return [(n, k) for n, k in self.want.items()
                if k is not None and self.applied[n] != k and now - self._last.get(n, -math.inf) >= self.period_s]

    def sent(self, name: str, now: float) -> None:
        self._last[name] = now

    def done(self, name: str, kind: str, ok: bool) -> None:
        if ok and self.want.get(name) == kind:
            self.applied[name] = kind

    def pending(self) -> bool:
        return any(k is not None and self.applied[n] != k for n, k in self.want.items())


def _wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def drive_command(pose, target, reverse: bool):
    """Pure control law of the dock drive: (linear m/s, angular rad/s) for ``pose`` = (x, y, yaw) towards ``target`` =
    (x, y, yaw), driving along the target heading (backwards if ``reverse``). Linear is signed (negative = reverse),
    |linear| <= DRIVE_MAX_MPS, |angular| <= DRIVE_MAX_RADPS."""
    x, y, yaw = pose
    tx, ty, tyaw = target
    if not all(math.isfinite(c) for c in (x, y, yaw, tx, ty, tyaw)):
        return 0.0, 0.0                                              # NaN / inf pose or target: stand still
    sgn = -1.0 if reverse else 1.0
    travel = tyaw + (math.pi if reverse else 0.0)                    # direction of motion along the line
    dx, dy = x - tx, y - ty
    remaining = -(dx * math.cos(tyaw) + dy * math.sin(tyaw)) * sgn   # distance still to drive (<= 0 once passed)
    cte = -dx * math.sin(travel) + dy * math.cos(travel)             # offset left of the travel direction
    steer = max(-DRIVE_STEER_MAX_RAD, min(DRIVE_STEER_MAX_RAD, math.atan2(cte, DRIVE_LOOKAHEAD_M)))
    desired_body = travel - steer - (math.pi if reverse else 0.0)
    herr = _wrap(desired_body - yaw)
    w = max(-DRIVE_MAX_RADPS, min(DRIVE_MAX_RADPS, DRIVE_HEADING_GAIN * herr))
    if remaining <= DRIVE_STOP_M or abs(herr) > DRIVE_TURN_ONLY_RAD:
        v = 0.0
    else:
        v = min(DRIVE_MAX_MPS, max(DRIVE_MIN_MPS, DRIVE_SLOW_GAIN * remaining)) * math.cos(herr)
    return sgn * min(v, DRIVE_MAX_MPS), w


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
        self._lost_ids: deque = deque(maxlen=64)    #: lease ids reported lost (possibly before their grant arrived)
        self._angular_speed = 0.0
        # v2 goal mode (ROUTE_GOALS); all of this is inert for waypoint tasks
        self._goal_mode = False
        self._goal_i = 0                       #: index of the current goal in the route
        self._order_type = api.ORDER_TRANSPORT
        self._carrying = False                 #: pallet order: the pallet is on board
        self._nav_inflight = False
        self._goal_retry_at: Optional[float] = None
        self._gref_d: Optional[float] = None   #: blocked detection: distance to the goal at the last progress
        self._gref_t = 0.0
        self._gref_xy: Optional[Tuple[float, float]] = None   #: position at the last progress reference
        # pallet docking (T042): phase None | "pick_drive" | "lift" | "drop_drive" | "unload" | "back_drive"
        self._pallet_on_board = False          #: set at lift completion, cleared when it is set down; survives tasks
        self._dock: Optional[str] = None
        self._dock_target: Tuple[float, float, float] = (0.0, 0.0, 0.0)
        self._dock_reverse = False
        self._dock_ref_d = 0.0
        self._dock_ref_t = 0.0
        self._goal_yaw = 0.0                   #: yaw of the current goal (vertex yaw, else the approach direction)

    # -- queries -------------------------------------------------------------------------------------------------
    def can_accept(self) -> bool:
        return (self._have_pose and not self._stale and not self._route
                and (self._mode in (RobotMode.IDLE, RobotMode.STUCK)
                     or (self._mode == RobotMode.CARRYING and self._pallet_on_board)))

    def driving(self) -> bool:
        """True while a straight dock drive is commanded (the node must keep publishing velocity only then)."""
        return self._dock in ("pick_drive", "drop_drive", "back_drive")

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
            angular_speed=self._angular_speed,
            last_vertex=self._route[self._reach] if active and not self._goal_mode else self._last_vertex,
            task_id=self._task_id, order_id=self._order_id,
            held_lease_ids=tuple(self.heartbeat_lease_ids()), linear_speed=self._speed,
            fault_reason=overlay or self._fault or (self._fault_reason_stuck if self._mode == RobotMode.STUCK else ""),
            next_vertex=self._next_vertex(), remaining_route=self._remaining_route())

    def _next_vertex(self) -> str:
        if not self._route:
            return ""
        i = self._goal_i if self._goal_mode else self._reach + 1
        return self._route[i] if i < len(self._route) else ""

    def _remaining_route(self) -> Tuple[str, ...]:
        if not self._route:
            return ()
        return tuple(self._route[self._goal_i if self._goal_mode else self._reach + 1:])

    # -- events --------------------------------------------------------------------------------------------------
    def start_task(self, task_id: str, order_id: str, route: Sequence[str], t: float, pickup: str = "",
                   dropoff: str = "", route_kind: str = api.ROUTE_WAYPOINTS,
                   order_type: str = api.ORDER_TRANSPORT) -> List[object]:
        if (self._mode not in (RobotMode.IDLE, RobotMode.STUCK)
                and not (self._mode == RobotMode.CARRYING and self._pallet_on_board)) or self._route:
            raise RuntimeError(f"{self.robot_id} is busy ({self._mode.value})")
        route = list(route)
        if not route:
            raise ValueError("empty route")
        if self._pallet_on_board and not self._is_dock_follow_up(route, route_kind, order_type):
            return [TaskFinished(task_id, False, FAILURE_PALLET_ON_BOARD)]
        if route_kind == api.ROUTE_GOALS:
            return self._start_goal_task(task_id, order_id, route, t, order_type)
        if route_kind != api.ROUTE_WAYPOINTS:
            raise ValueError(f"unknown route kind {route_kind!r}")
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

    def on_pose(self, x: float, y: float, yaw: float, speed: float, t: float,
                angular_speed: float = 0.0) -> List[object]:
        self._x, self._y, self._yaw, self._speed = x, y, yaw, speed
        self._angular_speed = angular_speed
        self._have_pose = True
        self._pose_t = t
        self._stale = False
        if self._goal_mode and self._route:
            return self._goal_on_pose(t)
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
        if self._goal_mode and self._route:
            return self._goal_nav_result(success, t)
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
        if decision.granted and decision.lease_id in self._lost_ids:
            self._lost_ids.remove(decision.lease_id)      # revoked/expired before the grant reached us: not valid
            self._retry_at = None
            return self._issue_request(t) if self._reach in self._holds else []
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
            if lease_id not in self._lost_ids:
                self._lost_ids.append(lease_id)           # may be the grant's event overtaking its response
            return []
        if not self._have_pose or t - self._pose_t > POSE_STALE_S:
            return []                                     # pose unknown/stale: may be inside, keep the lease
        zone = self.graph.zones.get(lease.zone)
        if lease.entered or (zone is not None and self._have_pose
                             and _inside_margin(self._x, self._y, zone.polygon) >= ENTERED_MARGIN_M):
            lease.entered = True
            return []
        self._leases.remove(lease)
        if lease.hold_idx is not None:
            self._granted.discard(lease.hold_idx)
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
            else:                                         # nowhere to ask: do not wait silently
                acts += self._release_all(ReleaseReason.FAULT)
                acts += self._fail("NO_HOLD", RobotMode.STUCK)
        return acts

    def _on_new_epoch(self, acts: List[object], t: float) -> bool:
        """New orchestrator instance: it does not know our leases. Leases of zones we are not inside are dropped
        silently; inside ones are kept (it blocks that zone by recovery) and still released with EXITED later.
        Returns True if the robot has to (re-)request a reservation."""
        again = self._mode == RobotMode.WAITING_RESERVATION
        self._lost_ids.clear()                            # ids are per orchestrator instance
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
                       and self._mode in (RobotMode.IDLE, RobotMode.STUCK, RobotMode.NAVIGATING,
                                          RobotMode.CARRYING))
        acts = self._check_release()
        if self._goal_mode and self._route:
            return acts + self._goal_tick(t)
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
                self._abandon_pending()               # orchestrator did not answer: ask again with a new id
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
        if self._mode == RobotMode.NAVIGATING or (self._goal_mode and self._nav_inflight):
            acts.append(CancelNav())
        acts += self._dock_abort()
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
            poly = self.graph.zones[lease.zone].polygon
            d = _outside_distance(self._x, self._y, poly)
            if _inside_margin(self._x, self._y, poly) >= ENTERED_MARGIN_M:
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

    def _is_dock_follow_up(self, route: List[str], route_kind: str, order_type: str) -> bool:
        """With a pallet on board only a pallet-order goal task without a pallet goal (a trip to the dock) may start."""
        return (route_kind == api.ROUTE_GOALS and order_type == api.ORDER_PALLET
                and all(v in self.graph.vertices and self.graph.vertices[v].kind != "pallet" for v in route))

    def _clear_task(self, mode: RobotMode) -> None:
        if self._route and not self._goal_mode:
            self._last_vertex = self._route[self._reach]
        self._goal_mode = False
        self._goal_i = 0
        self._order_type = api.ORDER_TRANSPORT
        self._carrying = False
        self._nav_inflight = False
        self._goal_retry_at = None
        self._gref_d = None
        self._dock = None
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
        if self._pallet_on_board and mode == RobotMode.IDLE:
            mode = RobotMode.CARRYING                     # the pallet stays on board after the task ends
        self._mode = mode

    # -- v2 goal mode (ROUTE_GOALS) --------------------------------------------------------------------------------
    # Kept apart from the v1 waypoint logic above: no holds, no reservations, no edge checks, no STUCK_TIMEOUT.
    # Phases: moving (NAVIGATING / CARRYING, or STUCK when blocked) with one single-pose Navigate in flight, or a
    # dwell (LOADING / UNLOADING / LIFTING) at a goal that Nav2 has reached.

    def _start_goal_task(self, task_id: str, order_id: str, route: List[str], t: float,
                         order_type: str) -> List[object]:
        for v in route:
            if v not in self.graph.vertices:
                raise ValueError(f"unknown vertex {v!r}")
        self._init_time(t)
        self._task_id, self._order_id, self._route = task_id, order_id, route
        self._reach = 0
        self._handled = set()
        self._granted = set()
        self._holds = {}
        self._pickup_idx = None
        self._has_order = bool(order_id)
        self._fault = ""
        self._fault_reason_stuck = ""
        self._pending = None
        self._retry_at = None
        self._goal_mode = True
        self._goal_i = 0
        self._order_type = order_type
        # a pallet order whose route has no pallet goal (follow-up task) already has the pallet on board
        self._carrying = self._pallet_on_board or (order_type == api.ORDER_PALLET
                                                   and not any(self.graph.vertices[v].kind == "pallet" for v in route))
        self._nav_inflight = False
        self._goal_retry_at = None
        self._gref_d = None
        self._dock = None
        self._mode = self._moving_mode()
        return self._goal_begin(t)

    def _moving_mode(self) -> RobotMode:
        return RobotMode.CARRYING if self._carrying else RobotMode.NAVIGATING

    def _goal_dwell(self, name: str) -> Optional[Tuple[RobotMode, float]]:
        """Dwell (mode, seconds) at a goal vertex by its kind, or None for a no-dwell goal."""
        kind = self.graph.vertices[name].kind
        if kind == "dock":
            secs = api.UNLOAD_DWELL_PALLET_S if self._order_type == api.ORDER_PALLET else api.UNLOAD_DWELL_BOX_S
            return RobotMode.UNLOADING, secs
        if kind == "loading":
            return RobotMode.LOADING, api.LOAD_DWELL_S
        if kind == "delivery":
            return RobotMode.UNLOADING, api.LOAD_DWELL_S
        return None

    def _is_pallet_goal(self, name: str) -> bool:
        """Goals of a pallet order that end in a closed-loop dock drive instead of a plain dwell (T042)."""
        return self._order_type == api.ORDER_PALLET and self.graph.vertices[name].kind in ("pallet", "dock")

    def _goal_pose(self, name: str) -> Tuple[float, float, float]:
        """Nav2 target of a goal: the vertex pose, or for a pallet the pre-dock pose in front of it."""
        v = self.graph.vertices[name]
        if self._order_type == api.ORDER_PALLET and v.kind == "pallet":
            return (v.x - PREDOCK_M * math.cos(self._goal_yaw), v.y - PREDOCK_M * math.sin(self._goal_yaw),
                    self._goal_yaw)
        return (v.x, v.y, self._goal_yaw)

    def _near_goal(self, name: str, t: float) -> bool:
        """Pose within tolerance of the goal; a stale pose never counts."""
        v = self.graph.vertices[name]
        return (self._have_pose and t - self._pose_t <= POSE_STALE_S
                and math.hypot(v.x - self._x, v.y - self._y) <= api.GOAL_TOLERANCE_M)

    def _goal_navigate(self, name: str) -> Navigate:
        return Navigate((name,), (self._goal_pose(name),))

    def _set_goal_yaw(self, name: str) -> None:
        v = self.graph.vertices[name]
        if v.yaw is not None:
            self._goal_yaw = v.yaw
        elif self._have_pose:
            self._goal_yaw = math.atan2(v.y - self._y, v.x - self._x)
        else:
            self._goal_yaw = 0.0

    def _goal_begin(self, t: float) -> List[object]:
        """Start the goal at ``_goal_i``; no-dwell goals the robot already stands at are passed at once."""
        while True:
            if self._goal_i >= len(self._route):
                return self._finish_ok()
            name = self._route[self._goal_i]
            self._set_goal_yaw(name)
            if self._goal_dwell(name) is None and not self._is_pallet_goal(name) and self._near_goal(name, t):
                self._last_vertex = name
                self._goal_i += 1
                continue
            break
        self._mode = self._moving_mode()
        self._fault_reason_stuck = ""
        self._goal_retry_at = None
        self._nav_inflight = True
        self._gref_t = t
        self._gref_d = self._goal_dist()
        self._gref_xy = (self._x, self._y) if self._have_pose else None
        return [self._goal_navigate(name)]

    def _goal_dist(self) -> Optional[float]:
        if not self._have_pose or self._goal_i >= len(self._route):
            return None
        x, y, _ = self._goal_pose(self._route[self._goal_i])
        return math.hypot(x - self._x, y - self._y)

    def _goal_moving(self) -> bool:
        return self._mode in (RobotMode.NAVIGATING, RobotMode.CARRYING, RobotMode.STUCK)

    def _goal_set_blocked(self) -> None:
        self._mode = RobotMode.STUCK
        self._fault_reason_stuck = api.FAULT_BLOCKED

    def _goal_clear_blocked(self) -> None:
        if self._mode == RobotMode.STUCK:
            self._mode = self._moving_mode()
        self._fault_reason_stuck = ""

    def _goal_on_pose(self, t: float) -> List[object]:
        if self._dock is not None:
            return self._dock_on_pose(t)
        if not self._goal_moving() or self._goal_i >= len(self._route):
            return []
        name = self._route[self._goal_i]
        if not self._near_goal(name, t):
            return []
        self._last_vertex = name
        if self._goal_dwell(name) is not None or self._is_pallet_goal(name):
            return []                                     # dwell / dock goals wait for Nav2 success
        acts: List[object] = [CancelNav()] if self._nav_inflight else []
        self._nav_inflight = False
        self._goal_clear_blocked()
        self._goal_i += 1
        return acts + self._goal_begin(t)

    def _goal_nav_result(self, success: bool, t: float) -> List[object]:
        if self._dock is not None:
            return []                                     # a drive is under way: Nav2 results are late/duplicates
        if not self._goal_moving() or self._goal_i >= len(self._route):
            return []                                     # late/duplicate result (dwell, finished task)
        self._nav_inflight = False
        if not success:
            self._goal_set_blocked()
            self._goal_retry_at = t + self._rng.uniform(GOAL_RETRY_MIN_S, GOAL_RETRY_MAX_S)
            return []
        name = self._route[self._goal_i]
        self._last_vertex = name
        self._goal_retry_at = None
        self._goal_clear_blocked()
        if self._is_pallet_goal(name):
            return self._dock_start(name, t)
        dwell = self._goal_dwell(name)
        if dwell is None:
            self._goal_i += 1
            return self._goal_begin(t)
        self._mode, secs = dwell
        self._dwell_until = t + secs
        return []

    def _goal_tick(self, t: float) -> List[object]:
        if self._dock is not None:
            return self._dock_tick(t)
        if self._mode in (RobotMode.LOADING, RobotMode.UNLOADING, RobotMode.LIFTING):
            if t < self._dwell_until:
                return []
            if self._mode == RobotMode.LIFTING:
                self._carrying = True
            self._goal_i += 1
            return self._goal_begin(t)
        if not self._goal_moving() or self._goal_i >= len(self._route):
            return []
        d = self._goal_dist()
        if d is not None:
            moved = (self._gref_xy is not None
                     and math.hypot(self._x - self._gref_xy[0], self._y - self._gref_xy[1]) >= GOAL_PROGRESS_MOVE_M)
            if self._gref_d is None or self._gref_d - d >= api.STUCK_MIN_PROGRESS_M or moved:
                # progress: closer to the goal, or displaced (a detour around an obstacle); turning in place is not
                self._gref_d, self._gref_t, self._gref_xy = d, t, (self._x, self._y)
                if self._goal_retry_at is None:           # a failed Navigate stays reported until it is re-sent
                    self._goal_clear_blocked()
            elif self._mode != RobotMode.STUCK and t - self._gref_t >= api.BLOCKED_REPORT_S:
                self._goal_set_blocked()                  # keep the task, do not cancel navigation
        if self._goal_retry_at is not None and t >= self._goal_retry_at:
            self._goal_retry_at = None
            self._nav_inflight = True
            return [self._goal_navigate(self._route[self._goal_i])]
        return []

    # -- pallet docking (T042) -------------------------------------------------------------------------------------
    # Nav2 cannot plan between pallet legs, so the last metre in and out is a straight DockDrive. The drive runs only
    # after Nav2 reported its result (no Nav2 goal is active), and every exit path emits StopDrive.

    def _dock_geometry(self, target: Tuple[float, float, float]) -> Tuple[float, float]:
        """(lateral, longitudinal) offset of ``target`` from the pose, along the target heading."""
        tx, ty, tyaw = target
        dx, dy = tx - self._x, ty - self._y
        return -dx * math.sin(tyaw) + dy * math.cos(tyaw), dx * math.cos(tyaw) + dy * math.sin(tyaw)

    def _dock_begin_drive(self, phase: str, mode: RobotMode, target: Tuple[float, float, float], reverse: bool,
                          t: float) -> List[object]:
        self._dock, self._mode = phase, mode
        self._dock_target, self._dock_reverse = target, reverse
        self._dock_ref_d = math.hypot(target[0] - self._x, target[1] - self._y)
        self._dock_ref_t = t
        self._goal_retry_at = None
        return [DockDrive(target, reverse)]

    def _dock_start(self, name: str, t: float) -> List[object]:
        """Nav2 reached the pre-dock pose (pallet) or the dock vertex: start the straight drive."""
        v = self.graph.vertices[name]
        yaw = self._goal_yaw
        if v.kind == "pallet":
            return self._dock_begin_drive("pick_drive", RobotMode.LIFTING, (v.x, v.y, yaw), False, t)
        target = (v.x + DOCK_AHEAD_M * math.cos(yaw), v.y + DOCK_AHEAD_M * math.sin(yaw), yaw)
        return self._dock_begin_drive("drop_drive", RobotMode.CARRYING, target, False, t)

    def _dock_on_pose(self, t: float) -> List[object]:
        if self._dock not in ("pick_drive", "drop_drive", "back_drive"):
            return []
        lat, lon = self._dock_geometry(self._dock_target)
        remaining = -lon if self._dock_reverse else lon        # distance still to drive (<= 0: reached or passed)
        if abs(lat) > DOCK_TOL_LAT_M or remaining > DOCK_TOL_LON_M:
            # Fail-safe: a lateral error above tolerance at the stop distance is never "reached". The node's control
            # law commands zero speed there (it only rotates), so the robot stalls and the DOCK_TIMEOUT_S progress
            # timeout fails the task instead of the robot driving on.
            return []
        name = self._route[self._goal_i]
        acts: List[object] = [StopDrive()]
        if self._dock == "pick_drive":
            self._last_vertex = name
            self._dock, self._mode = "lift", RobotMode.LIFTING
            self._dwell_until = t + PALLET_LIFT_S
        elif self._dock == "drop_drive":
            self._dock, self._mode = "unload", RobotMode.UNLOADING
            self._dwell_until = t + api.UNLOAD_DWELL_PALLET_S
        else:                                                  # backed out: the pallet stays at the dock
            self._dock = None
            self._carrying = False
            self._pallet_on_board = False
            self._last_vertex = name
            acts.append(SetFootprint("normal"))
            self._goal_i += 1
            acts += self._goal_begin(t)
        return acts

    def _dock_tick(self, t: float) -> List[object]:
        if self._dock in ("lift", "unload"):
            if t < self._dwell_until:
                return []
            name = self._route[self._goal_i]
            if self._dock == "lift":
                self._dock = None
                self._carrying = True
                self._pallet_on_board = True
                self._last_vertex = name
                self._goal_i += 1
                return [SetFootprint("carry")] + self._goal_begin(t)      # footprint first, then plan with it
            self._pallet_on_board = False                 # unload dwell over: the pallet is down
            tx, ty, yaw = self._dock_target
            back = (tx - UNDOCK_M * math.cos(yaw), ty - UNDOCK_M * math.sin(yaw), yaw)
            return self._dock_begin_drive("back_drive", RobotMode.LIFTING, back, True, t)
        d = math.hypot(self._dock_target[0] - self._x, self._dock_target[1] - self._y)
        if self._dock_ref_d - d >= api.STUCK_MIN_PROGRESS_M:
            self._dock_ref_d, self._dock_ref_t = d, t
        elif t - self._dock_ref_t >= DOCK_TIMEOUT_S:
            name = self._route[self._goal_i]
            acts = self._dock_abort()
            acts += self._release_all(ReleaseReason.FAULT)
            return acts + self._fail(api.FAILURE_NAV_PREFIX + name, RobotMode.STUCK)
        return []

    def _dock_abort(self) -> List[object]:
        """The task ends while a dock phase is active: stop the drive; once the pallet was set down the normal
        footprint applies again. Called before the task state is cleared."""
        phase, self._dock = self._dock, None
        acts: List[object] = [StopDrive()] if phase in ("pick_drive", "drop_drive", "back_drive") else []
        if phase in ("unload", "back_drive"):             # the pallet is already down
            self._pallet_on_board = False
            acts.append(SetFootprint("normal"))
        return acts
