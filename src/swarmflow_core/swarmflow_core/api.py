"""SwarmFlow orchestrator library API — frozen contract (design §6.7, milestone M2).

Pure Python: no ROS imports anywhere in ``swarmflow_core`` (CI enforces). All times are **sim seconds** passed in by
the caller, so every implementation is deterministic and testable.

Implementation map (frozen module paths and constructors; behaviour specified in the task cards):

- ``swarmflow_core.graph.load_graph(nav_graph_yaml, zones_yaml) -> WarehouseGraph`` (implements :class:`Graph`),
  ``swarmflow_core.graph.load_layout(generated_dir)``, ``swarmflow_core.graph.point_in_polygon(x, y, poly)``.
- ``swarmflow_core.reservations.FcfsReservationAuthority(graph)`` (implements :class:`ReservationAuthority`).
- ``swarmflow_core.policies.fcfs.FcfsPolicy(name="fcfs")`` (implements :class:`Policy`; ``name="independent"`` for
  Baseline A).
- ``swarmflow_core.fleet.FleetCore`` — order book + policy + stuck/park rules (the orchestrator's brain).
- ``swarmflow_core.decisions.render(decision) -> str`` — the only producer of ``Decision.explanation``.
- ``swarmflow_core.backends.fake.FakeBackend`` (implements :class:`Backend`).
- ``swarmflow_robot_agent.agent_core.AgentCore`` — robot-side protocol client (§6.3, §6.5 rules 3–6), pure Python
  (no rclpy); reused by the 2D sim.

ROS QoS for the §6.6 fleet topics: all **reliable**. ``/fleet/reservations`` and ``/fleet/order_status`` are also
**transient-local** (depth 100) so late joiners see the current state; heartbeats and releases depth 10.

Order lifecycle (how any backend's observations map to :class:`OrderState`):

- released order → ``QUEUED``; dispatched → ``ASSIGNED``;
- a robot snapshot with that ``order_id`` in mode ``LOADING`` → ``PICKING_UP``;
- the same robot leaving ``LOADING`` (any other mode, still on that order) → ``IN_TRANSIT`` ("picked up");
- :class:`TaskResult` success for the order's task → ``DELIVERED``;
- :class:`TaskResult` failure ``STUCK_TIMEOUT`` → ``FAILED`` (``STUCK_TIMEOUT``, §13.5); any other failure → back to
  ``QUEUED`` (``REASSIGN``) if not yet picked up, else ``FAILED`` (``STUCK_AFTER_PICKUP``, §6.3 item 5). After a
  failure the robot gets a move-only task to the nearest free park vertex.
- A **move-only task** (park) is an :class:`Assignment` with ``order_id == ""``; over ROS its ``DispatchTask.order``
  has an empty ``order_id``.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Mapping, Optional, Protocol, Sequence, Tuple, runtime_checkable

API_VERSION = 1

# ---------------------------------------------------------------------------------------------------------------
# Protocol constants (design §6.5, §13.3, §13.5). Adapters must use these, never their own literals.
# ---------------------------------------------------------------------------------------------------------------

LEASE_TTL_S = 5.0                 #: lease lifetime, renewed by each heartbeat (§6.5 rule 2)
HEARTBEAT_PERIOD_S = 1.0          #: agent → orchestrator reservation heartbeat period
LEASE_HARD_MIN_S = 10.0           #: hard bound = expected_exit + max(LEASE_HARD_MIN_S, LEASE_HARD_FRACTION × span)
LEASE_HARD_FRACTION = 0.5
RETRY_AFTER_S = 1.0               #: default retry delay after a deny (§6.5 rule 5)
RETRY_JITTER = 0.2                #: ± fraction applied by the agent to retry_after_s
ORCH_RESPONSE_TIMEOUT_S = 2.0     #: no service response within this → orchestrator considered down (§6.5 rule 6)
ORCH_HEARTBEAT_TIMEOUT_S = 3.0    #: no /fleet/orchestrator_heartbeat within this → orchestrator considered down
ORCHESTRATOR_TIMEOUT_S = 10.0     #: down for this long → agent mode FAULT (ORCHESTRATOR_TIMEOUT)
ROBOT_STATE_PERIOD_S = 0.2        #: RobotState publish period (5 Hz, §6.3)
RELEASE_MARGIN_M = 1.0            #: agent releases once its pose is >= this far outside the zone polygon (§6.5 r3)
RECOVERY_GRACE_S = 2.0            #: orchestrator start: collect RobotStates this long, then authority.recover()
STUCK_WINDOW_S = 60.0             #: < STUCK_MIN_PROGRESS_M towards the segment goal within this → STUCK (§13.5)
STUCK_MIN_PROGRESS_M = 0.2
LOAD_DWELL_S = 3.0                #: dwell in LOADING / UNLOADING (§7.4)


def lease_hard_expiry(earliest_entry_t: float, expected_exit_t: float) -> float:
    """Hard upper bound of a lease (§6.5 rule 2): ``expected_exit + max(10 s, 0.5 × (expected_exit − earliest_entry))``.
    Callers clamp both arguments to the grant time first (see :class:`ReservationAuthority`)."""
    span = max(0.0, expected_exit_t - earliest_entry_t)
    return expected_exit_t + max(LEASE_HARD_MIN_S, LEASE_HARD_FRACTION * span)


# ---------------------------------------------------------------------------------------------------------------
# Enumerations. String values equal the names; *_CODES map to the uint8 constants in swarmflow_interfaces.
# ---------------------------------------------------------------------------------------------------------------


class RobotMode(str, Enum):
    IDLE = "IDLE"
    NAVIGATING = "NAVIGATING"
    WAITING_RESERVATION = "WAITING_RESERVATION"
    LOADING = "LOADING"
    UNLOADING = "UNLOADING"
    STUCK = "STUCK"
    FAULT = "FAULT"


MODE_CODES: Mapping[RobotMode, int] = {m: i for i, m in enumerate(RobotMode)}  #: RobotState.MODE_* values


class OrderState(str, Enum):
    QUEUED = "QUEUED"
    ASSIGNED = "ASSIGNED"
    PICKING_UP = "PICKING_UP"
    IN_TRANSIT = "IN_TRANSIT"
    DELIVERED = "DELIVERED"
    FAILED = "FAILED"


ORDER_STATE_CODES: Mapping[OrderState, int] = {s: i for i, s in enumerate(OrderState)}  #: OrderStatus.STATE_*


class DecisionType(str, Enum):
    ASSIGN = "ASSIGN"
    REASSIGN = "REASSIGN"
    REROUTE = "REROUTE"
    PRIORITY = "PRIORITY"
    RESERVATION_GRANT = "RESERVATION_GRANT"
    RESERVATION_DENY = "RESERVATION_DENY"
    CONGESTION_RESPONSE = "CONGESTION_RESPONSE"
    DISRUPTION_RESPONSE = "DISRUPTION_RESPONSE"
    STUCK_FAIL = "STUCK_FAIL"


class LeaseState(str, Enum):
    GRANTED = "GRANTED"
    RELEASED = "RELEASED"
    EXPIRED = "EXPIRED"            #: reserved (v2); the v1 authority reports expiry as OCCUPIED_UNKNOWN
    REVOKED = "REVOKED"
    OCCUPIED_UNKNOWN = "OCCUPIED_UNKNOWN"


LEASE_STATE_CODES: Mapping[LeaseState, int] = {s: i for i, s in enumerate(LeaseState)}  #: ZoneReservation.STATE_*


class ReleaseReason(str, Enum):
    EXITED = "EXITED"
    TASK_CANCELLED = "TASK_CANCELLED"
    FAULT = "FAULT"


RELEASE_REASON_CODES: Mapping[ReleaseReason, int] = {r: i for i, r in enumerate(ReleaseReason)}


class Direction(str, Enum):
    UNSPECIFIED = "UNSPECIFIED"
    FORWARD = "FORWARD"   #: from zone entries[0] towards entries[1]
    REVERSE = "REVERSE"   #: from zone entries[1] towards entries[0]


DIRECTION_CODES: Mapping[Direction, int] = {d: i for i, d in enumerate(Direction)}

#: Failure reasons used in OrderStatus.failure_reason / DispatchTask results.
FAILURE_STUCK_TIMEOUT = "STUCK_TIMEOUT"
FAILURE_STUCK_AFTER_PICKUP = "STUCK_AFTER_PICKUP"
FAILURE_NAV_PREFIX = "NAV_FAILED:"         #: + failing vertex name (Nav2 failed twice, §6.3 item 5)
FAILURE_CANCELLED = "CANCELLED"
FAULT_ORCHESTRATOR_TIMEOUT = "ORCHESTRATOR_TIMEOUT"

#: Reservation deny reasons (ReservationDecision.reason / RequestReservation response).
DENY_UNKNOWN_ZONE = "UNKNOWN_ZONE"
DENY_BAD_ENTRY = "BAD_ENTRY"
DENY_ZONE_LEASED = "ZONE_LEASED"
DENY_OCCUPIED_UNKNOWN = "OCCUPIED_UNKNOWN"
DENY_RECOVERING = "RECOVERING"

#: Policy names (DecisionEvent.policy, orchestrator ``policy`` parameter, §13.5).
POLICY_FCFS = "fcfs"
POLICY_INDEPENDENT = "independent"
POLICY_PREDICTIVE = "predictive"
POLICY_RMF = "rmf"

# ---------------------------------------------------------------------------------------------------------------
# Graph (loaded from generated/nav_graph.yaml + generated/zones.yaml, design §6.1)
# ---------------------------------------------------------------------------------------------------------------

VertexKind = str  #: "loading" | "delivery" | "park" | "intersection" | "hold"
EdgeKey = Tuple[str, str]  #: (from_vertex_name, to_vertex_name) — edges are always referred to by name pairs


@dataclass(frozen=True)
class Vertex:
    name: str
    x: float
    y: float
    kind: VertexKind
    yaw: Optional[float] = None  #: docking yaw for stations, None elsewhere


@dataclass(frozen=True)
class Edge:
    """One direction of a lane."""

    start: str
    end: str
    length_m: float
    clear_width_m: float
    bidirectional: bool                     #: the authored lane is two-way
    zone: Optional[str] = None              #: conflict zone containing this edge
    feasible: Mapping[str, bool] = field(default_factory=dict)  #: payload type → fits (§7.2)

    @property
    def key(self) -> EdgeKey:
        return (self.start, self.end)


@dataclass(frozen=True)
class ZoneEntry:
    entry: str  #: vertex on the zone boundary
    hold: str   #: vertex outside the zone where a robot waits for its grant


@dataclass(frozen=True)
class Zone:
    name: str
    capacity: int
    edges: Tuple[EdgeKey, ...]
    vertices: Tuple[str, ...]
    entries: Tuple[ZoneEntry, ...]
    polygon: Tuple[Tuple[float, float], ...]  #: map frame, counter-clockwise


class Graph(Protocol):
    """Read-only warehouse graph. Implementation: ``swarmflow_core.graph.load_graph(nav_graph_yaml, zones_yaml)``."""

    vertices: Mapping[str, Vertex]
    edges: Mapping[EdgeKey, Edge]
    zones: Mapping[str, Zone]

    def successors(self, vertex: str) -> Sequence[str]:
        """Vertices reachable by one directed edge, sorted by name."""
        ...

    def shortest_path(self, start: str, goal: str, payload_type: str = "small") -> Sequence[str]:
        """Shortest directed route by length over payload-feasible edges, start and goal included; ties broken by
        vertex name order (deterministic). Raises ``KeyError`` for unknown vertices, ``ValueError`` if unreachable.
        Because zones can only be entered through their hold vertices, every route passes the hold before a zone."""
        ...

    def path_length(self, route: Sequence[str]) -> float:
        ...

    def zone_of_edge(self, start: str, end: str) -> Optional[str]:
        ...

    def zone_at(self, x: float, y: float) -> Optional[str]:
        """Name of the zone whose polygon contains the point (boundary counts as inside), else None."""
        ...


# ---------------------------------------------------------------------------------------------------------------
# Fleet state, orders, plans, decisions
# ---------------------------------------------------------------------------------------------------------------


@dataclass(frozen=True)
class OrderSpec:
    order_id: str
    release_t: float
    deadline_t: Optional[float]   #: None = no deadline
    pickup_vertex: str            #: e.g. "L1"
    dropoff_vertex: str           #: e.g. "D3"
    payload_type: str = "small"
    priority: int = 0             #: 0 = normal, higher = more urgent


@dataclass(frozen=True)
class RobotSnapshot:
    robot_id: str
    x: float
    y: float
    yaw: float
    mode: RobotMode
    last_vertex: str = ""          #: last graph vertex reached ("" if unknown)
    task_id: str = ""              #: "" when idle
    order_id: str = ""
    payload_type: str = ""         #: "" = unloaded
    held_lease_ids: Tuple[str, ...] = ()
    linear_speed: float = 0.0
    fault_reason: str = ""
    next_vertex: str = ""
    remaining_route: Tuple[str, ...] = ()
    eta_t: Optional[float] = None


@dataclass(frozen=True)
class FleetSnapshot:
    t: float
    robots: Sequence[RobotSnapshot]
    open_orders: Sequence[OrderSpec]          #: released, not yet assigned (QUEUED), in release order
    active_orders: Sequence[OrderSpec] = ()   #: assigned / picking up / in transit (v2 reassignment)
    leases: Sequence["Lease"] = ()            #: authority's active leases (v2 congestion-aware policies)
    closed_zones: Tuple[str, ...] = ()        #: zones removed from routing (v2 closures)


@dataclass(frozen=True)
class Assignment:
    robot_id: str
    order_id: str          #: "" for a move-only (park) task
    task_id: str
    route: Sequence[str]   #: vertex names from the robot's current vertex via pickup to dropoff, both included


@dataclass(frozen=True)
class TaskResult:
    """Outcome of a dispatched task (DispatchTask result over ROS)."""

    task_id: str
    robot_id: str
    success: bool
    failure_reason: str = ""   #: FAILURE_* constants, or FAILURE_NAV_PREFIX + vertex
    t: float = 0.0


@dataclass(frozen=True)
class OrderStatusChange:
    """One order state transition (→ /fleet/order_status, orders.jsonl)."""

    t: float
    order_id: str
    state: OrderState
    robot_id: str = ""
    failure_reason: str = ""


@dataclass(frozen=True)
class Decision:
    """A fleet decision (design §10). ``explanation`` is rendered from the other fields by
    ``swarmflow_core.decisions.render`` — never written by hand."""

    t: float
    event_id: str
    policy: str
    decision_type: DecisionType
    robot_id: str = ""
    order_id: str = ""
    trigger: str = ""
    previous_decision: str = ""
    new_decision: str = ""
    cost_keys: Tuple[str, ...] = ()
    cost_values: Tuple[float, ...] = ()
    predicted_improvement_s: float = 0.0
    explanation: str = ""


@dataclass(frozen=True)
class PlanResult:
    assignments: Sequence[Assignment]
    decisions: Sequence[Decision]


class Policy(Protocol):
    """Assignment policy. v1: ``swarmflow_core.policies.fcfs.FcfsPolicy`` (nearest idle robot, orders FIFO)."""

    name: str

    def plan(self, snapshot: FleetSnapshot, graph: Graph) -> PlanResult:
        ...


# ---------------------------------------------------------------------------------------------------------------
# Reservations (design §6.5)
# ---------------------------------------------------------------------------------------------------------------


@dataclass(frozen=True)
class ReservationRequest:
    request_id: str
    robot_id: str
    zone_id: str
    entry_vertex: str
    exit_vertex: str
    direction: Direction = Direction.UNSPECIFIED
    earliest_entry_t: float = 0.0
    expected_exit_t: float = 0.0
    priority: int = 0


@dataclass(frozen=True)
class ReservationDecision:
    granted: bool
    lease_id: str = ""           #: "" when denied
    lease_expiry_t: float = 0.0  #: current TTL expiry (renewed by heartbeats), 0 when denied
    reason: str = ""             #: deny reason: one of the DENY_* constants
    retry_after_s: float = 0.0


@dataclass(frozen=True)
class Lease:
    """Snapshot of a lease. Returned by state changes; ``state`` is the state after the change."""

    lease_id: str
    robot_id: str
    zone_id: str
    state: LeaseState
    granted_t: float
    expiry_t: float        #: TTL expiry (granted_t / last heartbeat + LEASE_TTL_S, capped by hard_expiry_t)
    hard_expiry_t: float
    reason: str = ""


@runtime_checkable
class ReservationAuthority(Protocol):
    """Sole reservation authority (orchestrator side). Safety invariant: at most ``capacity`` live leases per zone,
    and never a grant into a zone that is blocked. Implementation:
    ``swarmflow_core.reservations.FcfsReservationAuthority(graph)``.

    Lease state machine (every state change is one :class:`Lease` event in :meth:`pop_events`):

    - ``GRANTED`` on grant; renewals and idempotent re-grants produce no event.
    - TTL or hard bound passes (``t >= expiry_t``) → ``OCCUPIED_UNKNOWN``: the robot may still be inside, so the
      lease keeps **blocking** its zone (§6.5 rule 4). ``EXPIRED`` is reserved for v2 and not emitted in v1.
    - ``release`` while the robot's last observed pose (``observe_robot``) is inside the zone → ``OCCUPIED_UNKNOWN``
      (blocking); otherwise → ``RELEASED``. This also covers TASK_CANCELLED / FAULT releases mid-zone.
    - A blocking lease is cleared → ``RELEASED`` when ``observe_robot`` reports its owner outside the polygon with a
      pose stamp ``t`` not older than the moment the lease became blocking, or by a late ``release`` while the last
      observed pose is outside; ``clear_zone`` → ``REVOKED``.
    - A new authority starts **recovering**: every request is denied (``DENY_RECOVERING``) until :meth:`recover`.

    Every method first applies expiry at its ``t``. Hard bound at grant:
    ``lease_hard_expiry(max(req.earliest_entry_t, t), max(req.expected_exit_t, t))``. A request from a robot that
    already holds a GRANTED lease in the zone returns that same lease, renewed (idempotent retry after a lost
    response).
    """

    def recover(self, robots: Sequence[RobotSnapshot], t: float) -> Sequence[Lease]:
        """Orchestrator (re)start: after collecting RobotStates for ``RECOVERY_GRACE_S``, block (``OCCUPIED_UNKNOWN``,
        lease id ``recovered_<robot_id>_<zone>``) every zone that contains a robot pose or for which a robot reports
        held leases while inside it, then leave recovery mode. Returns the created leases."""
        ...

    def request(self, req: ReservationRequest, t: float) -> ReservationDecision:
        ...

    def heartbeat(self, robot_id: str, lease_ids: Sequence[str], t: float) -> None:
        """Renews the robot's listed GRANTED leases (unknown / foreign ids ignored)."""
        ...

    def release(self, robot_id: str, lease_id: str, t: float,
                reason: ReleaseReason = ReleaseReason.EXITED) -> None:
        """Releases the caller's lease per the state machine above. Unknown ids and other robots' leases are
        ignored."""
        ...

    def expire(self, t: float) -> Sequence[Lease]:
        """Leases with ``t >= expiry_t`` become blocking ``OCCUPIED_UNKNOWN``. Returns the changed leases."""
        ...

    def observe_robot(self, robot_id: str, x: float, y: float, t: float) -> Sequence[Lease]:
        """Feeds a robot pose (from RobotState); ``t`` is the **pose stamp**. Remembers the latest pose per robot.
        Clears this robot's blocking leases whose zone polygon does not contain the pose (see the state machine).
        Returns the changed leases."""
        ...

    def clear_zone(self, zone_id: str, reason: str, t: float) -> bool:
        """Operator clear (/fleet/clear_zone): revokes all leases in the zone. Returns False for an unknown zone."""
        ...

    def active_leases(self) -> Sequence[Lease]:
        """Leases currently GRANTED or blocking (OCCUPIED_UNKNOWN), sorted by lease_id."""
        ...

    def pop_events(self) -> Sequence[Lease]:
        """All lease state changes since the last call, in order (→ /fleet/reservations broadcasts)."""
        ...


# ---------------------------------------------------------------------------------------------------------------
# Backends (ROS adapter, fake backend, 2D sim)
# ---------------------------------------------------------------------------------------------------------------


class Backend(Protocol):
    def snapshot(self) -> FleetSnapshot:
        ...

    def dispatch(self, assignment: Assignment) -> None:
        ...

    def cancel(self, task_id: str) -> None:
        ...

    def poll_results(self) -> Sequence[TaskResult]:
        """Task results since the last call, in completion order."""
        ...


__all__ = [n for n in dir() if not n.startswith("_") and n not in {"annotations", "dataclass", "field", "Enum",
                                                                   "Mapping", "Optional", "Protocol", "Sequence",
                                                                   "Tuple", "runtime_checkable"}]
