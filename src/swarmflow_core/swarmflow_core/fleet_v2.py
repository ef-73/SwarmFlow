"""Assignment-only fleet core for v2 (improvements_v2 O-01 ... O-05, O-08, P-01).

Same surface as :class:`swarmflow_core.fleet.FleetCore` (``add_order``, ``update_robot``, ``task_result``, ``tick``,
``order_state``, ``orders``) but it only decides *which robot does which order* and the ordered goal list. Every
``Assignment`` it emits is a goal task (``api.ROUTE_GOALS``); no reservations are involved. The lifecycle follows the
``swarmflow_core.api`` module docstring (v2 section). Pure Python, deterministic, time passed in.

Design notes
- Dock holding (O-05): ``_holder[dock]`` is the order whose last goal is the dock, from its dispatch until the result
  of that task (a follow-up task of the same order keeps the hold). A new route for a held dock ends at the lowest
  free queue spot instead (``_spot_owner``); no free spot means the order simply stays QUEUED. When a dock frees, the
  queue owners in index order each move up (first -> dock, k-th remaining -> k-th lowest spot); an order whose target
  changes has its running task cancelled (``TickOutput.cancels``) and a follow-up dispatched in the same tick.
  Task ids the core cancelled itself are remembered; their results change nothing.
- A task that ends at a queue spot never delivers; the order is promoted to IN_TRANSIT and waits there (no home).
- Order states only move forward, except a requeue after a pre-pickup failure (REASSIGN back to QUEUED, api).
  Missing intermediate states are published before a later one (ASSIGNED -> IN_TRANSIT also emits PICKING_UP).
- Failures: STUCK_TIMEOUT -> FAILED. Other failures before pickup requeue the order and send the robot home; after
  pickup (PICKING_UP / IN_TRANSIT) the remaining goals are re-dispatched, counted in ``retries`` against
  ``api.MAX_LOADED_RETRIES``, then FAILED (STUCK_AFTER_PICKUP) and the robot goes home.
- Blocked (STUCK + FAULT_BLOCKED): the timer starts at the first tick that sees it and is cleared when the robot is no
  longer reported blocked on its current task. After ``BLOCKED_REQUEUE_S`` the task is cancelled and the order is
  requeued (robot home) or re-dispatched, as for a failure. Blocked robots get no v1 stuck-recovery park tasks.
- Home rule: an available robot not on a ``home`` vertex gets a goal task to the nearest free home slot (no robot
  there, not the target of another home task, not a slot that already failed for this robot); robots in id order.
"""

from __future__ import annotations

import dataclasses
import math
import re
from dataclasses import dataclass, field
from typing import Dict, List, Mapping, Optional, Set, Tuple

from . import api
from .api import (Assignment, DecisionType, FleetSnapshot, OrderSpec, OrderState, OrderStatusChange, RobotMode,
                  RobotSnapshot, TaskResult)
from .decisions import make_decision
from .fleet import TickOutput

_FORWARD = [OrderState.QUEUED, OrderState.ASSIGNED, OrderState.PICKING_UP, OrderState.IN_TRANSIT,
            OrderState.DELIVERED]
_RANK = {s: i for i, s in enumerate(_FORWARD)}
_ACTIVE = (OrderState.ASSIGNED, OrderState.PICKING_UP, OrderState.IN_TRANSIT)
BAD_HOME_TTL_S = 120.0   #: a failed home slot is avoided for this long (cleared earlier by any success)
_QUEUE_NAME = re.compile(r"Q_(.+)_(\d+)")


@dataclass
class _Order:
    spec: OrderSpec
    state: OrderState = OrderState.QUEUED
    robot_id: str = ""
    task_id: str = ""                 #: live task ("" while waiting for a follow-up or at a queue spot)
    attempts: int = 0                 #: requeues
    retries: int = 0                  #: follow-ups after failures / blocked while loaded
    n_tasks: int = 0
    picks_left: List[str] = field(default_factory=list)   #: goals before the target not yet reached
    target: str = ""                  #: final goal of the current/next task: the dock or a queue spot
    followup: bool = False
    followup_reason: str = ""


@dataclass
class _Task:
    task_id: str
    robot_id: str
    order_id: str                     #: "" for a home task
    goals: Tuple[str, ...] = ()
    reached: int = 0                  #: leading goals known reached
    baseline: str = ""                #: robot last_vertex at dispatch; goals are credited only after it changes
    moved: bool = False


class FleetCoreV2:
    def __init__(self, graph, policy, queue_spots: Optional[Mapping[str, Tuple[str, int]]] = None):
        self.graph = graph
        self.policy = policy
        if queue_spots is None:
            queue_spots = {}
            for name in graph.vertices_of_kind("queue"):
                m = _QUEUE_NAME.fullmatch(name)
                if m:
                    queue_spots[name] = (m.group(1), int(m.group(2)))
        self._spots: Dict[str, List[Tuple[int, str]]] = {}
        for vertex, (dock, idx) in queue_spots.items():
            self._spots.setdefault(dock, []).append((idx, vertex))
        for lst in self._spots.values():
            lst.sort()
        self._pending: List[OrderSpec] = []
        self._orders: Dict[str, _Order] = {}
        self._robots: Dict[str, RobotSnapshot] = {}
        self._results: List[TaskResult] = []
        self._tasks: Dict[str, _Task] = {}
        self._task_robot: Dict[str, str] = {}
        self._cancelled: Dict[str, Tuple[float, str]] = {}               # tasks the core cancelled itself -> time
        self._now = 0.0
        self._down_since: Dict[str, float] = {}            # robot -> first tick seen FAULT/STUCK while waiting
        self._order_of_robot: Dict[str, str] = {}          # robot -> its active order
        self._home_task_of_robot: Dict[str, str] = {}
        self._home_target: Dict[str, str] = {}             # home task id -> slot
        self._home_seq: Dict[str, int] = {}
        self._bad_home: Dict[str, Dict[str, float]] = {}    # robot -> slot -> time its home task failed
        self._holder: Dict[str, str] = {}                  # dock -> order
        self._spot_owner: Dict[str, str] = {}              # queue spot -> order
        self._blocked_since: Dict[str, float] = {}

    # ---- inputs -----------------------------------------------------------------------------------------------

    def add_order(self, order: OrderSpec) -> None:
        if order.order_type not in (api.ORDER_SHELF, api.ORDER_PALLET):
            raise ValueError(f"FleetCoreV2 supports shelf and pallet orders, got {order.order_type!r}")
        if order.order_id in self._orders or any(o.order_id == order.order_id for o in self._pending):
            raise ValueError(f"duplicate order id {order.order_id!r}")
        self._pending.append(order)
        self._pending.sort(key=lambda o: (o.release_t, o.order_id))

    def update_robot(self, snapshot: RobotSnapshot) -> None:
        self._robots[snapshot.robot_id] = snapshot

    def task_result(self, task_id: str, success: bool, failure_reason: str, t: float) -> None:
        robot = self._task_robot.get(task_id, "")
        self._results.append(TaskResult(task_id, robot, success, failure_reason, t))

    def task_result_obj(self, result: TaskResult) -> None:
        self._results.append(result)

    # ---- queries ----------------------------------------------------------------------------------------------

    def order_state(self, order_id: str) -> Optional[OrderState]:
        rec = self._orders.get(order_id)
        return rec.state if rec else None

    def orders(self) -> Dict[str, OrderState]:
        return {oid: rec.state for oid, rec in self._orders.items()}

    # ---- helpers ----------------------------------------------------------------------------------------------

    def _near(self, s: RobotSnapshot, vertex: str) -> bool:
        v = self.graph.vertices.get(vertex)
        return v is not None and math.hypot(s.x - v.x, s.y - v.y) <= api.GOAL_TOLERANCE_M

    def _vertex(self, s: RobotSnapshot) -> str:
        """Start vertex for path distance: last_vertex only while the pose confirms it (it can be stale)."""
        if s.last_vertex and self._near(s, s.last_vertex):
            return s.last_vertex
        return self.graph.nearest_vertex(s.x, s.y)

    def _at_home(self, s: RobotSnapshot) -> bool:
        return any(self._near(s, h) for h in self.graph.vertices_of_kind("home"))

    def _available(self, s: RobotSnapshot) -> bool:
        if s.mode in (RobotMode.STUCK, RobotMode.FAULT):
            return False
        if s.robot_id in self._order_of_robot:
            return False
        home = self._home_task_of_robot.get(s.robot_id)
        if s.task_id == "":
            return s.mode == RobotMode.IDLE or home is not None
        return home is not None and s.task_id == home

    def _change(self, out: TickOutput, t: float, rec: _Order, state: OrderState, reason: str = "") -> None:
        rec.state = state
        out.status_changes.append(OrderStatusChange(t, rec.spec.order_id, state, rec.robot_id, reason))

    def _advance(self, out: TickOutput, t: float, rec: _Order, state: OrderState) -> None:
        """Forward-only: publish every state between the current one and ``state``."""
        if rec.state not in _RANK or state not in _RANK or _RANK[state] <= _RANK[rec.state]:
            return
        for s in _FORWARD[_RANK[rec.state] + 1:_RANK[state] + 1]:
            self._change(out, t, rec, s)

    @staticmethod
    def _loaded(rec: _Order) -> bool:
        return rec.state in (OrderState.PICKING_UP, OrderState.IN_TRANSIT)

    def _release_claims(self, rec: _Order) -> None:
        oid = rec.spec.order_id
        for dock in [d for d, o in self._holder.items() if o == oid]:
            del self._holder[dock]
        for spot in [v for v, o in self._spot_owner.items() if o == oid]:
            del self._spot_owner[spot]
        rec.target = ""

    def _drop_robot(self, rec: _Order) -> None:
        self._order_of_robot.pop(rec.robot_id, None)
        self._blocked_since.pop(rec.robot_id, None)

    def _cancel_task(self, out: TickOutput, rec: _Order) -> None:
        if rec.task_id:
            self._tasks.pop(rec.task_id, None)
            self._cancelled[rec.task_id] = (self._now, rec.robot_id)
            self._task_robot.pop(rec.task_id, None)
            out.cancels.append(rec.task_id)
            rec.task_id = ""
        self._blocked_since.pop(rec.robot_id, None)

    def _flag_followup(self, rec: _Order, reason: str) -> None:
        rec.followup, rec.followup_reason = True, reason

    def _next_task_id(self, rec: _Order) -> str:
        n = rec.n_tasks
        rec.n_tasks += 1
        base = f"task_{rec.spec.order_id}"
        return base if n == 0 else f"{base}_{n}"

    def _requeue(self, out: TickOutput, t: float, rec: _Order, trigger: str, forced_home: Set[str],
                 reason: str = "") -> None:
        if rec.attempts >= api.MAX_LOADED_RETRIES:                 # requeues are capped like loaded retries
            self._fail(out, t, rec, reason or api.FAULT_BLOCKED, trigger, forced_home)
            return
        robot = rec.robot_id
        self._release_claims(rec)
        self._drop_robot(rec)
        forced_home.add(robot)
        rec.robot_id, rec.task_id, rec.picks_left = "", "", []
        rec.followup = False
        rec.attempts += 1
        self._change(out, t, rec, OrderState.QUEUED)
        out.decisions.append(make_decision(
            t=t, event_id=f"{t:.3f}-{rec.spec.order_id}-REASSIGN", policy=self.policy.name,
            decision_type=DecisionType.REASSIGN, robot_id=robot, order_id=rec.spec.order_id,
            trigger=trigger, previous_decision=f"{robot} -> {rec.spec.order_id}", new_decision="requeued"))

    def _fail(self, out: TickOutput, t: float, rec: _Order, reason: str, trigger: str,
              forced_home: Set[str]) -> None:
        robot = rec.robot_id
        self._change(out, t, rec, OrderState.FAILED, reason)
        out.decisions.append(make_decision(
            t=t, event_id=f"{t:.3f}-{rec.spec.order_id}-STUCK_FAIL", policy=self.policy.name,
            decision_type=DecisionType.STUCK_FAIL, robot_id=robot, order_id=rec.spec.order_id,
            trigger=trigger, previous_decision=f"{robot} -> {rec.spec.order_id}", new_decision=f"failed: {reason}"))
        self._release_claims(rec)
        self._drop_robot(rec)
        rec.task_id, rec.followup = "", False
        forced_home.add(robot)

    def _retry_or_fail(self, out: TickOutput, t: float, rec: _Order, reason: str, trigger: str,
                       forced_home: Set[str]) -> None:
        """Loaded robot: re-dispatch the remaining goals, up to MAX_LOADED_RETRIES times, then FAILED."""
        if rec.retries < api.MAX_LOADED_RETRIES:
            rec.retries += 1
            self._flag_followup(rec, trigger)
        else:
            self._fail(out, t, rec, api.FAILURE_STUCK_AFTER_PICKUP, trigger, forced_home)

    # ---- tick -------------------------------------------------------------------------------------------------

    def tick(self, t: float) -> TickOutput:
        out = TickOutput()
        self._now = t
        for tid, (ts, r) in list(self._cancelled.items()):              # prune our own cancels
            s = self._robots.get(r)
            if t - ts > 60.0 or (s is not None and s.task_id != tid):
                del self._cancelled[tid]
        forced_home: Set[str] = set()
        while self._pending and self._pending[0].release_t <= t:       # (1) release
            spec = self._pending.pop(0)
            rec = _Order(spec)
            self._orders[spec.order_id] = rec
            self._change(out, t, rec, OrderState.QUEUED)
        self._observe(out, t)                                          # (2) snapshot transitions
        self._results_step(out, t, forced_home)                        # (3) task results
        self._blocked_step(out, t, forced_home)                        # (4) blocked robots
        self._advance_queues(out)                                      # (5) dock freed -> queue moves up
        self._followups(out, t)                                        # (6) follow-up tasks
        self._plan(out, t, forced_home)                                # (7) new assignments
        self._home_step(out, t, forced_home)                           # (8) home slots
        return out

    # ---- (2) robot snapshots ------------------------------------------------------------------------------------

    def _observe(self, out: TickOutput, t: float) -> None:
        for oid in sorted(self._orders):
            rec = self._orders[oid]
            if rec.state not in _ACTIVE or not rec.task_id:
                continue
            s = self._robots.get(rec.robot_id)
            task = self._tasks.get(rec.task_id)
            if s is None or task is None or s.task_id != rec.task_id or s.order_id != oid:
                continue
            # last_vertex survives between tasks (api), so credit a goal only after it moved away from the dispatch-time
            # baseline, in goal order, and never the final goal (the task result covers it)
            if s.last_vertex != task.baseline:
                task.moved = True
            if task.moved and task.reached < len(task.goals) - 1 and s.last_vertex == task.goals[task.reached]:
                task.reached += 1
            if rec.spec.order_type == api.ORDER_SHELF:
                rec.picks_left = list(task.goals[task.reached:-1])
                if len(rec.picks_left) < len(rec.spec.pick_vertices):
                    self._advance(out, t, rec, OrderState.PICKING_UP)
                if not rec.picks_left:
                    self._advance(out, t, rec, OrderState.IN_TRANSIT)
            else:
                if s.mode == RobotMode.LIFTING and rec.state == OrderState.ASSIGNED:
                    self._advance(out, t, rec, OrderState.PICKING_UP)
                if s.mode == RobotMode.CARRYING:
                    self._advance(out, t, rec, OrderState.IN_TRANSIT)
                if rec.state == OrderState.IN_TRANSIT:
                    rec.picks_left = []

    # ---- (3) task results -----------------------------------------------------------------------------------------

    def _results_step(self, out: TickOutput, t: float, forced_home: Set[str]) -> None:
        results, self._results = self._results, []
        for res in results:
            if res.task_id in self._cancelled:
                # our own cancel: changes nothing
                self._cancelled.pop(res.task_id, None)
                self._task_robot.pop(res.task_id, None)
                continue
            task = self._tasks.pop(res.task_id, None)
            if task is None:
                continue
            self._task_robot.pop(res.task_id, None)
            if res.success:
                self._bad_home.pop(task.robot_id, None)
            if task.order_id == "":
                if self._home_task_of_robot.get(task.robot_id) == task.task_id:
                    del self._home_task_of_robot[task.robot_id]
                target = self._home_target.pop(task.task_id, "")
                if not res.success and target:
                    self._bad_home.setdefault(task.robot_id, {})[target] = t
                continue
            rec = self._orders[task.order_id]
            if rec.task_id != task.task_id:
                continue
            rec.task_id = ""
            self._blocked_since.pop(task.robot_id, None)
            if res.success:
                task.reached = len(task.goals)
                rec.picks_left = []
                self._advance(out, t, rec, OrderState.IN_TRANSIT)
                if task.goals[-1] == rec.spec.dropoff_vertex:
                    self._advance(out, t, rec, OrderState.DELIVERED)
                    self._release_claims(rec)
                    self._drop_robot(rec)
                continue
            reason = res.failure_reason
            trigger = f"task_failed:{reason}"
            if reason == api.FAILURE_STUCK_TIMEOUT:
                self._fail(out, t, rec, api.FAILURE_STUCK_TIMEOUT, trigger, forced_home)
            elif self._loaded(rec):
                self._retry_or_fail(out, t, rec, reason, trigger, forced_home)
            else:
                self._requeue(out, t, rec, trigger, forced_home, reason)

    # ---- (4) blocked robots ---------------------------------------------------------------------------------------

    def _blocked_step(self, out: TickOutput, t: float, forced_home: Set[str]) -> None:
        for rid in sorted(self._robots):
            s = self._robots[rid]
            oid = self._order_of_robot.get(rid)
            rec = self._orders.get(oid) if oid else None
            if rec is not None and rec.task_id == "" and not rec.followup and s.mode in (RobotMode.FAULT,
                                                                                        RobotMode.STUCK):
                # waiting robot (e.g. at a queue spot) that is down: do not hold the dock forever
                since = self._down_since.setdefault(rid, t)
                if t - since >= api.BLOCKED_REQUEUE_S:
                    del self._down_since[rid]
                    trigger = f"robot_down:{s.mode.value}"
                    if self._loaded(rec):
                        self._fail(out, t, rec, api.FAILURE_STUCK_AFTER_PICKUP, trigger, set())
                    else:
                        self._requeue(out, t, rec, trigger, set(), s.fault_reason or s.mode.value)
                continue
            self._down_since.pop(rid, None)
            blocked = (rec is not None and rec.task_id != "" and s.task_id == rec.task_id and s.order_id == oid
                       and s.mode == RobotMode.STUCK and s.fault_reason == api.FAULT_BLOCKED)
            if not blocked:
                self._blocked_since.pop(rid, None)
                continue
            since = self._blocked_since.setdefault(rid, t)
            if t - since < api.BLOCKED_REQUEUE_S:
                continue
            trigger = f"blocked:{s.next_vertex or rec.target}"
            self._cancel_task(out, rec)
            if self._loaded(rec):
                self._retry_or_fail(out, t, rec, api.FAULT_BLOCKED, trigger, forced_home)
            else:
                self._requeue(out, t, rec, trigger, forced_home, api.FAULT_BLOCKED)

    # ---- (5) dock queues ------------------------------------------------------------------------------------------

    def _advance_queues(self, out: TickOutput) -> None:
        for dock in sorted(self._spots):
            if dock in self._holder:
                continue
            spots = [v for _, v in self._spots[dock]]
            waiting = [(v, self._spot_owner[v]) for v in spots if v in self._spot_owner]
            if not waiting:
                continue
            for v, _ in waiting:
                del self._spot_owner[v]
            self._holder[dock] = waiting[0][1]
            moves = [(waiting[0][1], dock)]
            for k, (_, oid) in enumerate(waiting[1:]):
                self._spot_owner[spots[k]] = oid
                moves.append((oid, spots[k]))
            for oid, target in moves:
                rec = self._orders[oid]
                if rec.target == target:
                    continue
                rec.target = target
                self._cancel_task(out, rec)
                self._flag_followup(rec, "queue_advance")

    # ---- (6) follow-up tasks --------------------------------------------------------------------------------------

    def _followups(self, out: TickOutput, t: float) -> None:
        for oid in sorted(self._orders):
            rec = self._orders[oid]
            if not rec.followup or rec.state not in _ACTIVE:
                continue
            rec.followup = False
            goals = tuple(rec.picks_left) + (rec.target,)
            task_id = self._start_task(rec, goals)
            out.dispatches.append(Assignment(rec.robot_id, oid, task_id, goals, api.ROUTE_GOALS))
            out.decisions.append(make_decision(
                t=t, event_id=f"{t:.3f}-{oid}-REROUTE-{task_id}", policy=self.policy.name,
                decision_type=DecisionType.REROUTE, robot_id=rec.robot_id, order_id=oid,
                trigger=rec.followup_reason, previous_decision=f"{rec.robot_id} task ended",
                new_decision=f"goals {' > '.join(goals)}"))

    def _start_task(self, rec: _Order, goals: Tuple[str, ...]) -> str:
        task_id = self._next_task_id(rec)
        snap = self._robots.get(rec.robot_id)
        self._tasks[task_id] = _Task(task_id, rec.robot_id, rec.spec.order_id, goals,
                                     baseline=snap.last_vertex if snap else "")
        self._task_robot[task_id] = rec.robot_id
        rec.task_id = task_id
        self._blocked_since.pop(rec.robot_id, None)
        return task_id

    # ---- (7) planning ---------------------------------------------------------------------------------------------

    def _dock_has_room(self, dock: str) -> bool:
        if dock not in self._holder:
            return True
        return any(v not in self._spot_owner for _, v in self._spots.get(dock, ()))

    def _claim_dock(self, oid: str, dock: str) -> Optional[str]:
        if dock not in self._holder:
            self._holder[dock] = oid
            return dock
        for _, v in self._spots.get(dock, ()):
            if v not in self._spot_owner:
                self._spot_owner[v] = oid
                return v
        return None

    def _plan(self, out: TickOutput, t: float, forced_home: Set[str]) -> None:
        avail = {rid for rid, s in self._robots.items() if rid not in forced_home and self._available(s)}
        queued = [rec.spec for rec in self._orders.values()
                  if rec.state == OrderState.QUEUED and self._dock_has_room(rec.spec.dropoff_vertex)]
        if not avail or not queued:
            return
        robots = []
        for rid in sorted(self._robots):
            s = self._robots[rid]
            if rid in avail:
                robots.append(dataclasses.replace(s, mode=RobotMode.IDLE, task_id="", order_id="",
                                                  last_vertex=self._vertex(s), next_vertex=""))
            elif s.mode == RobotMode.IDLE and s.task_id == "" and s.order_id == "":
                robots.append(dataclasses.replace(s, order_id=self._order_of_robot.get(rid, "_unavailable")))
            else:
                robots.append(s)
        plan = self.policy.plan(FleetSnapshot(t=t, robots=robots, open_orders=queued), self.graph)
        dropped: Set[str] = set()
        for a in plan.assignments:
            rec = self._orders[a.order_id]
            if rec.state != OrderState.QUEUED or a.robot_id not in avail or a.robot_id in self._order_of_robot:
                dropped.add(a.order_id)
                continue
            goals = list(a.route)
            if goals[-1] == rec.spec.dropoff_vertex:
                target = self._claim_dock(a.order_id, goals[-1])
                if target is None:                                     # dock held and every queue spot taken
                    dropped.add(a.order_id)
                    continue
                goals[-1] = target
            else:
                target = goals[-1]
            home = self._home_task_of_robot.pop(a.robot_id, None)
            if home is not None:
                self._tasks.pop(home, None)
                self._home_target.pop(home, None)
                self._cancelled[home] = (self._now, a.robot_id)
                out.cancels.append(home)
            rec.robot_id, rec.target, rec.retries = a.robot_id, target, 0
            rec.picks_left = goals[:-1]
            self._order_of_robot[a.robot_id] = a.order_id
            task_id = self._start_task(rec, tuple(goals))
            self._change(out, t, rec, OrderState.ASSIGNED)
            out.dispatches.append(Assignment(a.robot_id, a.order_id, task_id, tuple(goals), api.ROUTE_GOALS))
        out.decisions.extend(d for d in plan.decisions if d.order_id not in dropped)

    # ---- (8) home slots -------------------------------------------------------------------------------------------

    def _issue_home(self, out: TickOutput, robot: RobotSnapshot, t: float) -> bool:
        rid = robot.robot_id
        taken = {h for h in self.graph.vertices_of_kind("home")
                 if any(self._near(s, h) for r, s in self._robots.items() if r != rid)}
        taken |= {self._home_target[tid] for r, tid in self._home_task_of_robot.items()
                  if r != rid and tid in self._home_target}
        bad = {v: ts for v, ts in self._bad_home.get(rid, {}).items() if t - ts <= BAD_HOME_TTL_S}
        if bad:
            self._bad_home[rid] = bad
        else:
            self._bad_home.pop(rid, None)
        taken |= set(bad)
        start = self._vertex(robot)
        best = None
        for slot in self.graph.vertices_of_kind("home"):
            if slot in taken:
                continue
            try:
                d = self.graph.path_length(self.graph.shortest_path(start, slot, robot.payload_type or "unloaded"))
            except (KeyError, ValueError):
                continue
            if best is None or (d, slot) < best:
                best = (d, slot)
        if best is None:
            return False
        old = self._home_task_of_robot.pop(rid, None)
        if old is not None:
            self._tasks.pop(old, None)
            self._home_target.pop(old, None)
            self._cancelled[old] = (self._now, rid)
            out.cancels.append(old)
        n = self._home_seq.get(rid, 0) + 1
        self._home_seq[rid] = n
        task_id = f"home_{rid}_{n}"
        self._tasks[task_id] = _Task(task_id, rid, "", (best[1],))
        self._task_robot[task_id] = rid
        self._home_task_of_robot[rid] = task_id
        self._home_target[task_id] = best[1]
        out.dispatches.append(Assignment(rid, "", task_id, (best[1],), api.ROUTE_GOALS))
        return True

    def _home_step(self, out: TickOutput, t: float, forced_home: Set[str]) -> None:
        for rid in sorted(self._robots):
            s = self._robots[rid]
            if rid in forced_home:
                self._issue_home(out, s, t)
                continue
            if not self._available(s) or rid in self._home_task_of_robot:
                continue
            if self._at_home(s):
                continue
            self._issue_home(out, s, t)
