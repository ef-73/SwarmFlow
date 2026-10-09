"""Fleet core: order book + assignment policy + stuck/park rules (design §6.7, §13.5).

Pure Python, deterministic, time passed in. The order lifecycle follows the ``swarmflow_core.api`` module docstring.

Station claims (M6 finding: two robots sent to one loading station met there and gridlocked): a QUEUED order is
only planned if its pickup station is not claimed by another order that is ASSIGNED or PICKING_UP, and its dropoff
station is not claimed by another order that is ASSIGNED, PICKING_UP or IN_TRANSIT. Orders admitted in a tick claim
their stations for the rest of that tick (older order wins). Blocked orders are skipped, not blocking others.
"""

from __future__ import annotations

import dataclasses
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Set

from . import api
from .api import (Assignment, Decision, DecisionType, FleetSnapshot, OrderSpec, OrderState, OrderStatusChange,
                  RobotMode, RobotSnapshot, TaskResult)
from .decisions import make_decision


@dataclass
class TickOutput:
    dispatches: List[Assignment] = field(default_factory=list)
    cancels: List[str] = field(default_factory=list)          #: task ids to cancel (preempted park tasks)
    status_changes: List[OrderStatusChange] = field(default_factory=list)
    decisions: List[Decision] = field(default_factory=list)


@dataclass
class _Order:
    spec: OrderSpec
    state: OrderState = OrderState.QUEUED
    robot_id: str = ""
    task_id: str = ""
    attempts: int = 0
    picked_up: bool = False


@dataclass
class _Task:
    task_id: str
    robot_id: str
    order_id: str          #: "" for a park task
    target: str = ""       #: park vertex for park tasks


class FleetCore:
    def __init__(self, graph, policy, authority, traffic_control: bool = True):
        self.graph = graph
        self.policy = policy
        self.authority = authority
        self.traffic_control = traffic_control
        self._pending: List[OrderSpec] = []
        self._orders: Dict[str, _Order] = {}
        self._robots: Dict[str, RobotSnapshot] = {}
        self._results: List[TaskResult] = []
        self._tasks: Dict[str, _Task] = {}
        self._order_task_of_robot: Dict[str, str] = {}   # robot -> outstanding order task id
        self._park_task_of_robot: Dict[str, str] = {}    # robot -> running park task id
        self._park_done: Set[str] = set()                # robots already given a park task for this stay
        self._park_seq: Dict[str, int] = {}

    # ---- inputs -----------------------------------------------------------------------------------------------

    def add_order(self, order: OrderSpec) -> None:
        self._pending.append(order)
        self._pending.sort(key=lambda o: (o.release_t, o.order_id))

    def update_robot(self, snapshot: RobotSnapshot) -> None:
        self._robots[snapshot.robot_id] = snapshot

    def task_result(self, task_id: str, success: bool, failure_reason: str, t: float) -> None:
        robot = self._tasks[task_id].robot_id if task_id in self._tasks else ""
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

    def _vertex(self, s: RobotSnapshot) -> str:
        return s.last_vertex or self.graph.nearest_vertex(s.x, s.y)

    def _available(self, s: RobotSnapshot) -> bool:
        if s.mode in (RobotMode.STUCK, RobotMode.FAULT):
            return False
        if s.robot_id in self._order_task_of_robot:
            return False
        park = self._park_task_of_robot.get(s.robot_id)
        if s.task_id == "":
            return s.mode == RobotMode.IDLE or park is not None
        return park is not None and s.task_id == park

    def _change(self, out: TickOutput, t: float, rec: _Order, state: OrderState, reason: str = "") -> None:
        rec.state = state
        out.status_changes.append(OrderStatusChange(t, rec.spec.order_id, state, rec.robot_id, reason))

    def _park_targets(self, exclude_robot: str) -> Set[str]:
        return {self._tasks[tid].target for rid, tid in self._park_task_of_robot.items() if rid != exclude_robot}

    def _issue_park(self, out: TickOutput, robot: RobotSnapshot) -> bool:
        taken = self._park_targets(robot.robot_id)
        taken |= {self._vertex(s) for rid, s in self._robots.items() if rid != robot.robot_id}
        start = self._vertex(robot)
        best = None
        for p in self.graph.vertices_of_kind("park"):
            if p in taken:
                continue
            try:
                route = list(self.graph.shortest_path(start, p, robot.payload_type or "unloaded"))
            except (KeyError, ValueError):
                continue
            d = self.graph.path_length(route)
            if best is None or d < best[0]:
                best = (d, p, route)
        if best is None:
            return False
        _, target, route = best
        n = self._park_seq.get(robot.robot_id, 0) + 1
        self._park_seq[robot.robot_id] = n
        task_id = f"park_{robot.robot_id}_{n}"
        self._tasks[task_id] = _Task(task_id, robot.robot_id, "", target)
        self._park_task_of_robot[robot.robot_id] = task_id
        self._park_done.add(robot.robot_id)
        out.dispatches.append(Assignment(robot.robot_id, "", task_id, tuple(route)))
        return True

    def _claim_filter(self) -> List[OrderSpec]:
        """QUEUED orders (FIFO) whose stations are free; admitted orders claim their stations for the tick."""
        pick: Set[str] = set()
        drop: Set[str] = set()
        for r in self._orders.values():
            if r.state in (OrderState.ASSIGNED, OrderState.PICKING_UP):
                pick.add(r.spec.pickup_vertex)
                drop.add(r.spec.dropoff_vertex)
            elif r.state == OrderState.IN_TRANSIT:
                drop.add(r.spec.dropoff_vertex)
        ok: List[OrderSpec] = []
        for r in self._orders.values():
            if r.state != OrderState.QUEUED:
                continue
            sp = r.spec
            if sp.pickup_vertex in pick or sp.dropoff_vertex in drop:
                continue
            pick.add(sp.pickup_vertex)
            drop.add(sp.dropoff_vertex)
            ok.append(sp)
        return ok

    # ---- tick -------------------------------------------------------------------------------------------------

    def tick(self, t: float) -> TickOutput:
        out = TickOutput()
        # (1) release
        while self._pending and self._pending[0].release_t <= t:
            spec = self._pending.pop(0)
            rec = _Order(spec)
            self._orders[spec.order_id] = rec
            self._change(out, t, rec, OrderState.QUEUED)
        # (2a) robot snapshot transitions
        for oid in sorted(self._orders):
            rec = self._orders[oid]
            snap = self._robots.get(rec.robot_id)
            if (rec.state not in (OrderState.ASSIGNED, OrderState.PICKING_UP) or snap is None
                    or snap.task_id != rec.task_id or snap.order_id != oid):
                continue
            if rec.state == OrderState.ASSIGNED:
                if snap.mode == RobotMode.LOADING:
                    rec.picked_up = True
                    self._change(out, t, rec, OrderState.PICKING_UP)
                elif snap.payload_type != "" or snap.mode == RobotMode.UNLOADING:  # LOADING was never observed
                    rec.picked_up = True
                    self._change(out, t, rec, OrderState.PICKING_UP)
                    self._change(out, t, rec, OrderState.IN_TRANSIT)
            elif snap.mode != RobotMode.LOADING:
                self._change(out, t, rec, OrderState.IN_TRANSIT)
        # (2b)+(3) task results
        results, self._results = self._results, []
        forced_park: Set[str] = set()
        for res in results:
            task = self._tasks.pop(res.task_id, None)
            if task is None:
                continue
            if task.order_id == "":
                if self._park_task_of_robot.get(task.robot_id) == task.task_id:
                    del self._park_task_of_robot[task.robot_id]
                continue
            rec = self._orders[task.order_id]
            if self._order_task_of_robot.get(task.robot_id) == task.task_id:
                del self._order_task_of_robot[task.robot_id]
            if res.success:
                self._change(out, t, rec, OrderState.DELIVERED)
                continue
            reason = res.failure_reason
            prev = f"{task.robot_id} -> {task.order_id}"
            trigger = f"task_failed:{reason}"
            if reason == api.FAILURE_STUCK_TIMEOUT or rec.picked_up:
                fail = reason if reason == api.FAILURE_STUCK_TIMEOUT else api.FAILURE_STUCK_AFTER_PICKUP
                self._change(out, t, rec, OrderState.FAILED, fail)
                out.decisions.append(make_decision(
                    t=t, event_id=f"{t:.3f}-{task.order_id}-STUCK_FAIL", policy=self.policy.name,
                    decision_type=DecisionType.STUCK_FAIL, robot_id=task.robot_id, order_id=task.order_id,
                    trigger=trigger, previous_decision=prev, new_decision=f"failed: {fail}"))
            else:
                rec.robot_id, rec.task_id = "", ""
                rec.attempts += 1
                self._change(out, t, rec, OrderState.QUEUED)
                out.decisions.append(make_decision(
                    t=t, event_id=f"{t:.3f}-{task.order_id}-REASSIGN", policy=self.policy.name,
                    decision_type=DecisionType.REASSIGN, robot_id=task.robot_id, order_id=task.order_id,
                    trigger=trigger, previous_decision=prev, new_decision="requeued"))
            robot = self._robots.get(task.robot_id)
            if robot is not None:
                forced_park.add(task.robot_id)
                if task.robot_id in self._park_task_of_robot:  # keep at most one park task per robot
                    del self._park_task_of_robot[task.robot_id]
                self._issue_park(out, robot)
        # (4) plan
        avail = [s for rid, s in sorted(self._robots.items()) if rid not in forced_park and self._available(s)]
        queued = self._claim_filter()
        if avail and queued:
            idle = [dataclasses.replace(s, mode=RobotMode.IDLE, task_id="", order_id="",
                                        last_vertex=self._vertex(s)) for s in avail]
            plan = self.policy.plan(FleetSnapshot(t=t, robots=idle, open_orders=queued), self.graph)
            for a in plan.assignments:
                rec = self._orders[a.order_id]
                if rec.attempts:
                    a = dataclasses.replace(a, task_id=f"{a.task_id}_{rec.attempts}")
                park = self._park_task_of_robot.pop(a.robot_id, None)
                if park is not None:
                    self._tasks.pop(park, None)
                    out.cancels.append(park)
                self._tasks[a.task_id] = _Task(a.task_id, a.robot_id, a.order_id)
                self._order_task_of_robot[a.robot_id] = a.task_id
                self._park_done.discard(a.robot_id)
                rec.robot_id, rec.task_id, rec.picked_up = a.robot_id, a.task_id, False
                self._change(out, t, rec, OrderState.ASSIGNED)
                out.dispatches.append(a)
            out.decisions.extend(plan.decisions)
        # (5) park rule
        assigned = {a.robot_id for a in out.dispatches}
        for rid, s in sorted(self._robots.items()):
            if rid in assigned or rid in forced_park or rid in self._park_task_of_robot or rid in self._park_done:
                continue
            if not self._available(s) or self.graph.vertices[self._vertex(s)].kind == "park":
                continue
            self._issue_park(out, s)
        return out
