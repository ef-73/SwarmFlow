"""Kinematic backend: simple unicycle robots driven by the real ``AgentCore`` (same protocol client as Gazebo).

Graph loading: ``from_json(path)`` loads ``nav_graph.yaml`` + ``zones.yaml`` from the directory next to the given
``sim2d.json`` via ``swarmflow_core.graph.load_layout`` (the json lacks zone capacity and per-edge payload
feasibility). The json supplies the spawn table (robot id -> vertex, yaw); robots without a spawn entry take the
remaining ``park`` vertices in sorted order.

Docking yaw at the final pose is not enforced (a kinematic model has no use for it).
"""

from __future__ import annotations

import json
import math
import pathlib
import sys
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

_ROOT = pathlib.Path(__file__).resolve().parents[2]
for _p in (_ROOT / "src" / "swarmflow_core", _ROOT / "src" / "swarmflow_robot_agent"):
    if _p.is_dir() and str(_p) not in sys.path:
        sys.path.append(str(_p))

from swarmflow_core import api  # noqa: E402
from swarmflow_core.api import Assignment, FleetSnapshot, OrderSpec, TaskResult  # noqa: E402
from swarmflow_core.graph import load_layout  # noqa: E402
from swarmflow_robot_agent.agent_core import (AgentCore, CancelNav, Navigate, Release,  # noqa: E402
                                              RequestReservation, TaskFinished)

ORCH_EPOCH = "epoch:sim2d"
ORCH_HEARTBEAT_PERIOD_S = 1.0
_DIST_EPS = 1e-9


def _wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


@dataclass
class _Robot:
    robot_id: str
    x: float
    y: float
    yaw: float
    agent: AgentCore
    poses: List[Tuple[float, float, float]] = field(default_factory=list)
    target: int = 0
    speed: float = 0.0
    inbox: List[Tuple[str, Optional[api.ReservationDecision]]] = field(default_factory=list)
    next_hb_t: float = 0.0
    next_orch_hb_t: float = 0.0


class Sim2DBackend:
    def __init__(self, graph, spawn: Dict[str, Tuple[str, float]], authority=None, speed_mps: float = 0.5,
                 turn_rate: float = 1.5, seed: int = 0, traffic_control: bool = True):
        self.graph = graph
        self.authority = authority
        self.speed_mps = speed_mps
        self.turn_rate = turn_rate
        self._t = 0.0
        self._results: List[TaskResult] = []
        self._orders: Dict[str, OrderSpec] = {}
        self.reservation_log: List[Tuple[float, api.ReservationRequest, api.ReservationDecision]] = []
        self._robots: Dict[str, _Robot] = {}
        for i, (rid, (vname, yaw)) in enumerate(sorted(spawn.items())):
            v = graph.vertices[vname]
            agent = AgentCore(rid, graph, traffic_control=traffic_control, rng_seed=seed * 1000 + i)
            self._robots[rid] = _Robot(rid, v.x, v.y, yaw, agent)
            agent.on_pose(v.x, v.y, yaw, 0.0, 0.0)

    @classmethod
    def from_json(cls, path, robots: Sequence[str], speed_mps: float = 0.5, turn_rate: float = 1.5, seed: int = 0,
                  authority_factory=None, traffic_control: bool = True) -> "Sim2DBackend":
        path = pathlib.Path(path)
        graph = load_layout(path.parent)
        data = json.loads(path.read_text(encoding="utf-8"))
        table = {s["robot_id"]: (s["vertex"], float(s.get("yaw", 0.0))) for s in data.get("spawn", [])}
        used = {table[r][0] for r in robots if r in table}
        free = [n for n in sorted(graph.vertices) if graph.vertices[n].kind == "park" and n not in used]
        spawn: Dict[str, Tuple[str, float]] = {}
        for rid in sorted(robots):
            spawn[rid] = table[rid] if rid in table else (free.pop(0), 0.0)
        authority = authority_factory(graph) if authority_factory else None
        return cls(graph, spawn, authority, speed_mps, turn_rate, seed, traffic_control)

    # ---- accessors --------------------------------------------------------------------------------------------

    def agent(self, robot_id: str) -> AgentCore:
        return self._robots[robot_id].agent

    def register_orders(self, orders: Sequence[OrderSpec]) -> None:
        for o in orders:
            self._orders[o.order_id] = o

    def poses(self) -> List[Tuple[str, float, float, float]]:
        return [(rid, r.x, r.y, r.yaw) for rid, r in sorted(self._robots.items())]

    def pop_reservation_log(self):
        log, self.reservation_log = self.reservation_log, []
        return log

    # ---- api.Backend ------------------------------------------------------------------------------------------

    def snapshot(self) -> FleetSnapshot:
        return FleetSnapshot(t=self._t, robots=[r.agent.state() for _, r in sorted(self._robots.items())],
                             open_orders=())

    def dispatch(self, assignment: Assignment) -> None:
        r = self._robots[assignment.robot_id]
        order = self._orders.get(assignment.order_id)
        acts = r.agent.start_task(assignment.task_id, assignment.order_id, assignment.route, self._t,
                                  pickup=order.pickup_vertex if order else "",
                                  dropoff=order.dropoff_vertex if order else "")
        self._execute(r, acts, self._t)

    def cancel(self, task_id: str) -> None:
        for _, r in sorted(self._robots.items()):
            if r.agent.state().task_id == task_id:
                self._execute(r, r.agent.cancel(self._t), self._t)

    def poll_results(self) -> Sequence[TaskResult]:
        res, self._results = self._results, []
        return res

    # ---- simulation -------------------------------------------------------------------------------------------

    def step(self, t: float, dt: float) -> None:
        self._t = t
        for _, r in sorted(self._robots.items()):
            inbox, r.inbox = r.inbox, []
            for rid, dec in inbox:
                self._execute(r, r.agent.on_reservation_response(rid, dec, t), t)
            self._move(r, dt)
            self._execute(r, r.agent.on_pose(r.x, r.y, r.yaw, r.speed, t), t)
            if r.poses and r.target >= len(r.poses):
                r.poses, r.target, r.speed = [], 0, 0.0
                self._execute(r, r.agent.on_nav_result(True, t), t)
            if t >= r.next_orch_hb_t:
                r.next_orch_hb_t = t + ORCH_HEARTBEAT_PERIOD_S
                self._execute(r, r.agent.on_orchestrator_heartbeat(t, epoch=ORCH_EPOCH), t)
            if t >= r.next_hb_t:
                r.next_hb_t = t + api.HEARTBEAT_PERIOD_S
                if self.authority is not None:
                    self.authority.heartbeat(r.robot_id, r.agent.heartbeat_lease_ids(), t)
            self._execute(r, r.agent.tick(t), t)

    def _move(self, r: _Robot, dt: float) -> None:
        """Turn in place towards the next pose, then drive straight; leftover time carries to the next pose."""
        budget = dt
        r.speed = 0.0
        while r.poses and r.target < len(r.poses) and budget > 1e-12:
            tx, ty, _ = r.poses[r.target]
            dx, dy = tx - r.x, ty - r.y
            dist = math.hypot(dx, dy)
            if dist <= _DIST_EPS:
                r.target += 1
                continue
            want = math.atan2(dy, dx)
            diff = _wrap(want - r.yaw)
            if diff != 0.0:
                need = abs(diff) / self.turn_rate
                if need <= budget:
                    r.yaw = want
                    budget -= need
                else:
                    r.yaw = _wrap(r.yaw + math.copysign(self.turn_rate * budget, diff))
                    budget = 0.0
                    break
            need = dist / self.speed_mps
            if need <= budget:
                r.x, r.y = tx, ty
                r.target += 1
                budget -= need
            else:
                f = self.speed_mps * budget / dist
                r.x, r.y = r.x + dx * f, r.y + dy * f
                budget = 0.0
            r.speed = self.speed_mps

    def _execute(self, r: _Robot, acts: Sequence[object], t: float) -> None:
        for a in acts:
            if isinstance(a, Navigate):
                r.poses, r.target = list(a.poses), 0
            elif isinstance(a, CancelNav):
                r.poses, r.target, r.speed = [], 0, 0.0
            elif isinstance(a, RequestReservation):
                if self.authority is None:
                    continue
                dec = self.authority.request(a.request, t)
                self.reservation_log.append((t, a.request, dec))
                r.inbox.append((a.request.request_id, dec))     # delivered on the next step
            elif isinstance(a, Release):
                if self.authority is not None:
                    self.authority.release(r.robot_id, a.lease_id, t, a.reason)
            elif isinstance(a, TaskFinished):
                self._results.append(TaskResult(a.task_id, r.robot_id, a.success, a.failure_reason, t))
