"""Run loop (pattern of ``swarmflow_core.backends.fake.run_fake``) and result type."""

from __future__ import annotations

import hashlib
import math
import pathlib
from dataclasses import dataclass
from typing import Dict, List, Sequence

from swarmflow_core import api
from swarmflow_core.api import Decision, OrderSpec, OrderState
from swarmflow_core.decisions import reservation_decision
from swarmflow_core.fleet import FleetCore
from swarmflow_core.policies.fcfs import FcfsPolicy
from swarmflow_core.reservations import FcfsReservationAuthority

from .backend import Sim2DBackend

_TERMINAL = (OrderState.DELIVERED, OrderState.FAILED)


@dataclass
class Sim2DResult:
    order_states: Dict[str, OrderState]
    decisions: List[Decision]
    max_robots_in_zone: int
    finish_t: float
    trajectory_digest: str


def run(layout_json, orders: Sequence[OrderSpec], policy: str, robots: Sequence[str], until_t: float,
        dt: float = 0.1, seed: int = 0) -> Sim2DResult:
    use_authority = policy != api.POLICY_INDEPENDENT
    backend = Sim2DBackend.from_json(
        pathlib.Path(layout_json), robots, seed=seed, traffic_control=use_authority,
        authority_factory=FcfsReservationAuthority if use_authority else None)
    graph, authority = backend.graph, backend.authority
    core = FleetCore(graph, FcfsPolicy(name=policy), authority, traffic_control=use_authority)
    backend.register_orders(orders)
    for o in orders:
        core.add_order(o)
    if authority is not None:
        authority.recover(backend.snapshot().robots, 0.0)
    digest = hashlib.sha256()
    decisions: List[Decision] = []
    order_ids = [o.order_id for o in orders]
    max_in_zone = 0
    finish_t = until_t
    steps = int(math.floor(until_t / dt + 1e-9))
    for i in range(steps + 1):
        t = round(i * dt, 9)
        backend.step(t, dt)
        for rid, x, y, yaw in backend.poses():
            digest.update(f"{t:.3f}|{rid}|{x:.3f}|{y:.3f}|{yaw:.3f};".encode())
        counts: Dict[str, int] = {}
        for s in backend.snapshot().robots:
            core.update_robot(s)
            if authority is not None:
                authority.observe_robot(s.robot_id, s.x, s.y, t)
            zone = graph.zone_at(s.x, s.y)
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
        if all(core.order_state(oid) in _TERMINAL for oid in order_ids):
            finish_t = t
            break
    states = {oid: core.order_state(oid) for oid in order_ids}
    return Sim2DResult(states, decisions, max_in_zone, finish_t, digest.hexdigest())
