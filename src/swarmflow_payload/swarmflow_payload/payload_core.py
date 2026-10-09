"""Package pose-follower core (design §7.4). Pure Python: no ROS, no gz; time is always an argument."""

from __future__ import annotations

import math
from typing import Dict, List, Mapping, Optional, Tuple

from swarmflow_core import api

POOL_SIZE = 12
PLATFORM_Z = 0.45        #: box centre while carried: platform top 0.30 m + half the 0.30 m package
FLOOR_Z = 0.15           #: box centre when lying on the floor / parked
RECYCLE_AFTER_S = 10.0
POOL_X_OFFSET = 3.0      #: pool x = x_min - 3.0 (same as the generated world)
POOL_Y_STEP = 0.6
PADDED_FOOTPRINT = [(0.35, 0.30), (0.35, -0.30), (-0.35, -0.30), (-0.35, 0.30)]  # v1 constant (design §7.2)
PAYLOAD_SIZE = 0.40

Pose4 = Tuple[float, float, float, float]


class _Carry:
    __slots__ = ("pkg", "order_id", "unload_start")

    def __init__(self, pkg: str, order_id: str):
        self.pkg = pkg
        self.order_id = order_id
        self.unload_start: Optional[float] = None


class PayloadCore:
    def __init__(self, graph, bounds: Tuple[float, float, float, float], drop_offsets: Mapping[str, float]):
        self.graph = graph
        self.bounds = tuple(bounds)
        self.drop_offsets = dict(drop_offsets)
        self.warnings: List[str] = []
        self._names = [f"pkg_{i:02d}" for i in range(POOL_SIZE)]
        self._free = set(self._names)
        self._poses: Dict[str, Tuple[float, float, float]] = {}      # robot -> latest ground-truth (x, y, yaw)
        self._carry: Dict[str, _Carry] = {}                           # robot -> carried package
        self._dropped: Dict[str, float] = {}                          # pkg -> drop time (lying on the floor)
        self._pending: Dict[str, Pose4] = {}                          # one-shot pose updates for tick()
        self._last_order: Dict[str, str] = {}                         # robot -> order of its last dropped package

    # -- helpers ---------------------------------------------------------------------------------------------------
    def pool_pose(self, pkg: str) -> Pose4:
        i = self._names.index(pkg)
        return (self.bounds[0] - POOL_X_OFFSET, self.bounds[1] + POOL_Y_STEP * i, FLOOR_Z, 0.0)

    def initial_poses(self) -> Dict[str, Pose4]:
        return {n: self.pool_pose(n) for n in self._names}

    # -- inputs ----------------------------------------------------------------------------------------------------
    def on_robot_pose(self, robot_id: str, x: float, y: float, yaw: float, t: float) -> None:
        self._poses[robot_id] = (x, y, yaw)

    def on_robot_state(self, state: Mapping, t: float) -> None:
        rid = state["robot_id"]
        mode = state["mode"]
        mode = getattr(mode, "value", mode)
        order_id = state.get("order_id", "")
        c = self._carry.get(rid)
        if mode == "LOADING":
            if c is None and self._last_order.get(rid) != order_id:
                self._take(rid, order_id)
            return
        if c is None:
            return
        if mode == "UNLOADING":
            if c.unload_start is None:
                c.unload_start = t
        elif mode in ("IDLE", "STUCK", "FAULT"):
            # task ended without completing UNLOADING: package returns to the pool
            del self._carry[rid]
            self._pending[c.pkg] = self.pool_pose(c.pkg)
            self._free.add(c.pkg)

    def _take(self, rid: str, order_id: str) -> None:
        if not self._free:
            self.warnings.append(f"payload pool exhausted: no package for {rid} order {order_id!r}")
            return
        pkg = min(self._free)
        self._free.discard(pkg)
        self._pending.pop(pkg, None)
        self._dropped.pop(pkg, None)
        self._carry[rid] = _Carry(pkg, order_id)

    def _drop_pose(self, rid: str) -> Pose4:
        x, y, yaw = self._poses.get(rid, (0.0, 0.0, 0.0))
        name = self.graph.nearest_vertex(x, y, kinds=("delivery",))
        v = self.graph.vertices[name]
        syaw = v.yaw if v.yaw is not None else yaw
        off = self.drop_offsets.get(name, 0.0)
        return (v.x + off * math.cos(syaw), v.y + off * math.sin(syaw), FLOOR_Z, syaw)

    # -- output ----------------------------------------------------------------------------------------------------
    def tick(self, t: float) -> Dict[str, Pose4]:
        out: Dict[str, Pose4] = dict(self._pending)
        self._pending.clear()
        for rid in list(self._carry):
            c = self._carry[rid]
            if c.unload_start is not None and t - c.unload_start >= api.LOAD_DWELL_S - 1e-9:
                out[c.pkg] = self._drop_pose(rid)
                self._dropped[c.pkg] = c.unload_start + api.LOAD_DWELL_S
                self._last_order[rid] = c.order_id
                del self._carry[rid]
            elif rid in self._poses:
                x, y, yaw = self._poses[rid]
                out[c.pkg] = (x, y, PLATFORM_Z, yaw)
        for pkg in list(self._dropped):
            if t - self._dropped[pkg] >= RECYCLE_AFTER_S - 1e-9:
                out[pkg] = self.pool_pose(pkg)
                del self._dropped[pkg]
                self._free.add(pkg)
        return out

    def package_of(self, robot_id: str) -> Optional[str]:
        c = self._carry.get(robot_id)
        return c.pkg if c else None

    def payload_state(self, robot_id: str) -> dict:
        c = self._carry.get(robot_id)
        return {
            "robot_id": robot_id,
            "loaded": c is not None,
            "order_id": c.order_id if c else "",
            "payload_type": "small",
            "size_x": PAYLOAD_SIZE,
            "size_y": PAYLOAD_SIZE,
            "footprint": list(PADDED_FOOTPRINT),
        }
