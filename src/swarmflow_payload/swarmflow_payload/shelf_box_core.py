"""Boxes of v2 shelf orders (T066). Pure Python: no ROS, no gz; time is always an argument.

A shelf order is picked on arrival (no dwell, api contract v2), so a box appears on the robot's deck when the robot's
``last_vertex`` becomes one of the order's pick points, one box per pick, stacked. The boxes ride on the robot until it
leaves UNLOADING (the truck takes them), and go back to the pool when the order is requeued, fails or is delivered, or
when the robot starts another order. Blocked (STUCK) robots keep their boxes.

Uses the same ``pkg_NN`` models as :class:`PayloadCore` (v1), which never moves them in v2: v2 robots never LOAD.
"""

from __future__ import annotations

from typing import Dict, List, Mapping, Sequence, Set, Tuple

from .payload_core import PADDED_FOOTPRINT, PAYLOAD_SIZE, PLATFORM_Z, POOL_X_OFFSET, POOL_Y_STEP

BOX_H = 0.30            #: package height: boxes stack this far apart
POOL_SIZE_V2 = 16       #: pkg models in a v2 world (tools/layoutgen POOL_V2)

Pose4 = Tuple[float, float, float, float]


class _Robot:
    __slots__ = ("pose", "order_id", "boxes", "picked", "unloading")

    def __init__(self):
        self.pose = None                 # (x, y, yaw)
        self.order_id = ""               # order the boxes belong to
        self.boxes: List[str] = []       # bottom first
        self.picked: Set[str] = set()    # pick vertices already loaded for order_id
        self.unloading = False


class ShelfBoxCore:
    def __init__(self, bounds: Sequence[float], pool_size: int = POOL_SIZE_V2):
        self.bounds = tuple(bounds)
        self.warnings: List[str] = []
        self._names = [f"pkg_{i:02d}" for i in range(pool_size)]
        self._free = set(self._names)
        self._picks: Dict[str, Set[str]] = {}       # shelf order -> pick vertices
        self._robots: Dict[str, _Robot] = {}
        self._pending: Dict[str, Pose4] = {}

    def pool_pose(self, pkg: str) -> Pose4:
        i = self._names.index(pkg)
        return (self.bounds[0] - POOL_X_OFFSET, self.bounds[1] + POOL_Y_STEP * i, BOX_H / 2, 0.0)

    # -- inputs ----------------------------------------------------------------------------------------------------
    def on_order(self, order_id: str, pick_vertices: Sequence[str]) -> None:
        if pick_vertices:
            self._picks[order_id] = set(pick_vertices)

    def on_order_state(self, order_id: str, state: str) -> None:
        """QUEUED (requeued), FAILED, DELIVERED: the order's boxes leave the robot that carries them."""
        if state not in ("QUEUED", "FAILED", "DELIVERED"):
            return
        for r in self._robots.values():
            if r.order_id == order_id and r.boxes:
                self._release(r)

    def on_robot_state(self, state: Mapping, t: float) -> None:
        r = self._robots.setdefault(state["robot_id"], _Robot())
        if "x" in state:
            r.pose = (float(state["x"]), float(state["y"]), float(state.get("yaw", 0.0)))
        mode = getattr(state["mode"], "value", state["mode"])
        order_id = state.get("order_id", "")
        if order_id and order_id != r.order_id:
            if r.boxes:
                self._release(r)                     # another order: the old boxes are not on board any more
            r.order_id, r.picked, r.unloading = order_id, set(), False
        if mode == "UNLOADING":
            r.unloading = bool(r.boxes)
            return
        if r.unloading:
            self._release(r)                         # left UNLOADING: the truck has the boxes
            return
        if mode == "FAULT":
            if r.boxes:
                self._release(r)
            return
        v = state.get("last_vertex", "")
        if v and r.order_id and v in self._picks.get(r.order_id, ()) and v not in r.picked:
            r.picked.add(v)
            self._take(r, state["robot_id"])

    def _take(self, r: _Robot, rid: str) -> None:
        if not self._free:
            self.warnings.append(f"payload pool exhausted: no box for {rid} order {r.order_id!r}")
            return
        pkg = min(self._free)
        self._free.discard(pkg)
        r.boxes.append(pkg)

    def _release(self, r: _Robot) -> None:
        for pkg in r.boxes:
            self._pending[pkg] = self.pool_pose(pkg)
            self._free.add(pkg)
        r.boxes = []
        r.unloading = False

    # -- output ----------------------------------------------------------------------------------------------------
    def tick(self, t: float) -> Dict[str, Pose4]:
        out: Dict[str, Pose4] = dict(self._pending)
        self._pending.clear()
        for r in self._robots.values():
            if r.pose is None:
                continue
            x, y, yaw = r.pose
            for k, pkg in enumerate(r.boxes):
                out[pkg] = (x, y, PLATFORM_Z + BOX_H * k, yaw)
        return out

    def boxes_of(self, robot_id: str) -> List[str]:
        r = self._robots.get(robot_id)
        return list(r.boxes) if r else []

    def payload_state(self, robot_id: str) -> dict:
        r = self._robots.get(robot_id)
        loaded = r is not None and bool(r.boxes)
        return {"robot_id": robot_id, "loaded": loaded, "order_id": r.order_id if loaded else "",
                "payload_type": "small", "size_x": PAYLOAD_SIZE, "size_y": PAYLOAD_SIZE,
                "footprint": list(PADDED_FOOTPRINT)}
