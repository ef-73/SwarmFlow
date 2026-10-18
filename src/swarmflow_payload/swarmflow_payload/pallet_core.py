"""Pallet follower core (F-03). Pure Python: no ROS, no gz; time is always an argument.

A pallet order's pallet rides on the robot that carries it: not while LIFTING, raised by ``LIFT_Z`` while CARRYING and
UNLOADING, and set down (z 0) at the robot's pose when the robot leaves UNLOADING; it never moves again afterwards.
"""

from __future__ import annotations

from typing import Dict, List, Mapping, Tuple

from swarmflow_core import api

LIFT_Z = 0.05
TRUCK_AFTER_S = 30.0          # the truck takes a pallet set down at a dock this long afterwards
POOL_SPACING = 1.5
CARRY_FOOTPRINT: List[Tuple[float, float]] = [(0.625, 0.525), (0.625, -0.525), (-0.625, -0.525), (-0.625, 0.525)]

Pose4 = Tuple[float, float, float, float]


class _Robot:
    __slots__ = ("mode", "pose", "pallet", "order_id", "unloading")

    def __init__(self):
        self.mode = ""
        self.pose = None            # (x, y, yaw)
        self.pallet = None          # model name while carrying
        self.order_id = ""
        self.unloading = False


class PalletCore:
    def __init__(self, spots: Mapping[str, Pose4], pool_origin: Tuple[float, float] = (-3.0, 0.0)):
        self.pool_origin = (float(pool_origin[0]), float(pool_origin[1]))
        self._set_down_t: Dict[str, float] = {}             # pallet -> set-down time, until the truck takes it
        self._taken = 0
        self.spots: Dict[str, Pose4] = {k: tuple(v) for k, v in spots.items()}
        self._orders: Dict[str, str] = {}                   # pallet order id -> pallet model name
        self._robots: Dict[str, _Robot] = {}
        self._emitted: Dict[str, Pose4] = {}                # last pose sent per pallet
        self._pending: Dict[str, Pose4] = {}                # set-down poses waiting for the next tick
        self._done = set()                                  # pallets set down for good

    def initial_poses(self) -> Dict[str, Pose4]:
        return dict(self.spots)

    def on_order(self, order_id: str, order_type: str, pallet_id: str) -> None:
        if order_type == api.ORDER_PALLET and pallet_id in self.spots:
            self._orders[order_id] = pallet_id

    def on_robot_state(self, state: Mapping, t: float) -> None:
        rid = state["robot_id"]
        mode = state["mode"]
        mode = getattr(mode, "value", mode)
        r = self._robots.setdefault(rid, _Robot())
        r.pose = (float(state.get("x", 0.0)), float(state.get("y", 0.0)), float(state.get("yaw", 0.0)))
        r.mode = mode
        order_id = state.get("order_id", "")
        if r.pallet is None:
            pallet = self._orders.get(order_id)
            if mode == "CARRYING" and pallet is not None and pallet not in self._done:
                r.pallet = pallet
                r.order_id = order_id
                r.unloading = False
            return
        if mode == "UNLOADING":
            r.unloading = True
        elif r.unloading:
            self._set_down(r, t)         # left UNLOADING for any mode: set the pallet down where the robot is

    def _set_down(self, r: _Robot, t: float) -> None:
        x, y, yaw = r.pose
        self._pending[r.pallet] = (x, y, 0.0, yaw)
        self._set_down_t[r.pallet] = t
        self._done.add(r.pallet)
        r.pallet = None
        r.order_id = ""
        r.unloading = False

    def tick(self, t: float) -> Dict[str, Pose4]:
        out: Dict[str, Pose4] = {}
        carried = set()
        for r in self._robots.values():
            if r.pallet is not None and r.pose is not None:
                x, y, yaw = r.pose
                out[r.pallet] = (x, y, LIFT_Z, yaw)
                carried.add(r.pallet)
        out.update(self._pending)
        self._pending.clear()
        for pallet, t0 in list(self._set_down_t.items()):
            if t >= t0 + TRUCK_AFTER_S and pallet not in out:
                out[pallet] = (self.pool_origin[0], self.pool_origin[1] + POOL_SPACING * self._taken, 0.0, 0.0)
                self._taken += 1
                del self._set_down_t[pallet]
        moved = {k: v for k, v in out.items() if k in carried or self._emitted.get(k) != v}   # carried: every tick
        self._emitted.update(moved)
        return moved

    def payload_state(self, robot_id: str) -> dict:
        r = self._robots.get(robot_id)
        carrying = r is not None and r.pallet is not None
        return {"robot_id": robot_id, "loaded": carrying, "order_id": r.order_id if carrying else "",
                "payload_type": "pallet" if carrying else "", "footprint": list(CARRY_FOOTPRINT)}
