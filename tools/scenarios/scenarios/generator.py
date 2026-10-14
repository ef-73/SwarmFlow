"""Scenario dataclass, YAML loader with validation, and the seeded order generator."""

from __future__ import annotations

import pathlib
import random
from dataclasses import dataclass, field
from typing import Any, Dict, List

import yaml

from swarmflow_core import api

POLICIES = ("fcfs", "independent", "assign")


@dataclass(frozen=True)
class Scenario:
    name: str
    seed: int
    duration_s: float
    layout: str
    policy: str
    robots: List[str]
    orders: Dict[str, Any] = field(default_factory=dict)


def _fail(msg: str):
    raise ValueError(msg)


def _num(d: dict, key: str, ctx: str, positive: bool = False) -> float:
    v = d.get(key)
    if isinstance(v, bool) or not isinstance(v, (int, float)):
        _fail(f"{ctx}: '{key}' must be a number, got {v!r}")
    if positive and v <= 0:
        _fail(f"{ctx}: '{key}' must be > 0, got {v!r}")
    return float(v)


def _str_list(v: Any, ctx: str) -> List[str]:
    if not isinstance(v, list) or not v or not all(isinstance(x, str) for x in v):
        _fail(f"{ctx} must be a non-empty list of strings, got {v!r}")
    return list(v)


def _validate_orders(orders: Any) -> Dict[str, Any]:
    if not isinstance(orders, dict):
        _fail("'orders' must be a mapping")
    mode = orders.get("mode")
    if mode == "rate":
        _num(orders, "rate_per_min", "orders", positive=True)
        _str_list(orders.get("pickups"), "orders.pickups")
        _str_list(orders.get("dropoffs"), "orders.dropoffs")
    elif mode == "explicit":
        lst = orders.get("list")
        if not isinstance(lst, list) or not lst:
            _fail("orders.list must be a non-empty list")
        for i, e in enumerate(lst):
            if not isinstance(e, dict):
                _fail(f"orders.list[{i}] must be a mapping")
            if _num(e, "t", f"orders.list[{i}]") < 0:
                _fail(f"orders.list[{i}]: 't' must be >= 0")
            for k in ("pickup", "dropoff"):
                if not isinstance(e.get(k), str):
                    _fail(f"orders.list[{i}]: '{k}' must be a string")
        if orders.get("repeat_every_s") is not None:
            _num(orders, "repeat_every_s", "orders", positive=True)
    elif mode == "v2_rate":
        _validate_v2(orders)
    else:
        _fail(f"orders.mode must be 'rate', 'explicit' or 'v2_rate', got {mode!r}")
    return dict(orders)


def _validate_v2(orders: dict) -> None:
    _str_list(orders.get("docks"), "orders.docks")
    shelf = orders.get("shelf")
    if not isinstance(shelf, dict):
        _fail("orders.shelf must be a mapping")
    _num(shelf, "rate_per_min", "orders.shelf", positive=True)
    picks = shelf.get("picks")
    if (not isinstance(picks, list) or len(picks) != 2
            or not all(isinstance(x, int) and not isinstance(x, bool) for x in picks)
            or not 1 <= picks[0] <= picks[1]):
        _fail(f"orders.shelf.picks must be [min, max] with 1 <= min <= max, got {picks!r}")
    pop = shelf.get("popularity", "uniform")
    if pop not in ("uniform", "zipf"):
        _fail(f"orders.shelf.popularity must be 'uniform' or 'zipf', got {pop!r}")
    if "zipf_s" in shelf:
        _num(shelf, "zipf_s", "orders.shelf", positive=True)
    slots = shelf.get("slots", "all")
    if slots != "all":
        _str_list(slots, "orders.shelf.slots (or 'all')")
    pallet = orders.get("pallet")
    if pallet is not None:
        if not isinstance(pallet, dict):
            _fail("orders.pallet must be a mapping")
        _num(pallet, "rate_per_min", "orders.pallet", positive=True)


def load_scenario(path) -> Scenario:
    p = pathlib.Path(path)
    try:
        raw = yaml.safe_load(p.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as e:
        raise ValueError(f"cannot read scenario {p}: {e}") from e
    if not isinstance(raw, dict):
        _fail(f"{p}: scenario must be a mapping")
    for k in ("name", "seed", "duration_s", "layout", "policy", "robots", "orders"):
        if k not in raw:
            _fail(f"{p}: missing field '{k}'")
    if not isinstance(raw["name"], str) or not raw["name"]:
        _fail("'name' must be a non-empty string")
    if isinstance(raw["seed"], bool) or not isinstance(raw["seed"], int):
        _fail("'seed' must be an integer")
    duration = _num(raw, "duration_s", "scenario", positive=True)
    if not isinstance(raw["layout"], str) or not raw["layout"]:
        _fail("'layout' must be a non-empty string")
    if raw["policy"] not in POLICIES:
        _fail(f"'policy' must be one of {POLICIES}, got {raw['policy']!r}")
    robots = _str_list(raw["robots"], "'robots'")
    return Scenario(raw["name"], raw["seed"], duration, raw["layout"], raw["policy"], robots,
                    _validate_orders(raw["orders"]))


def _check_vertex(graph, name: str, kind: str, role: str) -> None:
    v = graph.vertices.get(name)
    if v is None:
        _fail(f"{role} '{name}' is not a vertex of the layout")
    if v.kind != kind:
        _fail(f"{role} '{name}' has kind '{v.kind}', expected '{kind}'")


def _poisson_times(rng: random.Random, rate_per_min: float, duration_s: float):
    rate_per_s = rate_per_min / 60.0
    t = 0.0
    while True:
        t += rng.expovariate(rate_per_s)
        if t >= duration_s:
            return
        yield t


def _generate_v2(scenario: Scenario, graph) -> List[api.OrderSpec]:
    spec = scenario.orders
    docks = list(spec["docks"])
    for d in docks:
        _check_vertex(graph, d, "dock", "dock")
    shelf = spec["shelf"]
    slots = sorted(graph.vertices_of_kind("pick")) if shelf.get("slots", "all") == "all" else list(shelf["slots"])
    for v in slots:
        _check_vertex(graph, v, "pick", "pick slot")
    if not slots:
        _fail("the layout has no pick slots")
    lo, hi = shelf["picks"]
    hi = min(hi, len(slots))
    lo = min(lo, hi)
    slots = sorted(slots)
    if shelf.get("popularity", "uniform") == "zipf":
        s = float(shelf.get("zipf_s", 1.0))
        weights = [1.0 / (rank ** s) for rank in range(1, len(slots) + 1)]
    else:
        weights = [1.0] * len(slots)

    rng = random.Random(scenario.seed)
    raw: List[tuple] = []  # (release_t, stream (0 shelf, 1 pallet), spec kwargs)
    for t in _poisson_times(rng, float(shelf["rate_per_min"]), scenario.duration_s):
        n = rng.randint(lo, hi)
        chosen: List[str] = []
        avail, w = list(slots), list(weights)
        for _ in range(n):
            i = rng.choices(range(len(avail)), weights=w)[0]
            chosen.append(avail.pop(i))
            w.pop(i)
        raw.append((t, 0, dict(order_type=api.ORDER_SHELF, pickup_vertex="", dropoff_vertex=rng.choice(docks),
                               payload_type="small", pick_vertices=tuple(chosen))))
    pallet = spec.get("pallet")
    if pallet is not None:
        spots = sorted(graph.vertices_of_kind("pallet"))
        for t in _poisson_times(rng, float(pallet["rate_per_min"]), scenario.duration_s):
            if not spots:
                break
            spot = spots.pop(rng.randrange(len(spots)))
            raw.append((t, 1, dict(order_type=api.ORDER_PALLET, pickup_vertex=spot, dropoff_vertex=rng.choice(docks),
                                   payload_type="pallet", pallet_id=f"pallet_{spot}")))
    raw.sort(key=lambda r: (r[0], r[1]))
    return [api.OrderSpec(order_id=f"o{i:04d}", release_t=t, deadline_t=None, priority=0, **kw)
            for i, (t, _, kw) in enumerate(raw, start=1)]


def generate_orders(scenario: Scenario, graph) -> List[api.OrderSpec]:
    """Deterministic order stream for ``scenario`` (same seed -> identical list)."""
    spec = scenario.orders
    if spec["mode"] == "v2_rate":
        return _generate_v2(scenario, graph)
    raw: List[tuple] = []  # (release_t, pickup, dropoff)
    if spec["mode"] == "rate":
        pickups, dropoffs = list(spec["pickups"]), list(spec["dropoffs"])
        for p in pickups:
            _check_vertex(graph, p, "loading", "pickup")
        for d in dropoffs:
            _check_vertex(graph, d, "delivery", "dropoff")
        rng = random.Random(scenario.seed)
        rate_per_s = float(spec["rate_per_min"]) / 60.0
        t = 0.0
        while True:
            t += rng.expovariate(rate_per_s)
            if t >= scenario.duration_s:
                break
            raw.append((t, rng.choice(pickups), rng.choice(dropoffs)))
    else:
        for e in spec["list"]:
            _check_vertex(graph, e["pickup"], "loading", "pickup")
            _check_vertex(graph, e["dropoff"], "delivery", "dropoff")
        period = spec.get("repeat_every_s")
        k = 0
        while True:
            offset = k * float(period) if period else 0.0
            if offset >= scenario.duration_s:
                break
            for e in spec["list"]:
                t = offset + float(e["t"])
                if t < scenario.duration_s:
                    raw.append((t, e["pickup"], e["dropoff"]))
            if not period:
                break
            k += 1
    raw.sort(key=lambda r: r[0])  # stable: ties keep list order
    return [api.OrderSpec(order_id=f"o{i:04d}", release_t=t, deadline_t=None, pickup_vertex=p, dropoff_vertex=d,
                          payload_type="small", priority=0)
            for i, (t, p, d) in enumerate(raw, start=1)]
