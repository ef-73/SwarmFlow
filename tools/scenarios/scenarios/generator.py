"""Scenario dataclass, YAML loader with validation, and the seeded order generator."""

from __future__ import annotations

import pathlib
import random
from dataclasses import dataclass, field
from typing import Any, Dict, List

import yaml

from swarmflow_core import api

POLICIES = ("fcfs", "independent")


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
    else:
        _fail(f"orders.mode must be 'rate' or 'explicit', got {mode!r}")
    return dict(orders)


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


def generate_orders(scenario: Scenario, graph) -> List[api.OrderSpec]:
    """Deterministic order stream for ``scenario`` (same seed -> identical list)."""
    spec = scenario.orders
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
