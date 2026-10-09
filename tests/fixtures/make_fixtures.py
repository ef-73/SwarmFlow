#!/usr/bin/env python3
"""Generate the frozen M2 fixtures (design §14.1). Lead-owned; run once at the freeze:

    python3 tests/fixtures/make_fixtures.py

Writes, from layouts/standard/layout.yaml:
  tests/fixtures/standard/nav_graph.yaml   reference nav graph (layouts/schema/nav_graph.md)
  tests/fixtures/standard/zones.yaml       reference sidecar (layouts/schema/zones.schema.json)
  tests/fixtures/orders_standard.yaml      10 sample orders
  tests/fixtures/robot_states_standard.jsonl  synthetic 3-robot RobotState stream (5 Hz, 60 s)

The layout generator (WS-F) must reproduce nav_graph.yaml and zones.yaml for the standard layout (lead test).
Deterministic: no randomness, sorted output, fixed float rounding.
"""

from __future__ import annotations

import heapq
import json
import math
import pathlib

import yaml

ROOT = pathlib.Path(__file__).resolve().parents[2]
OUT = ROOT / "tests" / "fixtures"

CHASSIS = (0.60, 0.50)
PAD = 0.05  # per side (design §7.2)
PAYLOADS = {  # name: (size_x along heading, size_y lateral)
    "unloaded": (0.0, 0.0),
    "small": (0.40, 0.40),
    "medium": (0.65, 0.65),
    "wide": (0.65, 1.00),
    "long": (1.20, 0.60),
}


def r3(v: float) -> float:
    return round(float(v), 3)


def payload_geometry() -> dict:
    out = {}
    for name, (sx, sy) in PAYLOADS.items():
        px = max(CHASSIS[0], sx) + 2 * PAD
        py = max(CHASSIS[1], sy) + 2 * PAD
        out[name] = {"size_x": r3(sx), "size_y": r3(sy), "padded_x": r3(px), "padded_y": r3(py),
                     "rotation_diameter": r3(math.hypot(px, py))}
    return out


def build_graph(layout: dict):
    """Return (vertices dict name->data, directed edges list) for a layout dict."""
    verts = {}
    for s in layout["stations"]:
        verts[s["name"]] = {"x": r3(s["pose"]["x"]), "y": r3(s["pose"]["y"]), "yaw": r3(s["pose"]["yaw"]),
                            "kind": s["type"]}
    for i in layout["intersections"]:
        verts[i["name"]] = {"x": r3(i["pose"]["x"]), "y": r3(i["pose"]["y"]), "kind": "intersection"}
    for h in layout["holds"]:
        verts[h["name"]] = {"x": r3(h["pose"]["x"]), "y": r3(h["pose"]["y"]), "kind": "hold"}
    zone_of = {}
    for z in layout["zones"]:
        for ln in z["lanes"]:
            zone_of[(ln["from"], ln["to"])] = z["name"]
        for e in z["entries"]:
            verts[e["entry"]].setdefault("zone_entry_of", []).append(z["name"])
            verts[e["hold"]].setdefault("hold_for", []).append(z["name"])
    geom = payload_geometry()
    edges = []
    for ln in layout["lanes"]:
        pairs = [(ln["from"], ln["to"])] + ([(ln["to"], ln["from"])] if ln["bidirectional"] else [])
        for a, b in pairs:
            va, vb = verts[a], verts[b]
            w = ln["clear_width_m"]
            e = {"from": a, "to": b, "length_m": r3(math.hypot(vb["x"] - va["x"], vb["y"] - va["y"])),
                 "clear_width_m": r3(w), "bidirectional": bool(ln["bidirectional"]),
                 "zone": zone_of.get((a, b)),
                 "feasible": {p: bool(w >= g["padded_y"] + 0.10 - 1e-9) for p, g in geom.items()},
                 "two_way_ok": {p: bool(w >= 2 * g["padded_y"] + 0.30 - 1e-9) for p, g in geom.items()}}
            if "speed_limit_mps" in ln:
                e["speed_limit_mps"] = ln["speed_limit_mps"]
            edges.append(e)
    edges.sort(key=lambda e: (e["from"], e["to"]))
    return verts, edges


def vertex_order(verts: dict) -> list:
    def key(n):
        k = verts[n]["kind"]
        group = {"loading": 0, "delivery": 1, "park": 2, "intersection": 3, "hold": 4}[k]
        return (group, n)
    return sorted(verts, key=key)


def nav_graph(layout: dict, verts: dict, edges: list) -> dict:
    order = vertex_order(verts)
    idx = {n: i for i, n in enumerate(order)}
    vlist = []
    for n in order:
        v = verts[n]
        p = {"name": n}
        if v["kind"] == "park":
            p["is_parking_spot"] = True
            p["is_charger"] = True
        if v["kind"] == "hold":
            p["is_holding_point"] = True
        vlist.append([v["x"], v["y"], p])
    lanes = []
    for e in edges:
        p = {}
        if e.get("speed_limit_mps", 0) > 0:
            p["speed_limit"] = e["speed_limit_mps"]
        lanes.append([idx[e["from"]], idx[e["to"]], p])
    return {"building_name": layout["name"], "doors": {}, "lifts": {},
            "levels": {"L1": {"vertices": vlist, "lanes": lanes}}}


def zones_sidecar(layout: dict, verts: dict, edges: list) -> dict:
    zones = []
    for z in sorted(layout["zones"], key=lambda z: z["name"]):
        zones.append({"name": z["name"], "capacity": z["capacity"],
                      "edges": sorted([[ln["from"], ln["to"]] for ln in z["lanes"]]),
                      "vertices": sorted(z["vertices"]),
                      "entries": [{"entry": e["entry"], "hold": e["hold"]} for e in z["entries"]],
                      "polygon": [[r3(x), r3(y)] for x, y in z["polygon"]]})
    vout = {}
    for n in sorted(verts):
        v = dict(verts[n])
        for k in ("zone_entry_of", "hold_for"):
            if k in v:
                v[k] = sorted(v[k])
        vout[n] = v
    return {"schema_version": 1, "layout": layout["name"], "nav_graph": "nav_graph.yaml", "vertices": vout,
            "edges": edges, "zones": zones, "payloads": payload_geometry()}


def dijkstra(verts, edges, a, b):
    adj = {}
    for e in edges:
        adj.setdefault(e["from"], []).append((e["to"], e["length_m"]))
    dist, prev, pq = {a: 0.0}, {}, [(0.0, a)]
    while pq:
        d, u = heapq.heappop(pq)
        if u == b:
            break
        if d > dist.get(u, math.inf):
            continue
        for v, w in sorted(adj.get(u, [])):
            nd = d + w
            if nd < dist.get(v, math.inf) - 1e-12:
                dist[v], prev[v] = nd, u
                heapq.heappush(pq, (nd, v))
    path = [b]
    while path[-1] != a:
        path.append(prev[path[-1]])
    return path[::-1]


def orders() -> list:
    pairs = [("L1", "D3"), ("L2", "D1"), ("L3", "D2"), ("L1", "D1"), ("L2", "D3"),
             ("L3", "D1"), ("L1", "D2"), ("L2", "D2"), ("L3", "D3"), ("L1", "D1")]
    return [{"order_id": f"o{i + 1:03d}", "release_t": float(i * 15), "deadline_t": None,
             "pickup_vertex": p, "dropoff_vertex": d, "payload_type": "small", "priority": 0}
            for i, (p, d) in enumerate(pairs)]


def robot_states(verts, edges) -> list:
    """Synthetic stream: each robot drives its route at 0.5 m/s; robot_2 waits 4 s at a hold for a grant."""
    plans = {
        "robot_1": (["P1", "L1"], dijkstra(verts, edges, "L1", "D3"), "o001"),
        "robot_2": (["P2", "X_4_0", "X_3_0", "L3"], dijkstra(verts, edges, "L3", "D2"), "o003"),
        "robot_3": (["P3", "X_0_4", "X_1_4", "D1"], dijkstra(verts, edges, "D1", "L2"), ""),
    }
    rate, speed, horizon = 5.0, 0.5, 60.0
    out = []
    for rid, (pre, route, order_id) in sorted(plans.items()):
        full = pre + route[1:] if pre[-1] == route[0] else pre + route
        # timeline of (t, x, y, yaw, mode, last_vertex, next_vertex, remaining)
        t, events = 0.0, []
        for i in range(len(full) - 1):
            a, b = verts[full[i]], verts[full[i + 1]]
            if rid == "robot_2" and b["kind"] == "intersection" and a["kind"] == "hold":
                events.append((t, t + 4.0, full[i], full[i], "WAITING_RESERVATION", a, a))
                t += 4.0
            d = math.hypot(b["x"] - a["x"], b["y"] - a["y"])
            events.append((t, t + d / speed, full[i], full[i + 1], "NAVIGATING", a, b))
            t += d / speed
        k = 0
        while k * (1 / rate) <= horizon:
            tt = round(k / rate, 3)
            seg = next((e for e in events if e[0] <= tt < e[1]), None)
            if seg is None:
                last = full[-1]
                x, y, yaw, mode, lv, nv, spd, rem = verts[last]["x"], verts[last]["y"], 0.0, "IDLE", last, "", 0.0, []
            else:
                t0, t1, lv, nv, mode, a, b = seg
                f = 0.0 if t1 == t0 else (tt - t0) / (t1 - t0)
                x = a["x"] + f * (b["x"] - a["x"])
                y = a["y"] + f * (b["y"] - a["y"])
                yaw = math.atan2(b["y"] - a["y"], b["x"] - a["x"]) if (a is not b) else 0.0
                spd = 0.0 if mode == "WAITING_RESERVATION" else speed
                rem = full[full.index(nv) + 1:] if nv in full else []
            out.append({"t": tt, "robot_id": rid, "x": r3(x), "y": r3(y), "yaw": r3(yaw), "linear_speed": r3(spd),
                        "mode": mode, "fault_reason": "", "task_id": f"task_{rid}" if mode != "IDLE" else "",
                        "order_id": order_id if mode != "IDLE" else "", "last_vertex": lv, "next_vertex": nv,
                        "remaining_route": rem, "held_lease_ids": []})
            k += 1
    out.sort(key=lambda s: (s["t"], s["robot_id"]))
    return out


def dump_yaml(path: pathlib.Path, data) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", encoding="utf-8", newline="\n") as f:
        f.write("# GENERATED by tests/fixtures/make_fixtures.py — frozen fixture (M2), do not edit.\n")
        yaml.safe_dump(data, f, sort_keys=False, default_flow_style=None, width=120)


def main() -> None:
    layout = yaml.safe_load((ROOT / "layouts" / "standard" / "layout.yaml").read_text(encoding="utf-8"))
    verts, edges = build_graph(layout)
    dump_yaml(OUT / "standard" / "nav_graph.yaml", nav_graph(layout, verts, edges))
    dump_yaml(OUT / "standard" / "zones.yaml", zones_sidecar(layout, verts, edges))
    dump_yaml(OUT / "orders_standard.yaml", {"orders": orders()})
    with open(OUT / "robot_states_standard.jsonl", "w", encoding="utf-8", newline="\n") as f:
        for s in robot_states(verts, edges):
            f.write(json.dumps(s, sort_keys=True) + "\n")
    print("fixtures written to", OUT)


if __name__ == "__main__":
    main()
