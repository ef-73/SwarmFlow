#!/usr/bin/env python3
"""Writes layouts/warehouse_v2/layout.yaml (T026 geometry, improvements_v2 L-01..L-06).

Usage: build_warehouse_v2.py [out_path]   (default layouts/warehouse_v2/layout.yaml)
The geometry lives only here; edit this script and regenerate, then run tools/layoutgen/generate.py.
"""

import math
import pathlib
import sys

import yaml

PI, HALF_PI = round(math.pi, 6), round(math.pi / 2, 6)
ROW_Y0, ROW_PITCH, ROW_DEPTH = 8.0, 3.2, 1.0
AISLE_W = 2.2
CROSS_N_Y = 23.4          # centre line of the north cross aisle (y 21.8 .. 25.0)
STRIP_Y = 2.5             # home-strip travel line
PALLET_Y = 5.5
DOCK_Y = 30.6
DOCK_X = {1: 2.0, 2: 6.5, 3: 13.5, 4: 18.0}
EXIT_X = (4.25, 15.75)


def r(v):
    return round(v, 4)


def build():
    racks, stations, inter, lanes, aisles = [], [], [], [], []
    for side, (x0, x1) in (("W", (0.0, 8.0)), ("E", (12.0, 20.0))):
        for i in range(1, 6):
            y0 = ROW_Y0 + ROW_PITCH * (i - 1)
            racks.append({"name": f"R_{side}{i}", "x_min": x0, "y_min": r(y0), "x_max": x1,
                          "y_max": r(y0 + ROW_DEPTH), "height_m": 2.0})

    def st(name, typ, x, y, yaw, **extra):
        stations.append({"name": name, "type": typ, "pose": {"x": r(x), "y": r(y), "yaw": yaw}, **extra})
        return name

    nodes = {}

    def node(x, y):
        key = (r(x), r(y))
        if key not in nodes:
            nodes[key] = f"X_{len(nodes) // 10}_{len(nodes) % 10}"
            inter.append({"name": nodes[key], "pose": {"x": key[0], "y": key[1]}})
        return nodes[key]

    def lane(a, b, w):
        lanes.append({"from": a, "to": b, "clear_width_m": w, "bidirectional": True})

    def chain(names, w):
        for a, b in zip(names, names[1:]):
            lane(a, b, w)

    # central aisle spine (x = 10) and storage aisles
    spine = [node(10, STRIP_Y), node(10, PALLET_Y)]
    for a in range(1, 5):
        y_lo = ROW_Y0 + ROW_DEPTH + ROW_PITCH * (a - 1)
        y_hi = y_lo + AISLE_W
        yc = (y_lo + y_hi) / 2
        spine.append(node(10, yc))
        for side, xs, xm, yaw in (("W", (7, 5, 3, 1), 7.5, PI), ("E", (13, 15, 17, 19), 12.5, 0.0)):
            aisles.append({"name": f"{side}{a}", "x_min": 0.0 if side == "W" else 12.0, "y_min": r(y_lo),
                           "x_max": 8.0 if side == "W" else 20.0, "y_max": r(y_hi), "axis": "x", "keep": "right"})
            mouth = node(xm, yc)
            lane(node(10, yc), mouth, 4.0)
            for face, fy in (("S", y_lo + 0.45), ("N", y_hi - 0.45)):
                prev = mouth
                for ss, x in enumerate(xs, start=1):
                    prev_n = prev
                    prev = st(f"PK_{side}{a}_{face}_{ss:02d}", "pick", x, fy, yaw)
                    lane(prev_n, prev, AISLE_W)
    spine.append(node(10, CROSS_N_Y))
    chain(spine, 4.0)
    aisles.insert(0, {"name": "central", "x_min": 8.0, "y_min": 3.0, "x_max": 12.0, "y_max": 21.8, "axis": "y",
                      "keep": "right"})
    aisles.append({"name": "cross_north", "x_min": 0.0, "y_min": 21.8, "x_max": 20.0, "y_max": 25.0, "axis": "x",
                   "keep": "right"})
    aisles.append({"name": "cross_south", "x_min": 0.0, "y_min": 0.0, "x_max": 20.0, "y_max": 3.0, "axis": "x",
                   "keep": "right"})

    # diagonal shortcuts between the last aisle and the north cross aisle (clear of the rack corners)
    c19 = node(10, ROW_Y0 + ROW_DEPTH + ROW_PITCH * 3 + 1.1)
    # north cross line with dock-queue entries and exit corridors
    cross_x = sorted([*DOCK_X.values(), *EXIT_X, 8.5, 10.0, 11.5])
    cross = [node(x, CROSS_N_Y) for x in cross_x]
    chain(cross, 3.2)
    lane(c19, node(8.5, CROSS_N_Y), 4.0)
    lane(c19, node(11.5, CROSS_N_Y), 4.0)

    # docks, queue lanes, exit corridors
    exits = {x: node(x, DOCK_Y) for x in EXIT_X}
    for x in EXIT_X:
        lane(node(x, CROSS_N_Y), exits[x], 2.0)
    for k, x in DOCK_X.items():
        dk = st(f"DK{k}", "dock", x, DOCK_Y, HALF_PI)
        prev = dk
        qs = []
        for i in range(1, 4):
            q = st(f"Q_DK{k}_{i}", "queue", x, DOCK_Y - 1.6 * i, HALF_PI, queue_of=dk, queue_index=i)
            lane(prev, q, 2.0)
            prev = q
        lane(prev, node(x, CROSS_N_Y), 2.0)
        near = min(EXIT_X, key=lambda e: abs(e - x))
        lane(dk, exits[near], 2.0)

    # homes (south strip) and pallet stands
    strip = []
    for k in range(1, 9):
        x = 1.5 + 2.4 * (k - 1)
        hm = st(f"HM{k}", "home", x, 1.5, HALF_PI)
        n = node(x, STRIP_Y)
        lane(hm, n, 3.0)
        strip.append((x, n))
    strip.append((10.0, node(10, STRIP_Y)))
    chain([n for _, n in sorted(strip)], 3.0)
    pal = {"PL_W1": 2.5, "PL_W2": 5.5, "PL_E1": 14.5, "PL_E2": 17.5}
    for nm, x in pal.items():
        st(nm, "pallet", x, PALLET_Y, HALF_PI)
    centre = node(10, PALLET_Y)
    chain(["PL_W1", "PL_W2", centre, "PL_E1", "PL_E2"], 3.0)

    spawn = [{"robot_id": f"robot_{k}", "vertex": f"HM{k}", "yaw": HALF_PI} for k in range(1, 6)]
    return {"schema_version": 2, "name": "warehouse_v2",
            "description": "v2 warehouse: two rack blocks, central aisle, 4 truck docks with queue lanes, 8 home slots, "
                           "4 pallet stands, keep-right aisles.",
            "resolution_m": 0.05,
            "bounds": {"x_min": 0.0, "y_min": 0.0, "x_max": 20.0, "y_max": 32.0, "wall_height_m": 2.5,
                       "wall_thickness_m": 0.2},
            "racks": racks, "stations": stations, "intersections": inter, "holds": [], "lanes": lanes, "zones": [],
            "aisles": aisles, "spawn": spawn}


def flow(d):
    s = yaml.safe_dump(d, sort_keys=False, default_flow_style=True, width=10_000).strip()
    return s[:-3].strip() if s.endswith("...") else s


def main():
    out = pathlib.Path(sys.argv[1] if len(sys.argv) > 1 else "layouts/warehouse_v2/layout.yaml")
    L = build()
    lines = ["# GENERATED by tools/layoutgen/build_warehouse_v2.py - edit the builder, not this file.",
             "# Map frame: x east, y north, origin = south-west inner wall corner. Units m / rad."]
    for k in ("schema_version", "name", "description", "resolution_m"):
        lines.append(f"{k}: {flow(L[k])}")
    lines.append(f"bounds: {flow(L['bounds'])}")
    for k in ("racks", "stations", "intersections", "holds", "lanes", "zones", "aisles", "spawn"):
        if not L[k]:
            lines.append(f"{k}: []")
            continue
        lines.append(f"{k}:")
        lines.extend(f"  - {flow(i)}" for i in L[k])
    out.parent.mkdir(parents=True, exist_ok=True)
    with open(out, "w", encoding="utf-8", newline="\n") as f:
        f.write("\n".join(lines) + "\n")


if __name__ == "__main__":
    main()
