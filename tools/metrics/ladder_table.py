#!/usr/bin/env python3
"""Realism-ladder table (I-01): mean ± sd per rung over runs/<prefix>_r<rung>_<k>/ (metrics.csv, loc_error.json,
realism.txt). Usage: ladder_table.py runs ladder > table.md"""

from __future__ import annotations

import csv
import json
import pathlib
import re
import statistics
import sys

COLS = [("deliveries", "Deliveries / 600 s"), ("throughput_per_min", "Orders / min"),
        ("mean_latency_s", "Mean latency (s)"), ("stuck_events", "Stuck"), ("failed_orders", "Failed"),
        ("clearance_violations", "Clearance violations"), ("near_misses", "Near misses (< 0.30 m)"),
        ("min_separation_m", "Min footprint distance (m)")]


def ms(vals):
    vals = [v for v in vals if v == v]
    if not vals:
        return "—"
    if len(vals) == 1:
        return f"{vals[0]:.2f}"
    return f"{statistics.mean(vals):.2f} ± {statistics.stdev(vals):.2f}"


def collect(root: pathlib.Path, prefix: str):
    rungs = {}
    for d in sorted(root.glob(f"{prefix}_r*_*")):
        m = re.match(rf"{re.escape(prefix)}_r(\d+)_(\d+)$", d.name)
        if not m or not (d / "metrics.csv").is_file():
            continue
        row = next(csv.DictReader(open(d / "metrics.csv", encoding="utf-8")))
        loc = None
        if (d / "loc_error.json").is_file():
            le = json.load(open(d / "loc_error.json", encoding="utf-8"))
            means = [v["mean_m"] for v in le.values() if v.get("mean_m") is not None]
            loc = (statistics.mean(means), max(v["max_m"] for v in le.values() if v.get("max_m") is not None)) \
                if means else None
        realism = (d / "realism.txt").read_text(encoding="utf-8").strip() if (d / "realism.txt").is_file() else ""
        rungs.setdefault(int(m.group(1)), []).append((row, loc, realism))
    return rungs


def table(rungs) -> str:
    out = ["| Rung | Runs | " + " | ".join(t for _, t in COLS) + " | Localisation error mean / worst max (m) |",
           "|---|---|" + "---|" * len(COLS) + "---|"]
    for r in sorted(rungs):
        runs = rungs[r]
        cells = [ms([float(x[0][c]) for x in runs]) for c, _ in COLS]
        locs = [x[1] for x in runs if x[1]]
        loc = f"{statistics.mean(l[0] for l in locs):.2f} / {max(l[1] for l in locs):.2f}" if locs else "0 (ground truth)"
        out.append(f"| {r} | {len(runs)} | " + " | ".join(cells) + f" | {loc} |")
    out.append("")
    for r in sorted(rungs):
        out.append(f"- Rung {r}: " + "; ".join(sorted({x[2].replace(chr(10), ', ') for x in rungs[r]})))
    return "\n".join(out) + "\n"


def main(argv):
    if len(argv) != 3:
        print("usage: ladder_table.py <runs_dir> <prefix>", file=sys.stderr)
        return 2
    print(table(collect(pathlib.Path(argv[1]), argv[2])), end="")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
