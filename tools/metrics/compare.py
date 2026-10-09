#!/usr/bin/env python3
"""Mean +/- 95 % CI table over runs (design §13.2). Usage: compare.py <run_dir>... [--out file.md]"""

from __future__ import annotations

import argparse
import csv
import math
import pathlib
import sys

T975 = [12.706, 4.303, 3.182, 2.776, 2.571, 2.447, 2.365, 2.306, 2.262, 2.228,
        2.201, 2.179, 2.160, 2.145, 2.131, 2.120, 2.110, 2.101, 2.093, 2.086,
        2.080, 2.074, 2.069, 2.064, 2.060, 2.056, 2.052, 2.048, 2.045, 2.042]
METRICS = ["deliveries", "throughput_per_min", "mean_latency_s", "mean_wait_s", "stuck_events",
           "failed_orders", "clearance_violations"]


def t_crit(df: int) -> float:
    return T975[df - 1] if df <= 30 else 1.96


def mean_ci(vals: list) -> str:
    n = len(vals)
    mean = sum(vals) / n
    if n < 2:
        return f"{mean:.2f} ± n/a"
    sd = math.sqrt(sum((v - mean) ** 2 for v in vals) / (n - 1))
    return f"{mean:.2f} ± {t_crit(n - 1) * sd / math.sqrt(n):.2f}"


def build_table(run_dirs: list) -> str:
    groups = {}
    for d in run_dirs:
        with open(pathlib.Path(d) / "metrics.csv", newline="", encoding="utf-8") as f:
            for row in csv.DictReader(f):
                groups.setdefault((row["scenario"], row["policy"]), []).append(row)
    lines = ["| scenario | policy | n | " + " | ".join(METRICS) + " |",
             "|" + "---|" * (3 + len(METRICS))]
    for (scenario, policy) in sorted(groups):
        rows = groups[(scenario, policy)]
        cells = [mean_ci([float(r[m]) for r in rows]) for m in METRICS]
        lines.append(f"| {scenario} | {policy} | {len(rows)} | " + " | ".join(cells) + " |")
    return "\n".join(lines) + "\n"


def main(argv: list) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("runs", nargs="+")
    ap.add_argument("--out")
    args = ap.parse_args(argv[1:])
    try:
        table = build_table(args.runs)
    except FileNotFoundError as e:
        print(f"compare.py: missing file: {e}", file=sys.stderr)
        return 1
    if args.out:
        pathlib.Path(args.out).write_text(table, encoding="utf-8", newline="\n")
    else:
        sys.stdout.write(table)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
