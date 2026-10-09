---
id: T011
title: Metrics — clearance metric, compute.py, compare.py
workstream: B
status: review
claimed_by: "sf-implementer"
branch: ws-b/T011-metrics
model: sonnet
risk: normal
---

## Goal
Run records (`runs/<run_id>/`, design §13.5) turn into `metrics.csv`, and several runs into a mean ± 95 % CI table
for the README (§13.2). The clearance metric (§13.1, C20) lives in `swarmflow_core` so both backends share it.
Lead-delegated card: `tools/metrics/` is a Lead path handed to this card.

## Owned files
- src/swarmflow_core/swarmflow_core/metrics/**
- tools/metrics/**

## Lead tests (do not edit)
- tools/metrics/tests/test_metrics.py

## Forbidden
- AGENTS.md §4; stdlib + PyYAML only (no numpy/scipy); no ROS imports in `swarmflow_core`.

## Commands to run
```bash
export MSYS_NO_PATHCONV=1 PATH="/c/Program Files/Docker/Docker/resources/bin:$PATH"
export COMPOSE_PROJECT_NAME=swarmflow-t011
docker compose run --rm -T dev python3 -m pytest -q -p no:cacheprovider tools/metrics src/swarmflow_core/test
```

## Specification
**`swarmflow_core/metrics/clearance.py`** (`metrics/__init__.py` may be empty):
- Padded footprint = 0.70 × 0.60 m rectangle centred on the robot pose (v1 constant; design §7.2), rotated by yaw.
- `footprint_distance(pose_a, pose_b) -> float` (poses `(x, y, yaw)`): minimum distance between the two rectangles,
  **negative when they overlap** (any negative value; e.g. minus the penetration depth via separating-axis test).
- `count_violations(samples: dict[t -> dict[robot_id -> (x, y, yaw)]]) -> int`: in time order, for every robot pair a
  violation is a sample with distance < 0; contiguous violating samples of one pair count once (§13.1).
**`tools/metrics/compute.py <run_dir>`** (plus a package `tools/metrics/metrics_tools/` if you like and
`tools/metrics/pyproject.toml`) reads `config.yaml` (key `scenario` with `name`, `policy`, `seed`, `duration_s`; fall
back to parsing the run dir name `<scenario>-<policy>-s<seed>-...` and to the last sample time),
`orders.jsonl`, `decisions.jsonl`, `robot_states.jsonl`; writes `metrics.csv` (one header row + one data row) with
columns, in this order: `run_id, scenario, policy, seed, duration_s, deliveries, throughput_per_min, mean_latency_s,
total_wait_s, mean_wait_s, stuck_events, failed_orders, clearance_violations`.
- policy = the `policy` field of the first decision in `decisions.jsonl` that has one (the orchestrator's actual
  policy; Baseline A runs override the scenario file), else the config value.
- deliveries = orders whose last state is DELIVERED; failed_orders = last state FAILED; throughput = deliveries /
  (duration_s / 60); latency = DELIVERED t − first QUEUED t, mean over delivered orders (empty → 0).
- waiting time (§13.1): per robot, sum over consecutive state samples of the interval length while mode is
  WAITING_RESERVATION or (mode NAVIGATING and linear_speed < 0.05); total = sum over robots, mean = total / robots.
- stuck_events = number of STUCK_FAIL decisions; clearance_violations = `count_violations` over the state samples
  grouped by `t`. Numbers formatted with 3 decimals (ints as ints). Exit 1 with a message if a file is missing.
**`tools/metrics/compare.py <run_dir>... [--out file.md]`** reads each `metrics.csv`, groups by (scenario, policy),
writes a Markdown table: header `| scenario | policy | n | deliveries | throughput_per_min | mean_latency_s |
mean_wait_s | stuck_events | failed_orders | clearance_violations |`, one row per group sorted by scenario then policy,
each metric as `mean ± half-width` with 2 decimals, half-width = t(0.975, n−1) · sd / √n (sample sd; hard-code the
two-sided 95 % t table for df 1–30, df > 30 → 1.96); n = 1 → `x.xx ± n/a`. Prints the table to stdout when `--out`
is absent.

## Definition of done
- Lead tests pass unchanged; add tests (e.g. rotated overlap, empty runs). Commit "T011: metrics", card status
  `review`. Report test output.

## Inputs / contracts used
- docs/design.md §13.1, §13.2, §13.5; orchestrator run-record format (`src/swarmflow_orchestrator`, `decisions.jsonl`,
  `orders.jsonl`, `robot_states.jsonl`); scenario engine `config.yaml` (`src/swarmflow_scenarios`)
