---
id: T027
title: v2 order generator (shelf + pallet orders) and scenario engine fields
workstream: F
status: review
claimed_by: "sf-implementer"
branch: ws-f/T027-v2-orders
model: sonnet
risk: normal
---

## Goal
Seeded, reproducible v2 order streams for `warehouse_v2` (improvements_v2 P-02, F-06), published with the v2
`Order` fields, plus a `scenarios/v2_demo.yaml`.

## Design (lead)
- `tools/scenarios/scenarios/generator.py`: new `orders.mode: v2_rate` (format in the lead test docstring).
  Validate at load: `docks` non-empty list; `shelf.picks` = [min, max] with 1 ≤ min ≤ max; `popularity` in
  {uniform, zipf} (`zipf_s` > 0, default 1.0); `slots` = "all" or a list. At generation, check vertex kinds
  (`dock`, `pick`) like `_check_vertex`. Shelf orders: Poisson arrivals at `rate_per_min`; number of picks uniform in
  [min, max] (capped by the slot count), distinct picks drawn by popularity (zipf: weight 1 / rank^s over the slots in
  sorted name order); dock uniform. Pallet orders: separate Poisson stream; spot drawn uniformly from the spots
  not used yet (stop when none left); `payload_type: pallet`, `pallet_id: pallet_<spot>`. Both streams use one
  `random.Random(seed)` in a fixed order of draws so output is deterministic; merge by release time (ties: shelf
  first), ids `o0001`… in that order. v1 modes unchanged.
- Policy names accepted by scenarios: add `assign` (v2 fleet core).
- `scenarios/v2_demo.yaml`: layout `warehouse_v2`, policy `assign`, robots robot_1…robot_5, 600 s, docks DK1…DK4,
  shelf 4.0/min picks [1, 3] zipf 1.0, pallet 0.5/min, seed 1.
- `src/swarmflow_scenarios/swarmflow_scenarios/scenario_engine.py`: publish `order_type` (via
  `api.ORDER_TYPE_CODES`), `pick_vertices`, `pallet_id`; add `order_type`, `pick_vertices`, `pallet_id` to the
  `order_released` event record. v1 behaviour unchanged.

## Owned files
- tools/scenarios/** (except the lead tests below)
- scenarios/v2_demo.yaml
- src/swarmflow_scenarios/swarmflow_scenarios/scenario_engine.py

## Lead tests (do not edit)
- tools/scenarios/tests/test_v2_orders.py
- src/swarmflow_scenarios/test/test_scenario_engine_v2.py

## Forbidden
- AGENTS.md §4; no Gazebo; v1 scenario output must stay byte-identical (existing tests).

## Commands to run
```bash
export PATH="/c/Program Files/Docker/Docker/resources/bin:$PATH" COMPOSE_PROJECT_NAME=swarmflow-t027 SWARMFLOW_GUI=none
docker compose run --rm --no-deps dev python3 -m pytest -q tools/scenarios
scripts/lock.sh acquire build T027 30 && docker compose run --rm dev bash -c "colcon build --symlink-install --parallel-workers 2 && colcon test --packages-select swarmflow_scenarios && colcon test-result --verbose"; scripts/lock.sh release build
```

## Definition of done
- Lead tests and existing tests pass (`tools/scenarios`, `colcon test` for swarmflow_scenarios).
- `scripts/ci.sh` green on the branch rebased onto `main`.

## Inputs / contracts used
- `api.py` v2 (OrderSpec fields, ORDER_* constants), `Order.msg`, `layouts/warehouse_v2/generated`.
