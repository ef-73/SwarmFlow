---
id: T040
title: Steady-load v2 scenario for 4 robots (scenarios/v2_steady.yaml)
workstream: F
status: review
claimed_by: "sf-mechanical"
branch: ws-f/T040-v2-steady
model: haiku
risk: low
---

## Goal
A v2 scenario whose order rate 4 robots can serve (the first Gazebo v2 run with `v2_demo` at ~4.5 orders/min left a
backlog of 27 orders), for the M15 acceptance run and the E-01 benchmarks.

## Owned files
- scenarios/v2_steady.yaml

## Lead tests (do not edit)
- tools/scenarios/tests/test_v2_steady.py

## Forbidden
- AGENTS.md §4; no other files.

## Commands to run
```bash
export PATH="/c/Program Files/Docker/Docker/resources/bin:$PATH" COMPOSE_PROJECT_NAME=swarmflow-t040
docker compose run --rm --no-deps dev python3 -m pytest -q tools/scenarios/tests/test_v2_steady.py
```

## Definition of done
- `scenarios/v2_steady.yaml` modelled on `scenarios/v2_demo.yaml` with exactly the values the lead test asserts
  (name v2_steady, seed 2, 600 s, warehouse_v2, policy assign, robot_1…robot_4, docks DK1…DK4, shelf 1.6/min picks
  [1, 2] zipf 1.0, pallet 0.25/min) and a 2-line header comment; the lead test passes.

## Inputs / contracts used
- scenarios/v2_demo.yaml, tools/scenarios/tests/test_v2_orders.py (format)
