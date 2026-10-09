---
id: T017
title: Corridor choreography that also works at Gazebo timing
workstream: F
status: review
claimed_by: "sf-implementer"
branch: ws-f/T017-corridor-gazebo
model: sonnet
risk: low
---

## Goal
In Gazebo (M7 run 1) `v1_corridor` produced **no** opposing traffic: under FCFS 4 deliveries, 0 denies; every aisle
crossing was eastbound and loaded. Redesign the explicit order list so robots meet head-on in a storage aisle every
cycle at Gazebo-like timing, verified with the 2D backend (real AgentCore) slowed to Gazebo kinematics.

## Owned files
- scenarios/v1_corridor.yaml
- tools/scenarios/tests/test_generator_extra.py

## Lead tests (do not edit)
- tools/scenarios/tests/test_corridor_sim2d.py
- tools/scenarios/tests/test_scenarios.py

## Forbidden
- AGENTS.md §4; do not edit swarmflow_core, sim2d, the agent or the generator.

## Commands to run
```bash
export MSYS_NO_PATHCONV=1 PATH="/c/Program Files/Docker/Docker/resources/bin:$PATH"
export COMPOSE_PROJECT_NAME=swarmflow-t017
docker compose run --rm -T dev python3 -m pytest -q -p no:cacheprovider tools/scenarios
```

## What Gazebo showed (run v1_corridor-fcfs-s1-r1, sim seconds)
- o0001 L2→D2 assigned 13.2 to robot_1 (P1), aisle-2 eastbound 56.5–73.5, delivered 84.0.
- o0002 L2→D1 could only be assigned at 42.8 (L2 pickup claim), to robot_3 starting at P3: its trip P3→L2 goes via the
  south cross aisle (shorter than through aisle 2 from P3), so the L2 claim was held ~100 s.
- robot_1 sat idle after delivering at 84.0 (next L2 order blocked by robot_3's L2 claim) → park rule sent it away;
  it got o0003 only at 138.8, again from a park vertex → again no westbound aisle trip.
- Route facts (graph, shortest path, ties broken by vertex name): from D2 the way back to L1 or L2 goes through
  aisle 2 westbound; from a park vertex P3/P4 it does not. L2→D1/D2/D3 all go through aisle 2 eastbound.
- Station claims (fleet.py docstring): pickup claimed until picked up; dropoff claimed until delivered.

## Specification
- Design orders so that, in steady state, one robot leaves D2 westbound (next pickup reached through aisle 2) while the
  other is loaded eastbound in aisle 2 — e.g. both robots start on opposite sides and the order list keeps a backlog
  with alternating pickups/dropoffs that never block each other's claims. 2 or 3 robots; you may change `robots`,
  `repeat_every_s`, `duration_s` (≤ 360 s) and the list. Explain the choreography in the YAML comment, including why
  the claims never block it.
- Iterate with the lead test (sim2d at 0.45 m/s, 0.8 rad/s, orders shifted +10 s); throwaway search scripts only in
  your temp dir.

## Definition of done
- Both lead test files pass unchanged; commit "T017: corridor for Gazebo timing", card status `review`. Report the
  choreography and the deny minutes / max robots in zone you observed.
