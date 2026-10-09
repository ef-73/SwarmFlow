---
id: T016
title: Retune v1_corridor (and check v1_demo) under station claims
workstream: F
status: open
claimed_by: "sf-implementer"
branch: ws-f/T016-corridor-retune
model: sonnet
risk: low
---

## Goal
T014 added station claims to FleetCore (an order is planned only if its pickup station is not claimed by another
order still ASSIGNED/PICKING_UP and its dropoff station is not claimed by another order that is ASSIGNED,
PICKING_UP or IN_TRANSIT). With `v1_corridor` sending every order L2→D2, orders now serialize and no two robots meet:
the lead test `test_corridor_makes_robots_meet` fails (0 denies). Redesign the corridor choreography so robots still
meet head-on in one storage aisle every cycle, now with distinct stations.

## Owned files
- scenarios/v1_corridor.yaml
- scenarios/v1_demo.yaml
- tools/scenarios/tests/test_generator_extra.py

## Lead tests (do not edit)
- tools/scenarios/tests/test_scenarios.py

## Forbidden
- AGENTS.md §4; do not edit swarmflow_core or the generator code.

## Commands to run
```bash
export MSYS_NO_PATHCONV=1 PATH="/c/Program Files/Docker/Docker/resources/bin:$PATH"
export COMPOSE_PROJECT_NAME=swarmflow-t016
docker compose run --rm -T dev python3 -m pytest -q -p no:cacheprovider tools/scenarios sim2d src/swarmflow_core/test
```

## Specification
- Hint: a loaded robot crosses an aisle west→east (pickup L*, dropoff D*); a robot that just delivered at D* and is
  assigned an order with a west-side pickup reached through the same aisle crosses it east→west, empty. Use pairs of
  orders with **different** pickups/dropoffs (e.g. L2→D2 alternated with L2→D3 / L1→D2 …) and release offsets so the
  two directions overlap in one aisle each 60 s cycle, as in the current choreography. Search with `run_fake` like
  before; 2 or 3 robots.
- Keep `v1_demo` delivering ≥ 10 in the fake backend (lead test); adjust its rate only if needed.
- Update the YAML comment that explains the choreography.

## Definition of done
- All lead tests pass unchanged (with T014 in the branch). Commit "T016: corridor retune", card status `review`.
  Report the choreography and the deny cycles found.

## Inputs / contracts used
- src/swarmflow_core/swarmflow_core/fleet.py (claims docstring); backends/fake.py; layouts/standard/layout.yaml
