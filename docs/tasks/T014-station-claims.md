---
id: T014
title: Station claims in FleetCore (no two robots sent to one station)
workstream: B
status: review
claimed_by: "sf-implementer"
branch: ws-b/T014-station-claims
model: sonnet
risk: normal
---

## Goal
M6 demo finding: FCFS sent two robots to loading station L2 at once; they met at the station (padded footprints
overlapping) and one hit STUCK_TIMEOUT. FleetCore must only plan orders whose stations are free.

## Owned files
- src/swarmflow_core/swarmflow_core/fleet.py
- src/swarmflow_core/test/test_fleet_extra.py

## Lead tests (do not edit)
- src/swarmflow_core/test/test_station_claims.py

## Forbidden
- AGENTS.md §4; do not edit api.py, policies, reservations, the other lead tests.

## Commands to run
```bash
export MSYS_NO_PATHCONV=1 PATH="/c/Program Files/Docker/Docker/resources/bin:$PATH"
export COMPOSE_PROJECT_NAME=swarmflow-t014
docker compose run --rm -T dev python3 -m pytest -q -p no:cacheprovider src/swarmflow_core/test
```

## Specification
In `FleetCore.tick` step (4), before calling the policy, filter the QUEUED orders (keeping FIFO order) to those whose
pickup station is not claimed by another order in state ASSIGNED or PICKING_UP and whose dropoff station is not
claimed by another order that is ASSIGNED, PICKING_UP or IN_TRANSIT. Orders admitted in this tick claim their stations
for the rest of the tick too (two QUEUED orders with the same pickup → only the older is planned). The docstring of
FleetCore states the rule and why. Everything else unchanged; all existing tests must still pass.

## Definition of done
- Lead test passes unchanged; whole `src/swarmflow_core/test` passes; commit "T014: station claims", status `review`.

## Inputs / contracts used
- docs/design.md §6.7, §13.5; fleet.py; runs evidence summarised in the Goal.
