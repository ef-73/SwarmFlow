---
id: T003
title: FCFS reservation authority (safety-critical)
workstream: B
status: open
claimed_by: "sf-implementer"
branch: ws-b/T003-reservations
model: sonnet
risk: safety-critical
---

## Goal
`swarmflow_core.reservations.FcfsReservationAuthority` implements `swarmflow_core.api.ReservationAuthority` with all
rules of design §6.5 and passes the lead's property tests.

## Owned files
- src/swarmflow_core/swarmflow_core/reservations.py
- src/swarmflow_core/test/test_reservations_extra.py

## Lead tests (do not edit)
- src/swarmflow_core/test/test_reservations.py

## Forbidden
- AGENTS.md §4; no ROS imports; no wall-clock time (`time.time`, `datetime.now`); no randomness.
- Do not edit `api.py`, `graph.py` or the fixtures.

## Commands to run
```bash
export MSYS_NO_PATHCONV=1 PATH="/c/Program Files/Docker/Docker/resources/bin:$PATH"
export COMPOSE_PROJECT_NAME=swarmflow-<your-worktree-dir-name>
docker compose run --rm -T dev python3 -m pytest -q -p no:cacheprovider src/swarmflow_core/test
```

## Specification
The lease state machine, recovery, release-while-inside and pose-stamp rules are written in the
`ReservationAuthority` docstring in `api.py` — that docstring is the spec. In addition:
- Constructor: `FcfsReservationAuthority(graph, lease_ttl_s=api.LEASE_TTL_S, retry_after_s=api.RETRY_AFTER_S)`;
  `graph` is a `swarmflow_core.graph.WarehouseGraph` (zones with `capacity`, `entries`, `polygon`).
- A new instance is **recovering** (all requests → `DENY_RECOVERING`) until `recover(robots, t)` runs once.
- Every public method first applies expiry at its `t`. A lease expires when `t >= expiry_t`;
  `expiry_t = min(last_renewal_t + lease_ttl_s, hard_expiry_t)`. Heartbeats renew only the caller's own GRANTED
  leases and never beyond `hard_expiry_t`.
- Blocking = state `OCCUPIED_UNKNOWN`; remember for each blocking lease the time it became blocking. Clearing
  (`RELEASED`, reason `OBSERVED_OUTSIDE` / `LATE_RELEASE`) needs a pose stamp ≥ that time outside the polygon
  (`swarmflow_core.graph.point_in_polygon`; boundary = inside), or a late `release` while the robot's last observed
  pose is outside (no pose ever observed → the release is trusted). `clear_zone` → `REVOKED`, reason
  `CLEAR_ZONE:<reason>`.
- `release` of a GRANTED lease while the last observed pose is inside the zone → `OCCUPIED_UNKNOWN`, reason
  `RELEASED_INSIDE:<ReleaseReason>`; otherwise `RELEASED`, reason = the ReleaseReason value.
- `request` order: recovering → `DENY_RECOVERING`; unknown zone → `DENY_UNKNOWN_ZONE`; `entry_vertex` not an entry
  of the zone → `DENY_BAD_ENTRY`; requester already holds a GRANTED lease in the zone → GRANTED, same lease id,
  renewed (no event); zone has a blocking lease → `DENY_OCCUPIED_UNKNOWN`; GRANTED leases ≥ capacity →
  `DENY_ZONE_LEASED`; else GRANTED (new lease). Denies carry `retry_after_s`.
- `recover(robots, t)`: for each robot (sorted by id), every zone whose polygon contains its pose gets a blocking
  lease `recovered_<robot_id>_<zone>` (hard/TTL expiry = t); returns them (and records events). Leaves recovery mode.
- Lease ids are deterministic: `lease_00001`, `lease_00002`, … per instance.
- Events (`pop_events`): one `Lease` snapshot per state change, in order. `active_leases()`: GRANTED and blocking
  leases sorted by `lease_id`.
- Keep it simple and readable (~200–300 lines). Add your own tests in `test_reservations_extra.py` for anything the
  spec says that the lead tests don't cover.

## Definition of done
- `src/swarmflow_core/test/test_reservations.py` passes unchanged (incl. the hypothesis safety-invariant test),
  plus your extra tests; whole `src/swarmflow_core/test` passes.
- Commit on your branch with message "T003: FCFS reservation authority". Report test output.

## Inputs / contracts used
- docs/design.md §6.5; src/swarmflow_core/swarmflow_core/api.py; src/swarmflow_core/swarmflow_core/graph.py;
  tests/fixtures/standard/
