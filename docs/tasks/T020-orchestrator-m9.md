---
id: T020
title: Orchestrator / fleet-core hardening (STUCK recovery, pose validity, run records, clock reset)
workstream: B
status: done
claimed_by: "sf-implementer"
branch: ws-b/T020-orchestrator-m9
model: sonnet
risk: safety-critical
---

## Goal
Fix findings S4 (orchestrator side), S5, S7 and D2 of the M9 independent review without contract changes.

## Owned files
- src/swarmflow_core/swarmflow_core/fleet.py
- src/swarmflow_core/test/test_fleet_extra.py
- src/swarmflow_orchestrator/**

## Lead tests (do not edit)
- src/swarmflow_core/test/test_fleet.py
- src/swarmflow_core/test/test_station_claims.py
- src/swarmflow_core/test/test_fleet_m9.py
- src/swarmflow_orchestrator/test/test_orchestrator_node.py

## Forbidden
- AGENTS.md §4; no contract edits; no other packages.

## Commands to run
```bash
export MSYS_NO_PATHCONV=1 PATH="/c/Program Files/Docker/Docker/resources/bin:$PATH"
export COMPOSE_PROJECT_NAME=swarmflow-t020
docker compose run --rm -T dev python3 -m pytest -q -p no:cacheprovider src/swarmflow_core/test sim2d tools
scripts/lock.sh acquire build T020 20     # wait/retry if held; release after
docker compose run --rm -T dev colcon build --symlink-install --parallel-workers 2
docker compose run --rm -T dev python3 -m pytest -q -p no:cacheprovider src/swarmflow_orchestrator/test
scripts/lock.sh release build
```

## Specification
1. **S5 STUCK recovery (fleet.py):** constants `STUCK_RECOVERY_S = 30.0`, `STUCK_RECOVERY_ATTEMPTS = 3`. A robot whose
   latest snapshot is STUCK (or whose last park task failed) and that has no active task gets, after
   `STUCK_RECOVERY_S` since its last failure, a new move-only task to the nearest free park vertex, at most
   `STUCK_RECOVERY_ATTEMPTS` times in a row (counter reset when a task of that robot succeeds). Each recovery emits a
   `DecisionType.REROUTE` decision with `trigger="stuck_recovery:<attempt>"`. A robot that completes its recovery
   park task becomes available again.
2. **S4 pose validity (orchestrator_node.py):** RobotStates with `mode == FAULT` and `fault_reason` in
   `{"NO_POSE", "STALE_POSE"}` are passed to the core (the robot is unavailable) but **not** to
   `authority.observe_robot` (an unknown or old pose must never clear a blocked zone). The robot agent now stamps
   RobotState with the pose time (T019) — keep using `header.stamp`.
3. **S7 clock reset:** if the node clock goes backwards by more than 5 s (Gazebo restarted under a running
   orchestrator), log a WARN, reset the per-robot newest-stamp / last-seen bookkeeping and restart the recovery window
   (the authority goes back to recovering: create a new `FcfsReservationAuthority` and `recover()` again once every
   robot has reported). Orders and tasks stay as they are.
4. **D2 run records:** the run-record files (`decisions.jsonl`, `orders.jsonl`, `robot_states.jsonl`,
   `reservations.jsonl`) are truncated when the orchestrator starts (a reused run id must not mix two runs).
- Add tests for 2–4 in `src/swarmflow_orchestrator/test/` (new files) — prefer condition waits over fixed sleeps.

## Definition of done
- All lead tests pass unchanged; commit "T020: orchestrator hardening", status `review`. Report exact summaries.

## Inputs / contracts used
- design §6.5, §13.5; api.py; fleet.py; orchestrator_node.py; M9 review findings S4, S5, S7, D2 (quoted above).
