---
id: T019
title: Robot agent hardening — lost leases, pose validity
workstream: D
status: review
claimed_by: "sf-implementer"
branch: ws-d/T019-agent-m9
model: sonnet
risk: safety-critical
---

## Goal
Fix findings S1 and S4 of the M9 independent review in `swarmflow_robot_agent`, without contract changes.

## Owned files
- src/swarmflow_robot_agent/**

## Lead tests (do not edit)
- src/swarmflow_robot_agent/test/test_agent_core.py
- src/swarmflow_robot_agent/test/test_agent_node.py
- src/swarmflow_robot_agent/test/test_agent_core_m9.py

## Forbidden
- AGENTS.md §4; no contract edits; no other packages.

## Commands to run
```bash
export MSYS_NO_PATHCONV=1 PATH="/c/Program Files/Docker/Docker/resources/bin:$PATH"
export COMPOSE_PROJECT_NAME=swarmflow-t019
scripts/lock.sh acquire build T019 20     # wait/retry if held; release after
docker compose run --rm -T dev colcon build --symlink-install --parallel-workers 2
docker compose run --rm -T dev python3 -m pytest -q -p no:cacheprovider src/swarmflow_robot_agent/test sim2d
scripts/lock.sh release build
```

## Specification
**S1 — lost lease before entry (safety).** Today a granted agent drives into its zone even if the lease has meanwhile
expired, been revoked (`/fleet/clear_zone`) or been released by the authority. Add
`AgentCore.on_lease_event(lease_id, state: api.LeaseState, t) -> actions`: for a lease the agent holds, a state of
`OCCUPIED_UNKNOWN`, `REVOKED`, `RELEASED` or `EXPIRED` drops the lease; if the robot has **not yet entered** that zone
(pose never seen inside its polygon), emit `CancelNav`, go to `WAITING_RESERVATION` and request the zone again
(immediately or on the next tick); if it is inside, keep driving out (rule 6 spirit; the authority blocks the zone).
Other leases / `GRANTED` events: no action. The node subscribes `/fleet/reservations`
(`ZoneReservation`, reliable, **transient-local** depth 100 — the orchestrator publishes it latched) and feeds events
for its own `robot_id` (map the uint8 state with `api.LEASE_STATE_CODES`).
**S4 — pose validity.** `AgentCore.has_pose()`, `pose_time()`; before the first `on_pose`, `state()` is mode `FAULT`
with `fault_reason "NO_POSE"` and `can_accept()` is False; if `tick(t)` finds `t − pose_time > POSE_STALE_S (1.0)`,
`state()` reports `FAULT` / `"STALE_POSE"` (an overlay: the task state machine is not reset; a fresh pose clears it).
Node: do **not** publish `RobotState` before the first valid pose; stamp each `RobotState` header with the **pose time**
(TF stamp), not `now`, so the orchestrator's `observe_robot` never treats an old pose as fresh.
- Keep all existing behaviour; update your own extra tests if needed. Note in the module docstring why (M9 review).

## Definition of done
- All three lead test files pass unchanged; `sim2d` tests (which drive AgentCore) still pass. Commit "T019: agent
  hardening", status `review`. Report exact test summaries.

## Inputs / contracts used
- design §6.3, §6.5; api.py (LeaseState, LEASE_STATE_CODES, ROBOT_STATE_PERIOD_S); swarmflow_interfaces/ZoneReservation.
