---
id: T023
title: Replace fixed sleeps in ROS node tests with condition waits (X-06)
workstream: Lead
status: review
claimed_by: "sf-implementer"
branch: lead/T023-test-waits
model: sonnet
risk: low
---

## Goal
ROS node tests no longer wait a fixed time for something to happen: each "settle" sleep becomes a wait on an
observable condition, so the tests are faster and not flaky on a busy machine (improvements_v2 X-06).

## Owned files
- src/swarmflow_orchestrator/test/test_orchestrator_node.py
- src/swarmflow_orchestrator/test/test_orchestrator_extra.py
- src/swarmflow_orchestrator/test/test_orchestrator_review.py
- src/swarmflow_robot_agent/test/test_agent_node.py
- src/swarmflow_viz/test/test_viz_node.py
- src/swarmflow_viz/test/test_viz_qos.py

## Lead tests (do not edit)
- none (the owned files are older lead tests; this card hands them over)

## Forbidden
- AGENTS.md §4; no Gazebo; no changes outside the owned files (no production code changes)
- Do not weaken any assertion, remove a test, or raise a timeout above 15 s.

## Commands to run
```bash
export PATH="/c/Program Files/Docker/Docker/resources/bin:$PATH" COMPOSE_PROJECT_NAME=swarmflow-t023 SWARMFLOW_GUI=none
scripts/lock.sh acquire build T023 30 && docker compose run --rm dev bash -c "colcon build --symlink-install --parallel-workers 2 && colcon test --parallel-workers 1 --packages-select swarmflow_orchestrator swarmflow_robot_agent swarmflow_viz && colcon test-result --verbose"; scripts/lock.sh release build
```

## Definition of done
- Every `time.sleep(<n>)` in the owned files is one of: (a) inside a polling helper loop (`wait(...)`), (b) inside a
  fake server/callback that simulates work, or (c) a wait whose *purpose is the passage of time* (e.g. a grace period),
  written as a derived value from the library constant (e.g. `api.RECOVERY_GRACE_S + margin`) with a comment. All other
  fixed sleeps are replaced by `wait(<condition>)` on an observable condition (message received, service ready,
  state changed).
- The three packages' tests pass 3 times in a row (`colcon test` ×3), no flakes.
- Total test wall time for the three packages does not grow (report before/after).
- Commit(s) on `lead/T023-test-waits`; first commit sets `status: claimed`, last sets `status: review`.

## Inputs / contracts used
- `src/swarmflow_core/swarmflow_core/api.py` constants (read-only)
