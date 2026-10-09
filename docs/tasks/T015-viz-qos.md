---
id: T015
title: viz node — /fleet/robot_states subscription must be volatile
workstream: E
status: done
claimed_by: "sf-mechanical"
branch: ws-e/T015-viz-qos
model: haiku
risk: low
---

## Goal
In the M6 demo the viz node received no robot states: it subscribes `/fleet/robot_states` with TRANSIENT_LOCAL
durability, but the publishers (robot agents) are VOLATILE, so DDS refuses the match ("incompatible QoS ...
DURABILITY"). The T009 card was wrong here (lead error). Fix the subscription QoS.

## Owned files
- src/swarmflow_viz/swarmflow_viz/viz_node.py
- src/swarmflow_viz/test/test_viz_qos.py

## Lead tests (do not edit)
- none

## Forbidden
- AGENTS.md §4; change nothing else.

## Commands to run
```bash
export MSYS_NO_PATHCONV=1 PATH="/c/Program Files/Docker/Docker/resources/bin:$PATH"
export COMPOSE_PROJECT_NAME=swarmflow-t015
scripts/lock.sh acquire build T015 20     # wait/retry if held; release afterwards
docker compose run --rm -T dev colcon build --symlink-install --parallel-workers 2 --packages-up-to swarmflow_viz
docker compose run --rm -T dev python3 -m pytest -q -p no:cacheprovider src/swarmflow_viz/test
scripts/lock.sh release build
```

## Definition of done
- `/fleet/robot_states` subscription: reliable, **volatile**, depth 10. `/fleet/reservations` stays transient-local
  reliable depth 100.
- New test `src/swarmflow_viz/test/test_viz_qos.py`: a publisher with *default* QoS (volatile, reliable, depth 10)
  on `/fleet/robot_states` publishes 2 RobotStates (robot_1, robot_2) → the node publishes a MarkerArray on
  `/fleet/robot_markers` containing 2 markers with ns "robots" (copy the pattern of `test_viz_node.py`).
- All viz tests pass. Commit "T015: viz robot_states QoS", card status `review`. Report the exact test output.
