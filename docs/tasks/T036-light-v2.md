---
id: T036
title: Light mode for v2 - graph-path expansion of goals, deadlock breaker, v2 demo end to end
workstream: C
status: open
claimed_by: "sf-implementer"
branch: ws-c/T036-light-v2
model: sonnet
risk: normal
---

## Goal
Light mode runs the v2 loop (`policy: assign`, goal tasks, `warehouse_v2`, 5 robots) so assignment can be watched
and tuned without Gazebo (improvements_v2 M11/M15). Today a v2 goal task gives the kinematic robot one pose per goal,
and it would drive straight through racks; and dense free-space traffic can deadlock (T025 report).

## Design (lead)
- `sim2d/sim2d/light_paths.py`: `expand(graph, start_xy, goal_xy) -> list[(x, y)]`: the light-mode stand-in for
  Nav2's planner. If the straight segment start → goal is short (≤ 1.0 m) return `[goal]`; else go to the nearest
  graph vertex, follow `graph.shortest_path` to the vertex nearest the goal (skip vertices already behind), end at
  the goal. Deterministic. Used by the light navigation server for every received pose (v1 waypoint tasks keep
  working: consecutive graph vertices expand to themselves).
- `LightWorld` deadlock breaker for free space: a robot blocked for more than a few seconds by robots that are
  themselves blocked or idle yields deterministically (e.g. the lower-priority robot by id backs off or side-steps
  to free space, then retries); never creates an overlap (keep the hard layer). The existing lead tests
  (`test_light_world.py`) must still pass.
- `sim2d/sim2d/light.py`: nothing else should be needed for v2 (policy comes from the scenario); verify and fix if not.
- `sim2d/README.md`: v2 usage + CPU for 5 robots.

## Owned files
- sim2d/sim2d/**
- sim2d/tests/** (except the lead tests)
- sim2d/README.md
- docker/compose.light.yaml

## Lead tests (do not edit)
- sim2d/tests/test_light_v2.py
- sim2d/tests/test_light_world.py
- sim2d/tests/test_light_ros.py

## Forbidden
- AGENTS.md §4; no Gazebo; no edits to `src/**`.

## Commands to run
```bash
export PATH="/c/Program Files/Docker/Docker/resources/bin:$PATH" COMPOSE_PROJECT_NAME=swarmflow-t036 SWARMFLOW_GUI=none
docker compose run --rm --no-deps dev python3 -m pytest -q sim2d/tests/test_light_v2.py -k "not light_mode" sim2d/tests/test_light_world.py
scripts/lock.sh acquire build T036 30 && docker compose run --rm dev bash -c "colcon build --symlink-install --parallel-workers 2 && source install/setup.bash && python3 -m pytest -q sim2d"; scripts/lock.sh release build
```

## Definition of done
- All sim2d tests pass, `test_light_v2.py::test_v2_demo_in_light_mode` 3 runs in a row.
- README: CPU of a 5-robot v2 run at rtf 1 (process_time / sim time).
- `scripts/ci.sh` green on the branch rebased onto `main`.

## Inputs / contracts used
- `swarmflow_core.graph` (shortest_path, nearest_vertex), `layouts/warehouse_v2`, `scenarios/v2_demo.yaml`, T025 code.
