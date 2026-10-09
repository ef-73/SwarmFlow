---
id: T008
title: Scenarios, seeded order generator, scenario_engine node
workstream: F
status: open
claimed_by: "sf-implementer"
branch: ws-f/T008-scenarios
model: sonnet
risk: normal
---

## Goal
`scenarios/v1_demo.yaml` and `scenarios/v1_corridor.yaml`, a pure-Python seeded order generator in
`tools/scenarios/`, and the `scenario_engine` ROS node (`src/swarmflow_scenarios/`) that publishes the orders in sim
time and creates the run record (design §12.3, §13.5).

## Owned files
- tools/scenarios/**
- scenarios/**
- src/swarmflow_scenarios/**

## Lead tests (do not edit)
- tools/scenarios/tests/test_scenarios.py

## Forbidden
- AGENTS.md §4; no Gazebo; do not edit `swarmflow_core`, contracts, fixtures.

## Commands to run
```bash
export MSYS_NO_PATHCONV=1 PATH="/c/Program Files/Docker/Docker/resources/bin:$PATH"
export COMPOSE_PROJECT_NAME=swarmflow-t008
docker compose run --rm -T dev python3 -m pytest -q -p no:cacheprovider tools/scenarios
scripts/lock.sh acquire build T008 20     # only for the ROS package; release afterwards
docker compose run --rm -T dev colcon build --symlink-install --parallel-workers 2
docker compose run --rm -T dev python3 -m pytest -q -p no:cacheprovider src/swarmflow_scenarios/test
```

## Specification
**Scenario YAML** (fields; validate on load, `ValueError` with a clear message on any error):
`name`, `seed` (int), `duration_s`, `layout`, `policy` (`fcfs`|`independent`), `robots` (list of robot ids), `orders`:
either `{mode: rate, rate_per_min: <float>, pickups: [L..], dropoffs: [D..]}` (Poisson arrivals with
`random.Random(seed)`, uniform pickup/dropoff choice) or `{mode: explicit, list: [{t, pickup, dropoff}, ...]}`
with optional `repeat_every_s` (the list is repeated with that period until `duration_s`).
- `v1_demo.yaml`: seed 1, 600 s, standard, fcfs, robots robot_1..3, rate mode, all L/D stations, a rate that yields
  ≥ 10 deliveries in the fake backend (see lead test) without flooding (≤ 60 orders).
- `v1_corridor.yaml`: seed 1, `duration_s` 300, standard, fcfs, 2 or 3 robots, explicit orders with
  `repeat_every_s: 60` chosen so that two robots enter the **same storage aisle from opposite ends within 5 s** each
  cycle. Use `swarmflow_core.backends.fake.run_fake` (as the lead test does) to design the timing; the lead will
  re-tune it in Gazebo (M7). Explain the choreography in a YAML comment.

**`tools/scenarios/scenarios/`** (package, plus `tools/scenarios/pyproject.toml`, name `swarmflow-scenarios`):
`Scenario` dataclass (fields above; `orders` kept as a dict), `load_scenario(path)`,
`generate_orders(scenario, graph) -> list[api.OrderSpec]` — ids `o0001…` in release order, releases sorted,
`< duration_s`, payload `small`, no deadline, priority 0; reject pickups that are not `loading` vertices or dropoffs
that are not `delivery` vertices. Deterministic.

**`src/swarmflow_scenarios/`** (ament_python): node `ScenarioEngineNode(**kwargs)` named `scenario_engine`,
parameters `scenario_file`, `layout_dir`, `run_dir` (`""` = no files), `git_sha` (""), `image_digests` (""),
`start_delay_s` (5.0). It waits until the node clock is non-zero (sim time running), then
publishes each order on `/fleet/orders` (reliable, depth 50) when `now − start ≥ release_t + start_delay_s`
(`release_time` = that time). With `run_dir`: writes `config.yaml` (resolved scenario + git_sha + image_digests +
start time) once, and appends one JSON line per published order to `events.jsonl`. Console entry point
`scenario_engine`. A launch file `launch/scenario_engine.launch.py` with args `scenario` (name, env
`SWARMFLOW_SCENARIO`, default v1_demo), `scenarios_root` (/ws/scenarios), `run_id`, `runs_root` (/ws/runs) using the
same run-id rule as the orchestrator: env `SWARMFLOW_RUN_ID`, else `f"{scenario}-{policy}-latest"`; `use_sim_time:
true`. The scenario engine imports the generator from `tools/scenarios` (add it to `sys.path` from the launch-provided
`scenarios_tools` parameter, default `/ws/tools/scenarios`).
- Add a ROS test (wall clock) for the node: a tiny scenario with 2 explicit orders → both arrive on `/fleet/orders`
  in order; `config.yaml` and `events.jsonl` written.

## Definition of done
- Lead tests pass unchanged; your own tests pass; commit "T008: scenarios", card status `review`. Report test output
  and the corridor choreography.

## Inputs / contracts used
- docs/design.md §12.3, §13.4, §13.5; api.py; swarmflow_core (graph, fleet, backends.fake, reservations).
