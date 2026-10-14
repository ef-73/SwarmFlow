---
id: T043
title: Manual orders from the command line (O-07)
workstream: F
status: open
claimed_by: "sf-implementer"
branch: ws-f/T043-order-cli
model: sonnet
risk: low
---

## Goal
An operator can add an order by hand while a run is going (improvements_v2 O-07): a CLI that validates the order
against the layout and publishes it once on `/fleet/orders`. (The Foxglove Publish preset is the lead's part.)

## Design (lead)
- `tools/scenarios/scenarios/order_cli.py`, run as `python3 -m scenarios.order_cli ...` (usage in the lead test
  docstring) inside a robot or orchestrator container.
- Pure part (tested without ROS): `parse_args(argv)` (argparse, sub-commands shelf / pallet / transport;
  `--priority` default 0, `--id` default `manual_<UTC yyyymmddHHMMSS>_<4 random hex>`, `--layout-dir` default
  `/ws/layouts/$SWARMFLOW_LAYOUT/generated`), and `build_order(args, graph, now) -> api.OrderSpec` that checks
  vertex kinds (`pick`, `dock`, `pallet`, `loading`, `delivery`) and distinct picks, raising `ValueError` with a
  message naming the problem (the lead test matches on it).
- ROS part (`main()`): rclpy node with `use_sim_time` true, publisher on `/fleet/orders` with the orchestrator's QoS
  (reliable, depth 100), waits up to 5 s for a subscriber, publishes the `Order` (release_time = now, order_type via
  `api.ORDER_TYPE_CODES`), prints the order id, exits 0; exits 2 with the message on a `ValueError`.
- One line in `tools/scenarios/README.md` (create it if missing) with the usage.

## Owned files
- tools/scenarios/scenarios/order_cli.py
- tools/scenarios/README.md
- tools/scenarios/tests/test_order_cli_extra.py

## Lead tests (do not edit)
- tools/scenarios/tests/test_order_cli.py

## Forbidden
- AGENTS.md section 4; no Gazebo; do not change the generator or the scenario engine.

## Commands to run
```bash
export PATH="/c/Program Files/Docker/Docker/resources/bin:$PATH" COMPOSE_PROJECT_NAME=swarmflow-t043
docker compose run --rm --no-deps dev python3 -m pytest -q tools/scenarios
```

## Definition of done
- Lead test and all tools/scenarios tests pass; `scripts/ci.sh` green on the branch rebased onto `main`.

## Inputs / contracts used
- `swarmflow_core.api` (OrderSpec, ORDER_* and ORDER_TYPE_CODES), `Order.msg`, `swarmflow_core.graph`,
  scenario_engine.py (publishing pattern and QoS).
