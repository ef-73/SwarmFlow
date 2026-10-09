---
id: T002
title: Graph loader and queries (swarmflow_core.graph)
workstream: B
status: done
claimed_by: "claude-lead"
branch: lead/T002-graph
model: opus
risk: normal
---

## Goal
`swarmflow_core.graph` loads the generated nav graph + sidecar and implements `api.Graph`. Every lane depends on it,
so the lead implements it directly (no subagent wait chain).

## Owned files
- src/swarmflow_core/swarmflow_core/graph.py
- src/swarmflow_core/test/test_graph.py

## Lead tests (do not edit)
- src/swarmflow_core/test/test_graph.py

## Forbidden
- AGENTS.md §4; no ROS imports.

## Commands to run
```bash
docker compose run --rm -T dev python3 -m pytest -q -p no:cacheprovider src/swarmflow_core/test
```

## Definition of done
- Tests pass; `scripts/ci.sh` green.

## Inputs / contracts used
- docs/design.md §6.1; api.py; tests/fixtures/standard/
