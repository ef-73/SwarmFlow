---
id: T013
title: README draft (Lead-delegated)
workstream: Lead
status: open
claimed_by: "sf-implementer"
branch: lead/T013-readme
model: sonnet
risk: low
---

## Goal
A `README.md` a newcomer on Windows can follow to run SwarmFlow v1 with one command, accurate to the code on `main`.

## Owned files
- README.md

## Lead tests (do not edit)
- none (the lead reviews the text against the code and runs the fresh-clone test in M7)

## Forbidden
- AGENTS.md §4; do not edit any other file; never run Gazebo or `docker compose up`.
- Do not invent commands, env vars, ports or results: every one you mention must exist in `compose.yaml`,
  `docker/compose.yaml`, the launch files, `scripts/`, `tests/integration/`, `tools/metrics/`. Mark anything you could
  not verify with "(unverified)".

## Commands to run
```bash
grep -rn "SWARMFLOW_" docker/ src/*/launch tests/integration scripts | sort -u   # inventory of env vars
```

## Specification
Sections, in this order, concise (≈ 250–400 lines), GitHub Markdown:
1. **SwarmFlow** — one-paragraph identity (design §19) + v1 scope bullets (design §2.1/§4.1); badges none.
2. **Quick start (Windows)** — prerequisites (Windows 11, Docker Desktop with WSL2 backend, ~20 GB disk, an NVIDIA GPU
   recommended for the GUI, see design §8.4 spike outcome), then from PowerShell in the repo root:
   `docker compose up` (first run builds the images; give the measured build times from
   `docs/agent_log/2026-10-09T0905Z-claude-M1.md`). What appears (Gazebo window via WSLg; Foxglove at
   `ws://localhost:8765`). Stop with Ctrl+C / `docker compose down`. GUI routes table: `SWARMFLOW_GUI=wslg` (default),
   `vnc` (http://localhost:6080/vnc.html), `none`; `SWARMFLOW_GPU_ADAPTER`. How to set env vars in PowerShell
   (`$env:SWARMFLOW_GUI="vnc"; docker compose up`) and via a `.env` file.
3. **What you are looking at** — the standard layout (racks, 1.30 m one-way storage aisles = conflict zones, holds,
   L/D/P stations), the order → assign → hold → grant → aisle → deliver flow; link `docs/evidence/` screenshots.
4. **Foxglove** — import `viz/foxglove/swarmflow_v1.json`; topics shown.
5. **Configuration** — table of every `SWARMFLOW_*` variable used in `docker/compose.yaml` and the launch files with
   default and meaning (SCENARIO, POLICY, ROBOTS, RUN_ID, LAYOUT, LOCALIZATION, AGENT, TRAFFIC_CONTROL, RTF, GUI,
   GPU_ADAPTER, GIT_SHA, IMAGE_DIGESTS, …). Baseline A = `SWARMFLOW_POLICY=independent` +
   `SWARMFLOW_TRAFFIC_CONTROL=false`.
6. **Experiments and metrics** — `runs/<run_id>/` contents (design §13.5), `tests/integration/demo_run.sh`,
   `tools/metrics/compute.py`, `tools/metrics/compare.py`; a placeholder `### Corridor scene results (n = 3)` with the
   text "Filled in by the lead after M7." — the lead adds the table.
7. **Architecture** — a Mermaid diagram: Gazebo server (+GUI) ↔ per-robot containers (Nav2 + robot agent + bridges) ↔
   orchestrator (FleetCore + FCFS + reservation authority) ↔ scenario engine; payload and viz nodes; Foxglove bridge;
   one sentence per component; link design §6.
8. **Development** — the `dev` container commands from AGENTS.md §6, `scripts/ci.sh`, contracts/workstreams/task cards
   (link AGENTS.md, docs/design.md, docs/milestones.md, docs/tasks/), the sim lock rule.
9. **Known limitations (v1)** — ground-truth localization (docs/decisions/001), front 180° LiDAR (design §7.1 as built),
   packages are kinematic (no contact physics), one payload size, no workers/closures (v2), FCFS has no
   starvation guard (v2), Foxglove layout not yet verified by import (if still true), RMF only in v2.
10. **Repository layout** — short tree of top-level dirs with one line each.
11. **License** — "Apache-2.0 (see package.xml files)".

## Definition of done
- `README.md` written per the spec, every command/env var checked against the repo. Commit "T013: README draft",
  card status `review`. Report the list of anything marked "(unverified)".

## Inputs / contracts used
- docs/design.md §1, §2, §4, §6, §8.3, §8.4, §11.1, §13.5, §15.4, §19; docs/agent_log/*M1*, *M4*, *M5*;
  docs/decisions/001-ground-truth-localization.md; docker/compose.yaml; compose.yaml
