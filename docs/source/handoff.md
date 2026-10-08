# Handoff brief: SwarmFlow design doc v2.0

Written 2026-10-08 by a Claude session that reviewed the v1.1 design (`swarmflow_v1.1_original.md`)
with the user. Everything below is **approved by the user** unless marked otherwise. Your job is to
turn the v1.1 doc plus these decisions into `docs/design.md` (v2.0) and `AGENTS.md`.

## User decisions (final)

- Format: Markdown in this repo (`docs/design.md`, `AGENTS.md`; `CLAUDE.md` just points to `AGENTS.md`).
- Robot: **custom basic differential-drive chassis** — simple, just enough to get things done.
- Dashboard: Foxglove (+ RMF web dashboard) for v1; custom React dashboard only in v2.
- Development: Claude and ChatGPT/Codex agents running **24/7** in parallel; the user reviews/merges.
- Fleet layer: **Open-RMF for v1**, own orchestrator in v2, RMF kept as a benchmark baseline.
- **v1 must be done in 5 days. Optimize for speed.** v1's purpose: a working Docker + ROS 2 demo.

## Approved changes to the v1.1 doc

### Scope and structure
1. Replace the 9 phases with **v1 (5-day MVP)** and **v2 (research extension)**. v1 is a complete, shippable project on its own.
2. Cuts: live fleet expansion (cut); package "descend" animation (stretch); optimization modes kept but described honestly as weight presets; deterministic replay only in the 2D backend.
3. **Open-RMF section** (replaces the original "should we use RMF" question):
   - RMF does not navigate. It is a fleet layer (task dispatch/bidding, traffic schedule + negotiation, lane closures, web dashboard) on top of each robot's own nav stack, connected via a fleet adapter (free_fleet supports Nav2).
   - SwarmFlow has **no custom navigation**: Nav2 does planning, control and local avoidance for every robot in both v1 and v2. Custom work is the fleet layer, Nav2 configuration (footprints, planners, keepout filters), and glue.
   - v1 uses RMF to get dispatch, traffic control and a dashboard for free.
   - v2 replaces RMF's fleet layer (free_fleet → our robot agent, RMF core → our orchestrator) because the v2 features are exactly what RMF cannot be extended with cheaply: custom reservation/predictive logic (would mean rewriting RMF's C++ traffic core), payload-dependent footprints (RMF uses a fixed per-fleet footprint), and fast non-real-time 2D benchmarking.
   - RMF becomes **Baseline C** in v2 benchmarks.
   - The nav graph uses **RMF's format from day one** so both fleet layers read the same graph; RMF's building-map tools may generate the Gazebo world + nav graph.
   - Go/no-go gate (see timeline): if RMF + free_fleet + our Nav2 robots don't work by the gate, v1 falls back to our own FCFS orchestrator, which agents build in parallel from Day 1 anyway.
   - **Verify before writing as fact:** Open-RMF Jazzy binary availability, free_fleet's current Nav2 support (it was rewritten around Zenoh/EasyFullControl), rmf_demos Gazebo Harmonic support. Cite what you find.
4. A **Non-goals** list consolidating the scattered disclaimers (no certified safety, no contact physics, not recreating Amazon's systems, no custom navigation).

### Architecture
5. **Warehouse topology graph** is the core shared abstraction: nodes = stations/intersections, edges = corridors, conflict zones attached to graph elements. One layout definition generates world, Nav2 map, nav graph, and 2D sim layout.
6. **Robot agent (fleet adapter) per robot** is the only bridge from fleet layer to Nav2: graph route → `NavigateThroughPoses` segments; holds at zone-entry nodes until reservation granted. (v1: free_fleet plays this role; v2: ours.)
7. Corridor closures via **Nav2 keepout costmap filters** so the local planner agrees with the fleet layer (v1: RMF lane closures).
8. **Reservation protocol** fully specified for v2: request/grant/deny/release messages, time-limited leases + heartbeats, release on zone exit, defined behaviour when the orchestrator is down (hold safely → timeout → report).
9. **`swarmflow_interfaces` package frozen on Day 1** with concrete msg/srv/action definitions (orders, robot state, payload state, decision events, scenario events, reservations).
10. **Orchestrator = pure-Python library** (no ROS imports) + thin ROS and 2D-sim adapters. Unit-testable from day one; the fake adapter grows into the 2D backend.

### Simulation / Nav2
11. Pick the custom chassis first (~0.6 × 0.5 m differential drive, 2D LiDAR, flat platform, ID marker), then a concrete table of robot dims → payload dims → aisle widths per layout, designed so "wide payload is infeasible in dense aisles" is true by construction.
12. Footprint-aware planning (v2): SmacPlannerHybrid + MPPI with full footprint checking; runtime footprint updates via each costmap's `footprint` topic.
13. Packages via a **pose-follower node** (package model tracks robot pose while loaded), not runtime-spawned joints.
14. Workers start as moving primitive shapes (guaranteed LiDAR-visible); animated actors only if verified visible to the LiDAR.
15. Multi-robot bringup starts from Nav2's multi-robot example with composition enabled; doc states the TF namespacing choice explicitly.

### Docker / environment
16. Day-1 environment spike: Gazebo server headless in a container, GUI on host/WSLg (user is on **Windows 11 + Docker Desktop/WSL2**), `/clock` bridge + `use_sim_time` everywhere, `rmw_zenoh` or DDS with shared memory disabled, and a "hello multi-container" test that fails if topics are listed but no data flows.
17. Images: `dev` (build + unit tests, no GPU — what agents and CI use), `sim` (Gazebo), one shared robot image configured by env vars.

### Evaluation
18. Baseline A (independent Nav2) gets a stuck-timeout rule → order scored as failed.
19. Gazebo results reported over n runs with confidence intervals; the congestion tipping-point claim comes only from the 2D backend and must agree with Gazebo at 3 robots.
20. Collision metric = pairwise footprint distance checks, not physics contacts.

### Tooling
21. Foxglove (via `foxglove_bridge`) for map/paths/markers/plots in v1; custom React panel for orders, explanations, stress-test buttons in v2.
22. Decision logging and Foxglove view are **v1** features (needed for debugging).

### Multi-agent development (new section in design.md + basis for AGENTS.md)
23. **Contract-first**: Day 1 freezes interfaces package, layout/graph schema (RMF nav-graph format), orchestrator library API (Python protocols), mock fixtures (recorded robot-state streams).
24. Workstreams, each with own branch, owned folders and acceptance tests:
    - A: Docker, Gazebo world, custom chassis, Nav2 bringup (needs a human watching the sim)
    - B: Orchestrator library — FCFS fallback in v1, predictive in v2 (pure Python, ideal for agents)
    - C: 2D kinematic sim + benchmark harness (agents)
    - D: Robot agent / RMF fleet adapter integration (integration with A)
    - E: Foxglove layouts + v2 web panel/telemetry API on mock data (agents)
    - F: Scenario/order generator, layout generator (agents)
25. Single `AGENTS.md` (Codex reads it natively; `CLAUDE.md` points to it). Every agent task states: owned files, forbidden actions (no merges/pushes to `main`, no contract edits, no installs outside Docker), commands to run, definition of done.
26. CI is the referee: `colcon build` + `pytest` in the `dev` image on every PR, plus an interface-diff check. The user merges; agents never do.
27. Integration is the bottleneck: the user + a lead agent own merges and Gazebo/Nav2 debugging; agents take everything that runs without the simulator.
28. 24/7 guardrails: **one Gazebo simulation at a time** across all agents (file lock, e.g. `/tmp/swarmflow-sim.lock` or a lock file in the repo's `.run/`). The user's machine has a CPU thermal limit, so heavy sims must never overlap; agents default to unit tests and the 2D sim. Each agent keeps an append-only task log (`docs/agent_log.md`) the user reviews in batches.

## 5-day v1 plan (proposed by the lead session — present in design.md as the plan)

v1 scope, deliberately minimal: **standard layout only, 3 custom robots, one small payload size (constant footprint), no workers**, RMF dispatching delivery tasks, a narrow-corridor traffic scene, Foxglove + RMF dashboard, one-command `docker compose up`.

| Day | Critical path (user + lead agent) | Parallel agent lanes |
|---|---|---|
| 1 | Docker/zenoh spike; freeze contracts (interfaces, graph schema, orchestrator API) | A: chassis URDF/xacro + single robot Nav2 in world · D: RMF + free_fleet stock example running in Docker · B: interfaces pkg + FCFS orchestrator lib + tests · F: layout → world + nav graph generator, order generator |
| 2 | 3 namespaced Nav2 robots navigating in Gazebo. **Go/no-go at end of day:** RMF dispatches one delivery to one of our robots through free_fleet | E: Foxglove layout · B: own robot-agent adapter (fallback) · F: package pose-follower node |
| 3 | End-to-end deliveries with 3 robots, packages visible (RMF or fallback) | C: 2D sim skeleton on orchestrator lib (v2 seed) · E: decision-event log → Foxglove |
| 4 | Narrow-corridor scene: Baseline A (independent Nav2 + stuck timeout) vs RMF traffic; one-command compose | CI pipeline · small n metrics script (deliveries, wait time, stuck events) · README draft |
| 5 | Buffer + hardening; record demo video; freeze v1 tag | docs, architecture diagram, cleanup |

Realism notes to keep in the doc: Day 2 (multi-robot Nav2 namespacing) is the highest schedule risk; Day 5 is buffer, not feature time. If the RMF gate fails, the fallback keeps the 5-day target only because lane B was built in parallel.

**v2** (after v1, ~2–3 weeks with 24/7 agents, bounded by user review/integration): own orchestrator replaces RMF's fleet layer; payload footprints + Smac/MPPI; workers + closures via keepout filters; forecasting, rolling horizon, deadlines; stress-test injection; 2D benchmarks across layouts/fleet sizes with RMF as Baseline C; React panel.

## Writing instructions

- Keep the v1.1 doc's good material (problem statements, explainable-decision events and examples, metrics, stress-test events, demo concept, project identity), restructured around v1/v2.
- Every section should be actionable for an agent: concrete names, file paths, message fields, acceptance criteria.
- Mark anything not yet verified as such; do not invent package names or API details.
