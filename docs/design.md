# SwarmFlow
## Intelligent Warehouse Fleet Coordination & Digital Twin
**Technical Design Document — Version 2.0**

| | |
|---|---|
| **Status** | **Final plan for v1** (2026-10-09): review items R1–R16 applied (`docs/milestones.md` §1); host Docker engine pending (D3, §8.0) |
| **Supersedes** | v1.1 ([`docs/source/swarmflow_v1.1_original.md`](source/swarmflow_v1.1_original.md)) |
| **Change basis** | [`docs/source/handoff.md`](source/handoff.md) — approved changes C1–C28 (traceability in [Appendix A](#appendix-a--approved-change-traceability-c1c28)) |
| **Stack** | ROS 2 Jazzy · Gazebo Harmonic · Nav2 · Docker Compose · Python · Foxglove · Open-RMF (v2 Baseline C only) |
| **Decisions after handoff** | **D1 (2026-10-08, user):** v1 uses SwarmFlow's own lightweight FCFS orchestrator + robot agent instead of Open-RMF; RMF moves to v2 as Baseline C. Supersedes the v1 parts of C3 and the RMF items of the handoff's 5-day plan. **D2 (2026-10-08, user):** the Gazebo GUI runs in a container and must be startable on Windows (§8.4). |
| **Host** | Windows 11 + Docker Desktop (WSL2 backend); CPU has a thermal limit (one Gazebo sim at a time) |
| **Agent rules** | [`AGENTS.md`](../AGENTS.md) |

**Verification legend.** Statements about third-party software are tagged:
**[V]** verified against a cited source on 2026-10-08 · **[U]** unverified — must be checked by the
named workstream before code depends on it · untagged = SwarmFlow design decision.
Sources are listed in [Appendix B](#appendix-b--sources).

---

# 1. Vision and Central Question

SwarmFlow is a containerized multi-robot warehouse simulation and digital twin for studying fleet
coordination. Differential-drive robots navigate a shared 3D warehouse in Gazebo, collect packages at
loading stations and carry them to delivery stations. A fleet layer above the robots decides who
carries what, and who may enter which corridor when.

**Central question (research, v2):** *Can predictive, deadline-aware traffic coordination improve
warehouse throughput, reduce congestion and respond better to disruptions than independent robot
navigation and than an off-the-shelf fleet manager (Open-RMF)?*

The project ships in two versions:

| | **v1 — 5-day MVP** | **v2 — research extension** |
|---|---|---|
| Purpose | A working, one-command Docker + ROS 2 + Gazebo + Nav2 multi-robot demo | Answer the central question with measured results |
| Fleet layer | **Own lightweight orchestrator** (FCFS assignment + FCFS exclusive zone reservations) + **own robot agent** per robot (D1) | Same orchestrator, **predictive** policy; **Open-RMF** (via free_fleet) runnable as **Baseline C** |
| Navigation | Nav2 per robot (both versions — SwarmFlow writes **no** custom navigation) | Nav2 + footprint-aware planning (Smac Hybrid-A* + MPPI) |
| Scope | Standard layout, 3 robots, one small payload, no workers | 3 layouts, 4 payload types, workers, closures, stress tests, 2D benchmarks |
| UI | Gazebo GUI (containerized) + Foxglove | + custom React panel, RMF dashboard for Baseline C |
| Duration | 5 days (hard) | ~2–3 weeks with 24/7 agents, bounded by user review/integration |

v1 is a complete, shippable project on its own (C1). v2 builds on it; nothing in v2 is required for v1.

# 2. Goals and Non-goals

## 2.1 v1 goals (all must hold on the v1 tag)

1. `docker compose up` (one command from a Windows terminal, documented in `README.md`) starts Gazebo
   (server + GUI, both in containers), 3 robots with Nav2, the orchestrator and the Foxglove bridge.
2. Three custom robots complete pickup → delivery orders in the **standard** layout, with packages
   visibly riding on the robot.
3. A narrow-corridor traffic scene shows coordinated passage (FCFS reservations, Baseline B) versus
   Baseline A (independent Nav2 with stuck timeout), measured over a small *n*.
4. Decision events are logged and visible in Foxglove (C22).
5. CI builds and unit-tests every PR in the `dev` image.

## 2.2 v2 goals

1. Predictive policy on the v1 orchestrator and reservation protocol (§6.5); Open-RMF integrated as Baseline C.
2. Payload-dependent footprints and routes; "wide payload is infeasible in dense aisles" true by construction (§7.2).
3. Predictive coordination: congestion forecasting, rolling-horizon replanning, deadline-aware assignment (§9).
4. Workers, corridor closures (keepout filters), stress-test injection (§12).
5. 2D kinematic backend and benchmarks across layouts, fleet sizes and workloads, against Baselines A, B, C (§13).
6. React operations panel (§11.2).

## 2.3 Non-goals (C4)

- **No custom navigation.** Nav2 does global planning, control and local avoidance for every robot in v1 and v2.
  SwarmFlow's custom work is the fleet layer, Nav2 configuration (footprints, planner/controller choice,
  keepout filters) and glue.
- **No certified safety.** Simulated human avoidance is not a safety system and must not be described as one.
- **No contact physics for packages.** Packages follow the robot kinematically (§7.4); no grasping or friction.
- **Not a recreation of Amazon's (or any vendor's) warehouse software**, and no claim to have solved fleet congestion in general.
- **No live fleet expansion** (spawning robots mid-run) — cut (C2).
- **No bit-identical replay in Gazebo.** Deterministic replay exists only in the 2D backend (C2).
- **No real hardware**, no ML-based prediction, no decentralized coordination (see §19 Future extensions).
- **No localization research.** AMCL (or ground-truth localization as fallback) is used as-is.

# 3. Core Warehouse Problems

These are the problems v2 measures; v1 demonstrates the first and (narrowly) the fourth.

| Problem | Description | SwarmFlow response | Version |
|---|---|---|---|
| **Fleet congestion** | Robots independently choosing shortest routes create bottlenecks that cut throughput. | Conflict-zone reservations, coordinated waiting, direction-aware scheduling, traffic-aware assignment, predictive corridor utilization. | v1 (FCFS), v2 (predictive) |
| **Dynamic operations** | Workers crossing routes, temporary aisle blockages, slow robots, occupied loading stations, order bursts invalidate plans. | Detect and adapt without thrashing valid plans (replan hysteresis, §9.1). | v2 |
| **Delivery deadlines** | Operations care about orders fulfilled on time, not per-robot distance. Orders have release time, pickup, destination, priority, deadline. | Assignment on estimated completion time incl. predicted congestion and commitments. | v2 |
| **Dense layouts** | Tightly packed racks leave little manoeuvring space and create shared bottlenecks. | Test whether predictive coordination matters more as density and utilization rise. | v1 (standard), v2 (open/standard/dense) |
| **Payload-dependent navigation** | A loaded robot's effective footprint changes aisle clearance, turning feasibility and permissible routes. | Per-payload footprints, per-edge feasibility, runtime Nav2 footprint updates. | v2 |

# 4. Scope: v1 vs v2

## 4.1 v1 scope (deliberately minimal)

- **Standard layout only**, generated from one layout definition (§6.1).
- **3 custom robots** (§7.1), one shared robot image.
- **One payload size — small (0.40 × 0.40 m)**, which fits inside the chassis outline, so the Nav2 footprint is **constant** in v1.
- **No workers.**
- **Fleet layer: SwarmFlow orchestrator** — FCFS assignment (nearest idle robot) + FCFS exclusive zone
  reservations — and one **SwarmFlow robot agent** per robot (D1, §5.2). No Open-RMF in v1.
- **Narrow-corridor traffic scene**: Baseline A (independent Nav2) vs Baseline B (FCFS reservations).
- **Gazebo GUI in a container** (§8.4) **+ Foxglove** (map, robots, paths, zones, decision events, plots).
- **One-command `docker compose up`**, runnable from Windows.
- Decision logging (C22), CI (C26), small-*n* metrics script.

## 4.2 Cuts and honest reframings (C2)

| v1.1 feature | v2.0 decision |
|---|---|
| Live fleet expansion stress event | **Cut.** Fleet size is fixed per run. |
| Package "descend onto platform" animation | **Stretch** (v2, only if cheap). v1: package appears on the platform at the loading pose. |
| Throughput / Deadline / Balanced optimization modes | **Kept, described as what they are: weight presets** on the v2 cost function (§9.5). No mode relaxes safety constraints. |
| Record and replay of experiments | Event timelines are recorded for both backends; **deterministic replay only in the 2D backend**. Gazebo re-runs the same timeline but is not bit-identical (§12.3). |

## 4.3 v2 scope

Predictive policy on the v1 orchestrator; Open-RMF integrated as Baseline C; payload footprints +
Smac Hybrid-A*/MPPI; workers and closures via keepout filters; forecasting, rolling horizon, deadlines;
stress-test injection; 2D benchmarks across layouts and fleet sizes; React panel. Plan in §16.

# 5. Open-RMF and the Fleet Layer (C3)

## 5.1 What RMF is — and is not

Open-RMF **does not navigate robots.** It is a *fleet layer*: task dispatch and bidding, a traffic schedule with
conflict negotiation, lane closures, and a web dashboard, sitting on top of each robot's own navigation stack.
RMF talks to robots through a **fleet adapter**. For Nav2 robots, **free_fleet** provides that adapter **[V]** [S4].

```
            ┌──── v1 and v2 (SwarmFlow path) ─────────────────┐   ┌──── v2 Baseline C only ─────────────┐
fleet layer │ swarmflow orchestrator (pure-Python lib + ROS    │   │ RMF core (task dispatcher, traffic  │
            │ adapter): FCFS in v1, predictive in v2           │   │ schedule, negotiation) + rmf-web     │
            ├──────────────────────────────────────────────────┤   ├─────────────────────────────────────┤
adapter     │ swarmflow robot agent (one per robot)            │   │ free_fleet (zenoh bridge per robot)  │
            ├──────────────────────────────────────────────────┴───┴─────────────────────────────────────┤
navigation  │ Nav2 per robot (planner, controller, local avoidance, costmaps, keepout filter) — always     │
            ├───────────────────────────────────────────────────────────────────────────────────────────────┤
simulation  │ Gazebo Harmonic: headless server container + GUI container + ros_gz bridges                  │
            └───────────────────────────────────────────────────────────────────────────────────────────────┘
```

## 5.2 Why not RMF in v1 (decision D1)

The handoff planned RMF for v1 to get dispatch, traffic control and a dashboard for free. The verification below
showed the integration is not free: free_fleet is source-only, needs a separate zenoh bridge per robot plus a zenoh
router, forces Cyclone DDS, and expects a non-namespaced robot layout that our 3-robots-in-one-Gazebo setup only
matches in an upstream "testing-only" mode. RMF delivery tasks also assume workcell plugins we don't have.

v1's fleet layer is small enough to own: nearest-idle-robot assignment, exclusive FCFS leases on one-way aisles, and a
robot agent that turns a graph route into `NavigateThroughPoses` segments. That is pure-Python, agent-friendly work
(lane B was building it from Day 1 as the fallback anyway), and it is exactly what v2 extends. So the user chose
(2026-10-08) to make it the v1 path and remove the go/no-go RMF gate. What v1 gives up: RMF's web dashboard and traffic
negotiation — neither is needed for the v1 demo.

## 5.3 RMF in v2: Baseline C

RMF (with free_fleet) is integrated in v2 as **Baseline C** for Gazebo benchmarks (WS-D, §16). The findings in §5.5 are
the starting point for that work. SwarmFlow does not *extend* RMF because the v2 features are exactly what RMF cannot
be extended with cheaply:

| v2 need | Why RMF does not fit cheaply |
|---|---|
| Custom reservation / predictive traffic logic | Would mean modifying RMF's C++ traffic schedule and negotiation core. |
| Payload-dependent footprints | RMF uses a fixed per-fleet footprint (vehicle profile) **[U — confirm in `rmf_fleet_adapter` config schema, WS-D]**. |
| Fast, non-real-time 2D benchmarking (hundreds of runs) | RMF runs against ROS time with real nodes; not designed as a faster-than-real-time library. |

For Baseline C runs, free_fleet takes the robot agent's place and RMF core takes the orchestrator's place; Nav2,
the world and the robots are unchanged (§13.3).

## 5.4 Shared nav graph

The nav graph uses **RMF's nav-graph format from day one** (kept after D1), so Baseline C in v2 reads the same graph
as SwarmFlow with no conversion. In v1 SwarmFlow's own layout generator writes that file directly (no RMF
dependency in v1, AGENTS.md §11); the exact field layout is copied from an `rmf_demos` (`jazzy` branch) nav graph
**[U — confirmed only when Baseline C loads our graph in v2]**. RMF's building-map tools (`rmf_building_map_tools`,
released for Jazzy **[V]** [S7]) are not used in v1. SwarmFlow-only data (conflict zones, payload feasibility) lives in a sidecar file keyed by nav-graph vertex/lane
names (§6.1), so the RMF graph file itself stays unmodified.

## 5.5 Verification findings (2026-10-08)

| Claim | Finding | Tag | Source |
|---|---|---|---|
| Open-RMF has Jazzy binaries | Jazzy is a supported distro; `sudo apt install ros-jazzy-rmf-dev` installs RMF (excluding demos). `rmf_fleet_adapter` 2.7.2 is **released** for Jazzy. | **[V]** | [S1], [S2] |
| rmf_demos supports Gazebo Harmonic | The `jazzy` branch states it was built and tested on Ubuntu 24.04, ROS 2 Jazzy and **Gazebo Harmonic**. `rmf_demos` 2.4.0 is **released** for Jazzy. `main` targets Kilted + Gazebo Ionic — **use the `jazzy` branch/binaries**, not `main`. | **[V]** | [S3], [S2b] |
| free_fleet supports Nav2 | Yes: Nav2 single- and multi-TurtleBot3 examples; CI has Nav2 integration tests. Supports **Jazzy** (and Rolling via `main`). Rewritten for Jazzy onwards around the EasyFullControl adapter and zenoh bridges. | **[V]** | [S4], [S5] |
| free_fleet transport | Uses **zenoh as an overlay** between robots and adapter: `zenoh-bridge-ros2dds` **v1.5.0** standalone binary + `zenohd` router + pip `eclipse-zenoh==1.5.0`, `nudged`, `pycdr2`, `rosbags`. It does **not** use `rmw_zenoh`. Recommended RMW is **`rmw_cyclonedds_cpp`**; "other RMW implementations have shown varying results". | **[V]** | [S4] |
| free_fleet robot topology | Intended design: **each robot's Nav2 is non-namespaced**, and its zenoh bridge carries the robot name as namespace. The multi-robot example (pre-namespaced robots, one non-namespaced bridge) is flagged "only for testing purposes". | **[V]** | [S4] |
| free_fleet packaging | **Not released as a binary**; source build required. README TODOs include "docker images", "releases", "test replanning", "map switching". | **[V]** | [S4] |
| rmw_zenoh on Jazzy | `rmw_zenoh_cpp` 0.2.11 released for Jazzy. | **[V]** | [S6] |
| RMF web dashboard in Docker | `ghcr.io/open-rmf/rmf-web/api-server:jazzy-nightly` (port 8000) and `…/demo-dashboard:jazzy-nightly` (port 3000); host networking "typically required". Nightly tags — pin a digest. | **[V]** | [S8] |
| Nav2 multi-robot bringup | `nav2_bringup` ships `unique_multi_tb3_simulation_launch.py` and `cloned_multi_tb3_simulation_launch.py`; composed bringup is the default (`use_composition`). | **[V]** | [S9] |
| Nav2 keepout filter | `nav2_costmap_2d::KeepoutFilter`, fed by a Costmap Filter Info Server + a map server publishing the mask; usable in global and local costmaps. | **[V]** | [S10] |
| foxglove_bridge on Jazzy | Listed with a release in the Jazzy rosdistro. | **[V]** (version not pinned) | [S11] |
| RMF delivery tasks with our robots | RMF "delivery" uses dispenser/ingestor workcells in `rmf_demos`; whether free_fleet supports the needed pickup/dropoff actions is **unknown**. | **[U]** | — |

**Consequences** — these drove decision D1 and now scope the v2 Baseline C work (also in §17 Risks):

1. **Namespacing tension.** Our layout is 3 namespaced Nav2 robots in one Gazebo (§7.6), which matches free_fleet's
   *testing-only* multi-robot mode, not its intended per-robot layout. The tested alternative is domain-isolated,
   non-namespaced robots (§7.6 Option D). WS-D must prove one of them before Baseline C runs.
2. **free_fleet expects Cyclone DDS.** v1 picks Cyclone DDS anyway (§8.1), so Baseline C needs no RMW change.
3. **Source builds inside Docker:** free_fleet + zenoh bridge binary download + pip deps go in the v2 `fleet` image.
4. **Delivery semantics:** Baseline C can use RMF *go-to-place* tasks; pickup/drop-off stays with SwarmFlow's package
   node (§7.4), so RMF workcells are not needed.

# 6. Architecture

## 6.1 Warehouse topology graph — the core shared abstraction (C5)

Everything that knows about the warehouse reads one **layout definition**:

```
layouts/<name>/layout.yaml          (source of truth, SwarmFlow schema — frozen in M2)
        │  tools/layoutgen (WS-F, pure Python, no RMF dependency)
        ├──► layouts/<name>/generated/world.sdf                   (Gazebo world: floor, walls, racks as solid boxes, stations)
        ├──► layouts/<name>/generated/nav_graph.yaml              (nav graph in RMF nav-graph format, §5.4)
        ├──► layouts/<name>/generated/map.pgm + map.yaml          (Nav2 static map)
        ├──► layouts/<name>/generated/keepout/<zone>.pgm + .yaml  (Nav2 keepout masks, v2 closures)
        ├──► layouts/<name>/generated/zones.yaml                  (SwarmFlow sidecar: conflict zones, feasibility)
        └──► layouts/<name>/generated/sim2d.json                  (2D backend layout, v2)
```

**Graph model.**

- **Vertices** = stations (loading, delivery, charger/park) and intersections. Name pattern: `L1..Ln` (loading),
  `D1..Dn` (delivery), `P1..Pn` (park), `X_<row>_<col>` (intersection).
- **Edges (lanes)** = corridors between vertices; each has `clear_width_m`, `length_m`, `bidirectional`.
- **Conflict zones** attach to graph elements: a zone is a set of edges and/or vertices plus a polygon in the map
  frame, an `entry_vertices` list, and a `capacity` (1 for v1/v2 exclusive zones). Example: a one-way storage aisle
  is zone `Z_aisle_3` covering lanes `X_3_0→X_3_1`, `X_3_1→X_3_0`, with entry vertices `X_3_0`, `X_3_1`.
- **Payload feasibility** per edge is derived, not authored: `feasible(edge, payload) = clear_width_m ≥ padded_width(payload) + 0.10`
  (§7.2). The generator writes it into `zones.yaml` and asserts the §7.2 invariants in tests.

`layout.yaml` schema (frozen in M2, `layouts/schema/layout.schema.json`): `name`, `resolution_m`, `bounds`,
`racks[]` (rectangles), `stations[]` (name, type, pose x/y/yaw), `intersections[]`, `lanes[]` (from, to,
clear_width_m, bidirectional), `zones[]` (name, lanes[], vertices[], capacity), `spawn[]` (robot_id, vertex).

## 6.2 Components

| Component | v1 | v2 | Package / path |
|---|---|---|---|
| Fleet layer | SwarmFlow orchestrator, FCFS policy | + predictive policy | `src/swarmflow_core/`, `src/swarmflow_orchestrator/` |
| Robot agent (fleet adapter) — one per robot | SwarmFlow robot agent | same | `src/swarmflow_robot_agent/` |
| Baseline C (RMF core + free_fleet + rmf-web + state bridge) | — | ✔ | `src/swarmflow_rmf/`, `config/rmf/` |
| Navigation | Nav2 per robot | Nav2 + Smac Hybrid + MPPI + keepout | `src/swarmflow_nav/` |
| Robot model | custom diff-drive xacro | same + payload footprints | `src/swarmflow_description/` |
| World & sim bringup | Gazebo Harmonic + ros_gz bridges | + workers, closures | `src/swarmflow_gazebo/` |
| Package / payload node | pose-follower, payload state | + footprint publisher | `src/swarmflow_payload/` |
| Order & scenario generator | order stream (seeded) | + stress events, timelines | `tools/scenarios/`, `src/swarmflow_scenarios/` |
| Monitoring | decision log, Foxglove layout, metrics script | + telemetry API, React panel | `viz/foxglove/`, `tools/metrics/`, `web/` |
| 2D backend | skeleton only | benchmark backend | `sim2d/` |

The orchestrator **never** commands wheel velocities. The monitoring stack is **never** a source of truth for
navigation or safety decisions.

## 6.3 Robot agent per robot (C6)

The robot agent is the **only** bridge from the fleet layer to Nav2, in v1 and v2. (In v2 Baseline C runs,
free_fleet plays this role for RMF.)

Behaviour (`src/swarmflow_robot_agent/`):

1. Receives a task as a **graph route** (ordered vertex names) via the `DispatchTask` action (§6.6).
2. Splits the route into **segments** that end at the next zone-entry vertex (or the goal) and sends each segment
   to Nav2 as one `NavigateThroughPoses` goal (poses = vertex poses, yaw = heading of the next edge).
3. At a **zone-entry vertex** it stops (segment ends there) and requests a reservation (§6.5). It sends the next
   segment only after a grant.
4. Publishes `RobotState` at 5 Hz and reservation heartbeats at 1 Hz; releases leases on zone exit.
5. On Nav2 failure: retries the segment once, then reports `STUCK` with the failing vertex and cancels the task
   (the orchestrator reassigns).

## 6.4 Corridor closures (C7)

A closed corridor must be closed for **both** the fleet layer and the local planner, otherwise Nav2 would happily
replan through it. v1 has no closures. v2: Nav2 **keepout costmap filter** per robot **[V]** [S10]; the closure node republishes the keepout mask
(`OccupancyGrid`) with the closed zone's polygon marked, and the orchestrator removes the zone's lanes from
routing. Both happen from the same `ScenarioEvent` (§12.2). Exact runtime mask-update mechanism (map server
`load_map` vs. a custom mask publisher) **[U — WS-A, v2]**. (Baseline C maps closures to RMF lane closures
**[U — WS-D, v2]**.)

## 6.5 Reservation protocol (v1 with FCFS, fully specified) (C8)

The handoff specified this protocol for v2; after D1 it is the **v1** traffic control, with the FCFS policy. The
full rule set below (leases, heartbeats, orchestrator-down behaviour) is implemented in v1 — it is small, and the
safety rules are not optional.

Actors: **robot agent** (client), **orchestrator** (sole reservation authority). All times are **sim time**.

| Message | Direction | Interface | Key fields |
|---|---|---|---|
| Request | agent → orch | service `/fleet/request_reservation` (`RequestReservation.srv`) | request_id, robot_id, zone_id, entry_vertex, exit_vertex, direction, earliest_entry, expected_exit, priority |
| Grant / Deny | orch → agent | service response | result ∈ {GRANTED, DENIED}, lease_id, lease_expiry, reason, retry_after_s |
| Heartbeat | agent → orch | topic `/fleet/reservation_heartbeat` (`ReservationHeartbeat.msg`) | robot_id, lease_ids[] |
| Release | agent → orch | topic `/fleet/reservation_release` (`ReservationRelease.msg`) | robot_id, lease_id, reason ∈ {EXITED, TASK_CANCELLED, FAULT} |
| State broadcast | orch → all | topic `/fleet/reservations` (`ZoneReservation.msg`, one per change) | lease_id, robot_id, zone_id, state ∈ {GRANTED, RELEASED, EXPIRED, REVOKED, OCCUPIED_UNKNOWN} |

Rules:

1. **Safety invariant:** at most `capacity` (= 1) unexpired leases per zone; never two leases in conflicting
   zones. Property-tested in `src/swarmflow_core/test/test_reservations.py`.
2. **Leases are time-limited:** `lease_ttl = 5 s`, renewed by each heartbeat (1 Hz). A lease also has a hard bound
   `expected_exit + max(10 s, 0.5 × (expected_exit − earliest_entry))`.
3. **Release on zone exit:** the agent releases when its pose leaves the zone polygon *and* it has passed the exit vertex.
4. **Expiry while inside:** if a lease expires and the robot has not reported exit, the zone becomes
   `OCCUPIED_UNKNOWN` — no new grants until that robot reports outside the zone or an operator clears it. (Conservative
   by design: never grant into a zone that may be occupied.)
5. **Retries:** a denied agent waits at the entry vertex and retries after `retry_after_s` (default 1 s, jittered ±20 %).
6. **Orchestrator down** (no service response within 2 s, or no `/fleet/reservations` heartbeat for 3 s):
   - an agent **outside** a zone holds safely at its entry vertex;
   - an agent **inside** a zone continues to the exit vertex (Nav2 local avoidance still active) and then holds;
   - after `orchestrator_timeout = 10 s` the agent sets `mode = FAULT`, reports `fault_reason = ORCHESTRATOR_TIMEOUT`,
     and keeps holding until the orchestrator returns. It never enters a new zone without a grant.
7. Policies: **v1 / Baseline B = FCFS exclusive**; v2 predictive = deadline/congestion-aware with direction
   batching (§9.2). The protocol is identical for both.

In Baseline C runs (v2), RMF's traffic schedule and negotiation replace this protocol entirely.

## 6.6 `swarmflow_interfaces` — frozen in M2, Day 1 (C9)

Package `src/swarmflow_interfaces/` (ament_cmake, rosidl). **Frozen** after the Day-1 contract merge; changes go
through the contract-change process in `AGENTS.md`. Field lists below are the Day-0 draft; the lead finalizes
them on the M2 contract branch (`lead/contract-freeze`).

`msg/Order.msg`
```
string order_id
builtin_interfaces/Time release_time
builtin_interfaces/Time deadline        # zero = no deadline
string pickup_vertex                    # nav-graph vertex name, e.g. "L1"
string dropoff_vertex                   # e.g. "D3"
string payload_type                     # "small" | "medium" | "wide" | "long"
uint8 priority                          # 0 = normal, higher = more urgent
```

`msg/OrderStatus.msg`
```
std_msgs/Header header
string order_id
uint8 STATE_QUEUED=0
uint8 STATE_ASSIGNED=1
uint8 STATE_PICKING_UP=2
uint8 STATE_IN_TRANSIT=3
uint8 STATE_DELIVERED=4
uint8 STATE_FAILED=5
uint8 state
string robot_id
builtin_interfaces/Time eta
string failure_reason                   # e.g. "STUCK_TIMEOUT"
```

`msg/RobotState.msg`
```
std_msgs/Header header                  # frame_id = "map"
string robot_id                         # "robot_1"
float64 x
float64 y
float64 yaw
float64 linear_speed
uint8 MODE_IDLE=0
uint8 MODE_NAVIGATING=1
uint8 MODE_WAITING_RESERVATION=2
uint8 MODE_LOADING=3
uint8 MODE_UNLOADING=4
uint8 MODE_STUCK=5
uint8 MODE_FAULT=6
uint8 mode
string fault_reason
string task_id
string order_id
string last_vertex
string next_vertex
string[] remaining_route
string[] held_lease_ids
builtin_interfaces/Time eta
```

`msg/PayloadState.msg`
```
std_msgs/Header header
string robot_id
bool loaded
string order_id
string payload_type
float64 size_x                          # m, along robot heading
float64 size_y                          # m, lateral
geometry_msgs/Polygon footprint         # effective padded footprint, base_link frame
```

`msg/DecisionEvent.msg`
```
std_msgs/Header header
string event_id
string policy                           # "fcfs" | "predictive" | "rmf" | "independent"
string decision_type                    # "ASSIGN" | "REASSIGN" | "REROUTE" | "PRIORITY" | "RESERVATION_GRANT"
                                        # | "RESERVATION_DENY" | "CONGESTION_RESPONSE" | "DISRUPTION_RESPONSE" | "STUCK_FAIL"
string robot_id
string order_id
string trigger                          # e.g. "replan_tick", "scenario:corridor_closure:Z_aisle_3"
string previous_decision
string new_decision
string[] cost_keys
float64[] cost_values
float64 predicted_improvement_s
string explanation                      # rendered from the fields above, never free-written
```

`msg/ScenarioEvent.msg`
```
std_msgs/Header header
string event_id
string event_type                       # "ORDER_BURST" | "CORRIDOR_CLOSURE" | "CORRIDOR_OPEN" | "WORKER_ACTIVITY"
                                        # | "ROBOT_SLOWDOWN" | "STATION_DELAY" | "POLICY_CHANGE"
string target_id                        # zone, robot, station or policy name
float64 value                           # e.g. speed factor, burst size
float64 duration_s                      # 0 = until reverted
string params_json                      # extra typed params, schema per event_type in docs
```

`msg/ZoneReservation.msg`, `msg/ReservationHeartbeat.msg`, `msg/ReservationRelease.msg`, `srv/RequestReservation.srv`
— fields as in §6.5, with constants for `direction`, `state`, `result` and `reason`.

`action/DispatchTask.action`
```
string task_id
string robot_id
swarmflow_interfaces/Order order
string[] route                          # vertex names, pickup and dropoff included
---
bool success
string failure_reason
builtin_interfaces/Time completion_time
---
uint8 mode                              # RobotState MODE_* constants
string current_vertex
string waiting_for_zone
builtin_interfaces/Time eta
```

**Topic / service / action names** (frozen with the package):

| Name | Type | Producer → consumer |
|---|---|---|
| `/fleet/orders` | `Order` | scenario generator → fleet layer |
| `/fleet/order_status` | `OrderStatus` | fleet layer → monitoring |
| `/fleet/robot_states` | `RobotState` | robot agents (or RMF state bridge) → all |
| `/fleet/decisions` | `DecisionEvent` | orchestrator / bridges → monitoring, `decisions.jsonl` |
| `/fleet/scenario_events` | `ScenarioEvent` | scenario engine → subsystems |
| `/fleet/reservations` | `ZoneReservation` | orchestrator → all |
| `/fleet/request_reservation` | `RequestReservation` (srv) | agent → orchestrator |
| `/fleet/reservation_heartbeat`, `/fleet/reservation_release` | msgs | agent → orchestrator |
| `/robot_N/dispatch_task` | `DispatchTask` (action) | orchestrator → agent N |
| `/robot_N/payload_state` | `PayloadState` | payload node → agent, monitoring |
| `/robot_N/navigate_through_poses` | `nav2_msgs/action/NavigateThroughPoses` | agent N → Nav2 N |

In Baseline C runs (v2), a small **RMF state bridge** (WS-D) publishes `/fleet/robot_states` and `/fleet/order_status`
from RMF's fleet/task state so Foxglove and the metrics script work unchanged **[U — RMF topic and message names on Jazzy, WS-D]**.

## 6.7 Orchestrator = pure-Python library + thin adapters (C10)

`src/swarmflow_core/` is an ament_python package with **zero ROS imports** (enforced by
`test/test_no_ros_imports.py`). It holds graph loading, order models, policies, reservations, metrics and the
decision-event builder. Adapters:

- `src/swarmflow_orchestrator/` — ROS 2 node: subscribes/publishes the §6.6 interfaces, calls the library.
- `swarmflow_core.backends.fake` — in-memory fake backend used by unit tests; grows into the 2D backend (`sim2d/`).
- `sim2d/` — 2D kinematic backend (v2) implementing the same `Backend` protocol.

Frozen API (`src/swarmflow_core/swarmflow_core/api.py`, M2). Time is always passed in (sim seconds), so the
library is deterministic and testable:

```python
from dataclasses import dataclass
from typing import Protocol, Sequence

@dataclass(frozen=True)
class OrderSpec: order_id: str; release_t: float; deadline_t: float | None
    # plus pickup_vertex, dropoff_vertex, payload_type, priority

@dataclass(frozen=True)
class RobotSnapshot: robot_id: str; x: float; y: float; yaw: float; mode: str
    # plus last_vertex, task_id, order_id, payload_type, held_lease_ids

@dataclass(frozen=True)
class FleetSnapshot: t: float; robots: Sequence[RobotSnapshot]; open_orders: Sequence[OrderSpec]

@dataclass(frozen=True)
class Assignment: robot_id: str; order_id: str; task_id: str; route: Sequence[str]

@dataclass(frozen=True)
class PlanResult: assignments: Sequence[Assignment]; decisions: Sequence["Decision"]

class Policy(Protocol):
    name: str
    def plan(self, snapshot: FleetSnapshot, graph: "Graph") -> PlanResult: ...

class ReservationAuthority(Protocol):
    def request(self, req: "ReservationRequest", t: float) -> "ReservationDecision": ...
    def heartbeat(self, robot_id: str, lease_ids: Sequence[str], t: float) -> None: ...
    def release(self, robot_id: str, lease_id: str, t: float) -> None: ...
    def expire(self, t: float) -> Sequence["Lease"]: ...

class Backend(Protocol):          # ROS adapter, fake backend, 2D sim
    def snapshot(self) -> FleetSnapshot: ...
    def dispatch(self, assignment: Assignment) -> None: ...
    def cancel(self, task_id: str) -> None: ...
```

The dataclasses above are abbreviated; the M2 contract branch writes them out in full with type hints and docstrings.

# 7. Simulation, Robot and Nav2

## 7.1 Custom chassis (C11)

A deliberately basic differential-drive robot, `src/swarmflow_description/urdf/swarmflow_bot.urdf.xacro`:

| Property | Value |
|---|---|
| Body (L × W × H) | **0.60 × 0.50 × 0.25 m** box, flat top = cargo platform |
| Drive | 2 drive wheels (Ø 0.15 m, track 0.42 m) + 2 casters; Gazebo `DiffDrive` system |
| Max speed | 0.6 m/s linear, 1.5 rad/s angular (Nav2 limits) |
| Sensor | 2D LiDAR, 360°, 10 Hz, range 0.12–8 m, mounted centre-front, above platform height of small payloads |
| Odometry | Gazebo odometry publisher → `odom` frame |
| ID marker | coloured top plate + robot-number panel (visual only), colour per `robot_id` |
| Frames | `base_footprint` → `base_link` → `lidar_link`, wheel links |
| Parameters (xacro args) | `robot_id`, `color`, `namespace` |

LiDAR mount height and any self-occlusion by payloads: **[U — WS-A verifies in sim]**.

## 7.2 Robot → payload → aisle dimensions (C11)

Footprint rule: effective footprint = axis-wise max of chassis and payload, plus **0.05 m padding per side**.
Feasible lane: `clear_width ≥ padded_width + 0.10 m`. Two-way lane: `clear_width ≥ 2 × padded_width + 0.30 m`.
In-place rotation needs `clear_width ≥ 2 × circumradius(padded footprint)`.

| Load | Payload (L × W, m) | Padded footprint (L × W, m) | Rotation Ø (m) | Version |
|---|---|---|---|---|
| Unloaded | — | 0.70 × 0.60 | 0.92 | v1 |
| **Small** | 0.40 × 0.40 | 0.70 × 0.60 (fits on platform → constant footprint) | 0.92 | **v1** |
| Medium | 0.65 × 0.65 | 0.75 × 0.75 | 1.06 | v2 |
| Wide | 0.65 × 1.00 | 0.75 × 1.10 | 1.33 | v2 |
| Long | 1.20 × 0.60 | 1.30 × 0.70 | 1.48 | v2 |

Aisle clear widths per layout (between rack faces):

| Layout | Main aisles | Storage aisles | Cross aisles | Consequences (by construction) |
|---|---|---|---|---|
| Open (v2) | 2.60 | 2.60 | 2.60 | Everything feasible; two-way for all loads (wide needs 2.50). |
| **Standard (v1)** | 2.20 | **1.30** | 2.20 | Main aisles two-way for unloaded/small/medium. Storage aisles **one-way for all loads** (unloaded needs 1.50) → each storage aisle is an exclusive conflict zone = the v1 narrow-corridor scene. Wide fits storage aisles (needs 1.20). Long cannot rotate in storage aisles. |
| Dense (v2) | 1.60 | **1.00** | 1.60 | **Wide is infeasible in storage aisles** (needs 1.20 > 1.00). Medium/long/small fit one-way. Unloaded/small can rotate in place in storage aisles (0.92 < 1.00); medium, wide and long can rotate only in main/cross aisles (long: 1.48 ≤ 1.60), so loaded robots enter storage aisles already aligned. |

**Nav2 clearance check (analytical, 2026-10-08)** against the Jazzy `nav2_bringup` default params **[V]** [S12]:
costmap resolution 0.05 m, inflation radius 0.70 m, cost scaling factor 3.0, NavFn planner, MPPI controller with
`consider_footprint: false` and collision at cost ≥ 253. With our footprint polygon (0.70 × 0.60 m, + Nav2's
default 0.01 m padding → inscribed radius 0.31 m, circumradius 0.47 m), using Nav2's inflation formula
`cost = 252·exp(−3.0·(d − 0.31))` for a cell at distance *d* from the nearest rack:

| Aisle | Centre-to-rack *d* | Centre-line cost | Free centre band (cost < 253) | Rotate in place (0.47 < *d*) |
|---|---|---|---|---|
| Standard storage 1.30 m | 0.65 m | ≈ 90 | 0.68 m | ✔ (0.18 m spare) |
| Standard main 2.20 m | 1.10 m | 0 (beyond 0.70 m inflation) | 1.58 m | ✔ |
| Dense storage 1.00 m (v2) | 0.50 m | ≈ 142 | 0.38 m | ✔ but only 0.03 m spare — tight at 0.05 m resolution |

Result: **the 1.30 m standard storage aisle is passable with defaults** — the aisle centre sits well below the
collision cost and the whole footprint (circumradius 0.47 m) stays clear of the racks even while rotating. Two
unloaded robots *could* in theory squeeze past each other in 1.30 m (≈ 0.03 m margins), but only through cells at
cost ≈ 245, which MPPI/NavFn will not plan through in practice — they block each other, which is the behaviour the
Baseline A scene needs. Required settings: replace `robot_radius: 0.22` with our `footprint` polygon in both costmaps,
and model racks as **solid collision boxes down to the floor** so the LiDAR cannot see through shelf legs.
Still to confirm in simulation (WS-A Day 2 acceptance): real LiDAR noise and MPPI behaviour at the 1.30 m aisle.
For the dense layout (v2), consider 1.05 m storage aisles if in-place rotation proves flaky.

All values remain provisional until the WS-F generator tests assert every row of the tables above. If Nav2 needs more
room, widen all aisles by the same delta and update the tables — keep the inequalities, not the numbers.

## 7.3 Nav2 configuration (C12, C15)

- **v1:** Nav2 defaults from the multi-robot example (`nav2_bringup`) with our footprint polygon (0.70 × 0.60) and
  tuned inflation; global planner and controller as in the example **[U — exact defaults on Jazzy, WS-A]**.
- **v2 footprint-aware planning:** `nav2_smac_planner` **SmacPlannerHybrid** + **MPPI** controller with full
  footprint collision checking **[U — confirm param names (e.g. MPPI cost critic `consider_footprint`) against Jazzy docs, WS-A]**.
- **Runtime footprint updates (v2):** the payload node publishes the padded footprint polygon to each costmap's
  `footprint` topic (`/robot_N/local_costmap/footprint`, `/robot_N/global_costmap/footprint`) on load/unload
  **[U — confirm topic name and message type (`geometry_msgs/Polygon` vs `PolygonStamped`) on Jazzy, WS-A]**.
- Keepout filter in global and local costmaps (v2) **[V]** [S10].

## 7.4 Packages via pose-follower (C13)

No runtime-spawned joints. Node `src/swarmflow_payload/swarmflow_payload/pose_follower.py`:

1. All package models for the run are spawned at startup, parked at their loading station shelves (or hidden below floor).
2. On load (robot within 0.10 m / 5° of the loading pose and in `MODE_LOADING`), the node sets the package model's
   pose to the robot's `base_link` pose + platform offset each tick (target 20 Hz), until unload.
3. On unload, the package is placed at the delivery station drop pose and stops following.
4. Publishes `/robot_N/payload_state`.

Mechanism to set a model pose in Gazebo Harmonic from ROS (gz `set_pose` world service via gz-transport or a
`ros_gz` bridge) **[U — lead verifies in M4/M6]**. Packages must not collide with the robot (disable collisions on
package models in v1). If service-call pose updates visibly jitter or lag (checked on Day 3), fall back to Gazebo's
`DetachableJoint` system: a joint to the package defined in advance, attached/detached by a topic — not a
runtime-spawned joint **[U — Harmonic plugin name and topic API to confirm before use]**.

## 7.5 Workers (C14, v2)

Workers start as **moving primitive shapes** (e.g. 0.4 m × 1.7 m cylinders) on scripted, seeded trajectories —
guaranteed visible to a 2D LiDAR. Animated actors are used only if WS-A verifies they appear in the LiDAR scan
**[U]**. Workers are not coordinated with reservations; local avoidance handles them and the orchestrator handles
sustained blockage.

## 7.6 Multi-robot bringup and TF namespacing (C15)

Start from Nav2's multi-robot example (`unique_multi_tb3_simulation_launch.py` **[V]** [S9]) with
**composition enabled** (one Nav2 container process per robot — less CPU).

**Default — Option N (namespaced, single ROS domain):**
- Each robot in namespace `/robot_N` (`robot_1..robot_3`). Nav2 nodes, costmaps and actions live under it.
- **Separate TF tree per robot:** each robot publishes on `/robot_N/tf` and `/robot_N/tf_static` (remapped from
  `/tf`), with **unprefixed frame names** (`map`, `odom`, `base_link`) — this is how the Nav2 multi-robot example works.
- All robots localize in the same `map` (same static map, AMCL with launch-supplied initial pose).
  Fallback if AMCL costs time: ground-truth localization (static `map→odom` identity + Gazebo odometry).
- Global views (Foxglove, orchestrator) use `/fleet/robot_states` poses (map frame), not a merged TF tree.
- v1 uses **Option N**; the SwarmFlow robot agent works with it directly. For Baseline C (v2), free_fleet with
  Option N means its "pre-namespaced robots, one non-namespaced bridge" example, which upstream marks testing-only
  **[V]** [S4].

**Alternative — Option D (domain-isolated, non-namespaced), v2 Baseline C only:** each robot container uses its own
`ROS_DOMAIN_ID` with non-namespaced Nav2 and its own `ros_gz_bridge` (gz-transport topics `robot_N/...` mapped to plain
ROS names, `/clock` bridged per domain); one namespaced zenoh bridge per robot. This matches free_fleet's intended
design. WS-D tries Option N with free_fleet first and switches to Option D only if it fails.

# 8. Docker and Environment (C16, C17)

## 8.0 Host prerequisites (R1)

Host: Windows 11, Intel Core Ultra 9 285H (16 threads, integrated Arc GPU), 63 GB RAM, WSL 2.5.10 with WSLg
(checked 2026-10-09). Already present: WSL2 + an `Ubuntu` distro. **Missing: a Docker engine** — nothing in this
document runs until one exists. Two ways to provide it (decision **D3**, see `docs/milestones.md` G1):

| Option | What it is | Containment |
|---|---|---|
| **A. Docker Desktop** (WSL2 backend) | Docker's official Windows app; `docker` works from PowerShell. | System-wide app, but SwarmFlow data stays in Docker's own disk image (location configurable), CPU/RAM capped in settings, and `docker compose down -v --rmi local` removes only this project's containers, volumes and images. |
| **B. Dedicated `swarmflow` WSL distro** with Docker Engine inside | A separate Ubuntu 24.04 distro used only for this project; `docker` runs inside it (`wsl -d swarmflow -- docker compose up` from PowerShell). | Fully contained: one VHDX file; `wsl --unregister swarmflow` deletes everything. No Docker Desktop. WSLg works natively inside a distro (the [V] route in §8.4). |

Until D3 is decided, commands in this document are written for `docker compose` in the repo root; under option B
they run inside the `swarmflow` distro (a `swarmflow.ps1` wrapper can hide this).

## 8.1 Day-1 environment spike (C16)

The spike must pass before anything else depends on Docker networking. Owner: lead + WS-A.

1. **Gazebo server headless** in the `gazebo` container (`gz sim -s -r <world>`); **GUI in its own container**
   (`gz sim -g`), displayed on Windows (§8.4). The spike proves which display route works on the user's machine.
2. **`/clock`** bridged from Gazebo once; **`use_sim_time: true`** on every node (launch files pass it; a test
   greps launch files for it).
3. **RMW (ROS middleware): Cyclone DDS** — `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` in every container, all ROS
   containers on one user-defined Docker network. Why: Nav2 is widely run on it, it needs no extra router process,
   and it avoids Fast DDS's shared-memory transport, which breaks between containers that don't share `/dev/shm`
   (if Fast DDS is ever used, disable shared memory). It is also what free_fleet expects, so Baseline C (v2) needs no
   change. `rmw_zenoh` (released for Jazzy **[V]** [S6]) is the alternative if DDS discovery between containers
   fails on Docker Desktop: it needs one `rmw_zenohd` router container and routes all traffic through it.
4. **"Hello multi-container" test** (`tests/integration/test_multi_container.sh`): container A publishes a counter at
   10 Hz; container B must **receive ≥ 50 messages in 10 s**. Fails if the topic is merely listed (`ros2 topic list`)
   but no data flows — the classic cross-container DDS failure.
5. Measure: image build times, idle CPU of Gazebo with the standard world, CPU with 1 and 3 robots + Nav2, and
   `colcon build` time with bind-mounted vs volume-backed `build/` (§8.3). Record in an agent-log entry.

## 8.2 Images (C17)

| Image | Dockerfile | Contents | Used by |
|---|---|---|---|
| `swarmflow/dev` | `docker/dev.Dockerfile` | ROS 2 Jazzy base, build tools, `colcon`, `pytest`, our source deps. **No Gazebo GUI, no GPU.** | agents, CI, unit tests, 2D sim |
| `swarmflow/sim` | `docker/sim.Dockerfile` | `dev` + Gazebo Harmonic + `ros_gz` (+ noVNC stack for the browser GUI fallback, §8.4) | `gazebo`, `gazebo_gui` services |
| `swarmflow/robot` | `docker/robot.Dockerfile` | `dev` + Nav2 + our robot packages + robot agent | `robot_1..robot_3` — **one image, configured by env vars** (`ROBOT_ID`, `ROBOT_NAMESPACE`, `SPAWN_VERTEX`, `ROS_DOMAIN_ID`) |
| `swarmflow/fleet` (v2) | `docker/fleet.Dockerfile` | `dev` + Open-RMF (`ros-jazzy-rmf-dev`) + free_fleet (source) + zenoh bridge + `zenohd` | Baseline C: `rmf`, `fleet_adapter` |
| upstream (v2) | — | `ghcr.io/open-rmf/rmf-web/api-server:jazzy-nightly`, `…/demo-dashboard:jazzy-nightly` (pinned by digest) **[V]** [S8] | Baseline C: `rmf_api`, `rmf_dashboard` |

Software installs happen **only** in these Dockerfiles. Versions (apt package versions where practical, image
digests, pip pins) are recorded in `docker/versions.lock.md`.

## 8.3 Compose services

**Files (R4):** `compose.yaml` in the repo root is what `docker compose up` finds; it only `include`s
`docker/compose.yaml`, where the services live. Both are owned by WS-A.

**Volumes (R9):** the repo's `src/` (and `layouts/`, `scenarios/`, `tests/`) are bind-mounted into containers;
`build/`, `install/` and `log/` are **named volumes** (`swarmflow_build`, `swarmflow_install`, `swarmflow_log`), because
building on a bind-mounted Windows folder is slow and burns CPU. Results go to a bind-mounted `runs/`.

| Service | Image | v1 | Notes |
|---|---|---|---|
| `dev` | dev | tools profile | build/test shell for agents and `scripts/ci.sh`: `docker compose run --rm dev <cmd>`. In profile `tools`, so plain `up` never starts it. |
| `gazebo` | sim | ✔ | headless server, `/clock` bridge, spawns robots and packages; label `swarmflow.sim=1` |
| `gazebo_gui` | sim | ✔ | `gz sim -g` client, shown on Windows (§8.4); can be stopped alone to save CPU; label `swarmflow.sim=1` |
| `robot_1..robot_3` | robot | ✔ | Nav2 (composed) + bridges + SwarmFlow robot agent |
| `orchestrator` | dev | ✔ | `swarmflow_orchestrator` (FCFS in v1) |
| `scenario_engine` | dev | ✔ | order generator; stress events in v2 |
| `payload` | dev | ✔ | pose-follower + payload state |
| `foxglove_bridge` | robot | ✔ | `foxglove_bridge`, port 8765 |
| `rmf`, `fleet_adapter`, `rmf_api`, `rmf_dashboard` | fleet / upstream | v2 (profile `baseline-c`) | RMF core, free_fleet + `zenohd`, dashboard ports 8000 / 3000 |
| `telemetry_api`, `web` | dev / node | v2 | FastAPI + React panel |

`docker compose up` with no other arguments starts the full v1 stack including the GUI. Host networking on Docker
Desktop for Windows is **not assumed**; if the RMF dashboard needs it in v2 (as upstream suggests [S8]), WS-D documents
the workaround **[U]**.

## 8.4 Gazebo GUI in a container, on Windows (decision D2)

Requirement: the Gazebo GUI runs **in a container** and the user can start everything **from Windows** with
`docker compose up`. The GUI is a separate `gazebo_gui` container (`gz sim -g`) that connects to the headless server
over gz-transport on the shared Docker network (same `GZ_PARTITION` in both containers; discovery across containers
**[U — spike step 1]**). Two display routes, chosen by `SWARMFLOW_GUI=wslg|vnc` in `.env`:

| Route | How it works | Status | Trade-off |
|---|---|---|---|
| **WSLg** (preferred) | Windows 11's built-in Linux GUI support. The container mounts the WSLg X11 socket and gets `DISPLAY=:0`, `QT_QPA_PLATFORM=xcb`; the Gazebo window appears as a normal Windows window. Optional GPU: `/dev/dxg` + `/usr/lib/wsl` mounts. | Documented to work for Gazebo containers when `docker` is run **from a WSL Ubuntu terminal** (mount `/mnt/wslg/.X11-unix` → `/tmp/.X11-unix`) **[V]** [S13], [S14]. Running the same compose file **from PowerShell** needs Docker Desktop's path to WSLg (reported as `/run/desktop/mnt/host/wslg/...`) **[U — spike must test]**. | Native window, can use the GPU, low CPU. |
| **noVNC** (guaranteed fallback) | The GUI renders into a virtual display (Xvfb) inside the container; a VNC + noVNC server shows it in the browser at `http://localhost:6080`. | Works from any Windows terminal with only Docker Desktop **[U — standard technique, not yet tested here]**. | Software rendering: more CPU, so watch the thermal limit; browser tab instead of a window. |

Day-1 spike outcome required: from **PowerShell**, `docker compose up` shows the Gazebo GUI via WSLg; if not, `vnc`
becomes the default and the README says so. Foxglove (browser, `ws://localhost:8765`) works either way.
Under host option B (§8.0) compose runs inside a WSL distro, which is exactly the verified WSLg route.

## 8.5 Thermal budget (R10)

The laptop CPU throttles under sustained load. Controls, in order of use:

1. One Gazebo at a time, lead-only (AGENTS.md §5); builds throttled to 2 workers.
2. Stop `gazebo_gui` when nobody is watching; on the WSLg route use the GPU (`/dev/dxg`) so the GUI doesn't software-render.
3. **Cap Gazebo's real-time factor** (e.g. 0.5 via the world's physics `real_time_factor`) when the CPU is hot. All
   nodes run on sim time and all metrics are in sim time, so results stay valid; runs just take longer in wall time.
4. Nav2 composition (one process per robot); ground-truth localization (§15.3 cut 1) removes 3 AMCL instances.
5. Docker CPU limit (Desktop settings or `.wslconfig` `processors=`) as the last resort.

M1 measures CPU at 1 and 3 robots so this budget is based on numbers, not guesses.

# 9. Coordination Algorithms (v2)

v1 uses FCFS: nearest-idle-robot assignment (`swarmflow_core.policies.fcfs`) plus FCFS exclusive reservations
(= Baseline B). Everything below is the v2 **predictive policy**, implemented in `swarmflow_core.policies.predictive`.

## 9.1 Rolling-horizon replanning

Every `replan_period = 5 s` (sim) and immediately on significant events (new urgent order, corridor closure,
robot `STUCK`/`FAULT`, slowdown):

1. Collect robot and order states (`FleetSnapshot`).
2. Update predicted edge travel times (free-flow time × congestion factor).
3. Identify congested zones (forecast utilization > threshold, §9.3).
4. Evaluate current assignments and alternative routes (k-shortest feasible paths, k = 3, payload-feasible edges only).
5. Update reservations / priorities where beneficial.
6. Dispatch revised routes.

**Hysteresis:** replace a robot's route or assignment only if predicted improvement ≥ `max(3 s, 10 %)` of its
remaining ETA; never revise a robot currently inside a zone.

## 9.2 Dynamic zone reservations

Conflict zones as in §6.1, reservation fields as in §6.5 (robot, zone, entry/exit time, direction, priority, expiry).
Guarantees: conflicting reservations are never granted simultaneously (§6.5 rule 1); a robot enters a zone only with a
valid lease **and** Nav2 still performing local avoidance.

- **Baseline B:** first-come-first-served exclusive.
- **Predictive:** grants ordered by a score combining deadline slack, priority and waiting time; **direction batching**
  (consecutive grants in the same direction through a one-way aisle when that reduces total wait); starvation guard:
  any request waiting > 60 s gets top priority.

## 9.3 Congestion forecasting

Analytical, no ML. For each zone, predicted occupancy over the next horizon (`60 s`, 1 s bins) from: current routes
and ETAs of each robot, active leases, expected passage duration (zone length / speed for the robot's payload),
current closures and blockages. Utilization threshold default 0.7. Prediction accuracy is logged (predicted vs actual
zone entry times) as a secondary metric.

## 9.4 Deadline-aware task assignment

For each open order and each available robot: estimated completion time = time to pickup + load time + pickup→dropoff
time, each including predicted zone waits; feasibility (payload-feasible route exists) is a hard constraint. Solve the
assignment by greedy insertion ordered by deadline slack, then local improvement; OR-Tools is optional and must stay
behind the same `Policy` protocol.

## 9.5 Optimization modes = weight presets (C2)

Cost of a plan: `J = w_tp·(−completed) + w_late·Σ tardiness + w_wait·Σ waiting + w_dist·Σ distance`. Modes are presets:

| Mode | w_tp | w_late | w_wait | w_dist |
|---|---|---|---|---|
| Throughput | 1.0 | 0.2 | 0.3 | 0.1 |
| Deadline | 0.3 | 1.0 | 0.2 | 0.1 |
| Balanced | 0.6 | 0.6 | 0.4 | 0.1 |

Values are starting points to tune in the 2D backend. Changing mode changes weights only — never safety constraints,
feasibility or reservation invariants.

# 10. Explainable Decisions (C22 — v1)

Every significant fleet decision emits a `DecisionEvent` (§6.6) on `/fleet/decisions`, also appended to
`runs/<run_id>/decisions.jsonl`. This is a **v1 feature** because it is the primary debugging tool.

- v1 (FCFS): `ASSIGN`, `RESERVATION_GRANT`, `RESERVATION_DENY`, `STUCK_FAIL`.
- v2: all decision types. Baseline C: the RMF state bridge emits `ASSIGN` and `STUCK_FAIL` from observed state; RMF's
  internal negotiation is not explained (documented limitation).

Event data: timestamp, robot/order id, decision type, previous decision, new decision, cost estimates
(`cost_keys`/`cost_values`), trigger, predicted improvement. The `explanation` string is **rendered by a template from
these fields** — never written after the fact — so explanations are exactly as accurate as the scheduler's inputs and
predictions. Example renderings (v2):

- **Route change:** "Robot 2 rerouted because zone Z_aisle_B has an estimated 12 s delay. Alternative route via X_4_1 is predicted to save 7 s."
- **Priority update:** "Robot 3 granted priority because Order 104 has a predicted deadline risk (slack −4 s)."
- **Congestion response:** "Zone Z_aisle_A forecast utilization 0.82 exceeded threshold 0.70. New routes avoid it when feasible."
- **Disruption:** "Corridor closure in Z_aisle_D (scenario event e17). 2 active reservations are being re-evaluated."

Uses: debugging, algorithm evaluation, reproducibility, dashboard explanations, presentations, finding poor decisions.

# 11. Visualization and Dashboard (C21)

## 11.1 v1: Gazebo GUI + Foxglove

`foxglove_bridge` (WebSocket, port 8765) exposes ROS topics to Foxglove. Layout file `viz/foxglove/swarmflow_v1.json`
(WS-E) contains:

- **3D/Map panel:** `/map`, robot poses (from `/fleet/robot_markers`, a `visualization_msgs/MarkerArray` published by a
  small `swarmflow_viz` node from `/fleet/robot_states`), per-robot Nav2 plans (`/robot_N/plan`), zones (polygons,
  colored by reservation state), stations.
- **Decision log panel** on `/fleet/decisions`.
- **Plots:** deliveries completed, robot speeds, waiting time.
- **Order table** from `/fleet/order_status`.

The Gazebo GUI (containerized, §8.4) is the 3D showcase; Foxglove is the operational view. The RMF web dashboard
is used only for Baseline C runs in v2.

## 11.2 v2: custom React panel

React + TypeScript panel (`web/`) on a FastAPI + WebSocket telemetry API (`src/swarmflow_telemetry/`), developed
against recorded mock data (`tests/fixtures/`) before connecting to the live stack. Views (from v1.1 §7):

- **Fleet overview:** robots, state, current package, destination, ETA, order queue, active/completed deliveries, policy/mode.
- **Live map:** positions, intended paths, predicted trajectories, reserved zones, congested corridors, worker positions, closures, stations.
- **Metrics:** deliveries/min, on-time %, mean wait, mean latency, corridor utilization, congestion events, clearance violations, replans.
- **Robot inspection:** assignment, remaining route, leases, payload geometry, ETA, nav status, recent decisions.
- **Controls:** optimization mode preset (§9.5) and stress-test buttons (§12.1).

# 12. Stress Tests and Reproducible Scenarios (v2)

## 12.1 Supported events

| Event | Effect | `ScenarioEvent.event_type` |
|---|---|---|
| Urgent order burst | N orders with tight deadlines | `ORDER_BURST` |
| Corridor closure / reopen | keepout mask + zone removed from routing (§6.4) | `CORRIDOR_CLOSURE` / `CORRIDOR_OPEN` |
| Human traffic increase | more/faster worker trajectories around a zone | `WORKER_ACTIVITY` |
| Robot slowdown | lower max speed for one robot (Nav2 speed limit) | `ROBOT_SLOWDOWN` |
| Loading station delay | longer load time → pickup queue | `STATION_DELAY` |
| Traffic policy change | switch FCFS ↔ predictive at runtime | `POLICY_CHANGE` |

Fleet expansion is **cut** (C2).

## 12.2 Event handling

The scenario engine publishes a `ScenarioEvent`; the owning subsystem applies it (closure → keepout + orchestrator,
order burst → order queue, worker activity → worker driver, slowdown → robot Nav2 speed limit + ETA model). The
orchestrator reacts only through its normal planning loop (§9.1).

## 12.3 Reproducibility and replay

A scenario file `scenarios/<name>.yaml` fixes: random seed, duration, fleet size, layout, order arrival rate, payload
distribution, worker activity, deadline distribution, policy, mode, and a timeline of scheduled `ScenarioEvent`s.
Every run writes `runs/<run_id>/` with the resolved config, git SHA, image digests, `decisions.jsonl`,
`events.jsonl`, `metrics.csv`, and a rosbag2 (Gazebo runs).

- **2D backend:** deterministic replay — same seed + scenario + code = identical results (tested).
- **Gazebo:** the same timeline is re-played, results vary run to run (physics, timing). Hence n runs + CIs (§13.2).

# 13. Evaluation and Benchmarking

## 13.1 Metrics

**Core:** throughput (deliveries / min), deadline success rate, mean delivery latency (release → delivered), mean
waiting time (time in `WAITING_RESERVATION` or speed < 0.05 m/s while navigating), **clearance violations** (C20).

**Secondary:** distance per order, idle time, zone utilization, replans, orchestrator compute time per tick, prediction
accuracy, fairness (max/mean wait ratio), unfinished orders at end.

**Collision / clearance metric (C20):** computed from poses, **not physics contacts** — every 0.1 s, for every robot
pair, the distance between their effective padded footprint polygons (incl. payload); a violation is distance < 0
(overlap of padded footprints), counted once per contiguous episode. Also robot–worker. Implemented in
`swarmflow_core.metrics.clearance` and shared by both backends.

## 13.2 Statistical reporting (C19)

- Gazebo results: **n ≥ 5 runs** per configuration in v2 (v1 small-*n*: n = 3), reported as mean ± 95 % CI
  (t-distribution).
- The **congestion tipping-point** claim (fleet size at which throughput stops improving) comes **only from the 2D
  backend** (fleet sizes 2–30, n ≥ 30 seeds each), and the 2D backend must **agree with Gazebo at 3 robots**:
  2D mean throughput lies inside the Gazebo 95 % CI for the same scenario. If not, the claim is not published until
  the 2D model is recalibrated.

## 13.3 Policies

| Policy | Description | Backends |
|---|---|---|
| **Baseline A — independent Nav2** | Robots get routes/goals with no traffic coordination; local avoidance only. **Stuck-timeout rule (C18):** if a robot makes < 0.2 m progress toward its goal in 60 s (sim), the order is scored **failed** (`STUCK_TIMEOUT`) and the robot is sent to park. | Gazebo, 2D |
| **Baseline B — reactive FCFS reservations** | §6.5 protocol, FCFS exclusive policy, nearest-idle assignment. (= the v1 fleet layer.) | Gazebo, 2D |
| **Baseline C — Open-RMF** | RMF dispatch + traffic schedule via free_fleet (v2 integration, §5.3). | Gazebo only |
| **Proposed — predictive orchestration** | §9 forecasting, adaptive reservations, rolling horizon, deadline-aware priorities. | Gazebo, 2D |

## 13.4 Environments

Open / standard / dense layouts; low / medium / high order rates; payload distributions (all-small, mixed, wide-heavy);
fleet sizes 3 (Gazebo) and 2–30 (2D); worker activity off / low / high. Every policy runs on matched scenarios and seeds.

# 14. Multi-Agent Development (C23–C28)

The project is built by Claude and ChatGPT/Codex agents running 24/7 in parallel; the user reviews and merges.
Operational rules are in [`AGENTS.md`](../AGENTS.md); this section is the design rationale and ownership map.

## 14.1 Contract-first (C23)

Four contracts are drafted on Day 0 and frozen by midday on Day 1 (milestone M2, gate G2). Everything else is built
in parallel against them.

| Contract | Path | Frozen by |
|---|---|---|
| ROS interfaces | `src/swarmflow_interfaces/**` (§6.6) | lead |
| Layout / graph schema (RMF nav-graph format + SwarmFlow sidecar) | `layouts/schema/**` (§6.1) | lead |
| Orchestrator library API (Python protocols) | `src/swarmflow_core/swarmflow_core/api.py` (§6.7) | lead |
| Mock fixtures (synthetic robot-state streams, sample orders, sample nav graph) | `tests/fixtures/**` | lead (re-recorded from Gazebo on Day 3) |

## 14.2 Workstreams (C24)

| WS | Scope | Owned paths | Needs sim? | Acceptance (v1) |
|---|---|---|---|---|
| **A** | Docker, Gazebo world bringup, custom chassis, Nav2 bringup | `docker/`, `compose.yaml`, `src/swarmflow_description/`, `src/swarmflow_gazebo/`, `src/swarmflow_nav/`, `scripts/` (except `ci.sh`) | **Yes — lead runs it, human watching** | 3 namespaced robots each reach 5 random goals in the standard world; hello-multi-container test passes |
| **B** | Orchestrator library (FCFS v1, predictive v2) + ROS adapter | `src/swarmflow_core/`, `src/swarmflow_orchestrator/` | No (unit tests, fake backend) | FCFS + reservation property tests pass; fake-backend end-to-end of 10 orders × 3 robots |
| **C** | 2D kinematic sim + benchmark harness | `sim2d/`, `tools/bench/` | No | v1: skeleton runs the FCFS policy on the standard layout from `sim2d.json` deterministically |
| **D** | Robot agent (v1, integration with A); RMF + free_fleet Baseline C and RMF state bridge (v2) | `src/swarmflow_robot_agent/`; `src/swarmflow_rmf/`, `config/rmf/` (v2) | Agent: unit tests against a fake Nav2 action server first, then sim | Agent drives a fake `NavigateThroughPoses` server through a route with a zone hold (unit test); Day-2 gate (§15.2) in sim |
| **E** | Foxglove layouts, viz node; v2 web panel + telemetry API on mock data | `viz/`, `src/swarmflow_viz/`, `web/`, `src/swarmflow_telemetry/` | No (mock data) | Foxglove layout shows robots, plans, zones, decisions from fixtures |
| **F** | Layout generator, scenario/order generator, package pose-follower | `layouts/`, `tools/layoutgen/`, `tools/scenarios/`, `scenarios/`, `src/swarmflow_scenarios/`, `src/swarmflow_payload/` | Layoutgen: no; pose-follower: yes | Generator emits all §6.1 artefacts for `standard`; §7.2 invariants asserted in tests |
| **Lead** | Contracts, integration, CI, docs, task cards, `main` | contract paths, `tests/`, `.github/`, `scripts/ci.sh`, `docs/`, `README.md`, `AGENTS.md`, `CLAUDE.md`, `.gitattributes`, `.gitignore`, `tools/metrics/` | Yes (only the lead runs Gazebo) | — |

After D1 the robot agent belongs to WS-D from Day 1 (it is the robot-side half of the integration with WS-A), so no
ownership hand-over is needed in v2.

## 14.3 CI is the referee (C26)

One script, `scripts/ci.sh` (Lead-owned), run in the `dev` image, is the referee (R2). It runs locally before every
integration and in GitHub Actions (`.github/workflows/ci.yml`, remote `origin` = `github.com/ef-73/SwarmFlow`) on
every push and PR:

1. `colcon build --symlink-install` (all packages).
2. `colcon test` + `pytest` (`src/swarmflow_core` and other pure-Python tests, no simulator).
3. **Contract-diff check:** fails if a branch other than `lead/contract-*` touches a frozen contract path (§14.1);
   on GitHub a PR touching contracts additionally needs the user's `contract-change` label.
4. `test_no_ros_imports.py` for `swarmflow_core`.
5. **Owned-files check:** files changed on a task branch ⊆ the owned files listed in its task card (§14.6).

No Gazebo in CI. The user has authorized the lead to fast-forward `main` after `scripts/ci.sh` passes on the branch
rebased onto `main` (R14); `main` must always be green (AGENTS.md §6).

## 14.4 Integration is the bottleneck (C27)

The user plus the lead own integration and all Gazebo/Nav2 debugging. Agents take everything that runs without the
simulator (WS-B, C, E, F-layoutgen, D's unit-tested agent logic). **Only the lead runs Gazebo** (R8): sim-dependent
work in WS-A, D and F-pose-follower is integrated and tested by the lead.

## 14.5 24/7 guardrails (C28)

- **One Gazebo simulation at a time** — the user's CPU has a thermal limit. Only the lead runs Gazebo, and still takes
  `scripts/sim_lock.sh` (WS-A, M1): an atomic `mkdir` lock in `$SWARMFLOW_LOCK_DIR` (one Windows path that every shell,
  Windows or WSL, resolves to the same folder) **and** a check that no running container carries label
  `swarmflow.sim=1` (the Docker engine is shared by all worktrees).
- Agents default to unit tests and the 2D sim. Sim runs are time-boxed (≤ 20 min) and always torn down. Thermal
  controls in §8.5.
- Builds are throttled (`colcon build --parallel-workers 2`, `MAKEFLAGS=-j2`).
- **Agent log:** one file per entry in `docs/agent_log/` so parallel branches never conflict (R6), reviewed by the
  user in batches.
- **WIP limit:** ≤ 2 unmerged branches per workstream (R15).
- **Codex / cloud agents** can't reach the local Docker engine: they get only sim-free cards whose tests run with plain
  `pytest` in their own sandbox (R12).

## 14.6 Task cards (R7)

Work is handed out as task cards in `docs/tasks/T<NNN>-<slug>.md` (template `docs/tasks/TEMPLATE.md`): owned files,
lead tests, forbidden actions, commands, definition of done, model routing and risk level. Only the lead writes cards;
an agent claims one by pushing the card's branch with `status: claimed`. Subagent model routing and the verification
applied to each result are in `docs/milestones.md` §3.

# 15. v1 Plan — 5 Days

## 15.1 Day-by-day

Revised after D1 (no RMF in v1). The structure of the handoff plan is kept; the RMF lane is replaced by the robot agent.
**Day 0** (before the clock starts, R11): the four contracts are drafted on paper, so Day 1 only has to build-check and
freeze them. The clock starts when a Docker engine exists (§8.0). Milestone mapping: `docs/milestones.md` §4.

| Day | Critical path (user + lead agent) | Parallel agent lanes |
|---|---|---|
| **1** | Docker/RMW + GUI spike (§8.1, §8.4); build-check and **freeze contracts** by midday (interfaces, graph schema, orchestrator API, fixtures) | **A:** chassis URDF/xacro + single robot Nav2 in world · **B:** FCFS orchestrator lib + reservation property tests · **D:** robot agent against a fake `NavigateThroughPoses` server (unit tests) · **F:** layout → world + nav graph generator, order generator |
| **2** | 3 namespaced Nav2 robots navigating in Gazebo. **Go/no-go at end of day** (§15.2) | **B:** orchestrator ROS adapter · **D:** robot agent on robot_1 in sim · **E:** Foxglove layout · **F:** package pose-follower node |
| **3** | End-to-end deliveries with 3 robots, packages visible | **C:** 2D sim skeleton on orchestrator lib (v2 seed) · **E:** decision-event log → Foxglove |
| **4** | Narrow-corridor scene: Baseline A (independent Nav2 + stuck timeout) vs Baseline B (FCFS reservations); one-command compose | CI pipeline · small-*n* metrics script (deliveries, wait time, stuck events) · README draft |
| **5** | Buffer + hardening; record demo video; freeze `v1.0.0` tag | docs, architecture diagram, cleanup |

**Realism notes.** Day 2 (multi-robot Nav2 namespacing) is the highest schedule risk. **Day 5 is buffer, not feature
time.** Removing RMF from v1 removes the biggest integration unknown, but the robot agent is now custom code on the
critical path — which is why it starts on Day 1 against a fake Nav2 server instead of waiting for the sim.

## 15.2 Go/no-go gate (end of Day 2)

**GO** if all hold, demonstrated live with the user watching:

1. 3 namespaced Nav2 robots each reach goals in the standard world (WS-A acceptance, §14.2).
2. The orchestrator dispatches **one order to one robot**; the robot agent drives it through Nav2 from pickup to
   dropoff, **holding at one zone-entry vertex until granted**, and the order reaches `DELIVERED`.

**NO-GO → fallback** if either fails by end of Day 2. No extensions: the decision is made at the gate.

## 15.3 Fallback (scope cuts, in this order)

Cut until the gate's chain works, then continue the plan:

1. **Ground-truth localization** instead of AMCL (static `map→odom` + Gazebo odometry, §7.6).
2. **2 robots** instead of 3 (the corridor scene still works with 2).
3. **Single-goal dispatch:** agent sends `NavigateToPose` per vertex instead of `NavigateThroughPoses` segments.
4. **GUI via noVNC** if WSLg is still not working (§8.4); Foxglove-only video as the last resort.

Each cut is logged in `docs/decisions/` and listed in the README's "known limitations".

## 15.4 v1 definition of done

- `git clone` + `docker compose up` (from a Windows terminal) → Gazebo server + GUI, 3 robots, orchestrator,
  Foxglove bridge running; README states exact steps.
- ≥ 10 deliveries completed by 3 robots in a 10-minute run without manual intervention, packages visible.
- Corridor scene metrics (n = 3) for Baseline A vs coordinated policy in `runs/` and summarized in README.
- CI green on `main`; `v1.0.0` tag; demo video linked from README.

# 16. v2 Plan (after v1, ~2–3 weeks)

Order of work (each a milestone with its own acceptance test):

1. **Baseline B reproducible** in Gazebo (n ≥ 5) — the v1 fleet layer, unchanged.
2. **2D backend** calibrated against Gazebo at 3 robots (§13.2).
3. **Baseline C:** RMF + free_fleet in the `fleet` image, our nav graph loaded, RMF go-to-place tasks driving our
   robots (§5.3, §5.5 consequences, §7.6 Option N/D). Can run in parallel with items 4–6.
4. **Predictive policy**: forecasting, rolling horizon, deadlines, weight presets (§9).
5. **Payload footprints**: 4 payload types, runtime footprint updates, Smac Hybrid + MPPI, feasibility routing (§7.2–7.3).
6. **Workers + closures** via keepout filters (§6.4, §7.5).
7. **Stress-test injection** and scenario timelines (§12).
8. **Benchmarks** across layouts/fleet sizes with Baselines A, B, C (§13).
9. **React panel** + telemetry API (§11.2).
10. Final presentation (§19).

v2 duration is bounded by user review and Gazebo integration time, not by agent throughput.

# 17. Risk Register

| # | Risk | Likelihood | Impact | Mitigation |
|---|---|---|---|---|
| R1 | Multi-robot Nav2 namespacing/TF in Gazebo Harmonic eats Day 2 | High | High | Start from the Nav2 multi-robot example; scope cuts §15.3; Day 5 buffer |
| R2 | Custom robot agent is now on the v1 critical path | Medium | High | Starts Day 1 against a fake Nav2 action server; small scope (§6.3); fallback cut 3 (§15.3) |
| R3 | Cross-container DDS on Docker Desktop/WSL2: topics listed but no data | Medium | High | Day-1 hello-multi-container test; Cyclone DDS; one Docker network; `rmw_zenoh` as alternative |
| R4 | (v2) free_fleet does not work with namespaced robots in one domain; source build + zenoh binaries brittle | Medium | Medium (v2 only) | Option D; pin zenoh 1.5.0; Baseline C is not needed for v1 |
| R5 | (v2) RMF delivery tasks need workcells we don't have | Medium | Low | Baseline C uses go-to-place; payload node handles load/unload |
| R6 | CPU thermal limit throttles Gazebo with 3 robots (+ GUI) | Medium | Medium | Headless server, composition, GUI container can be stopped, one sim at a time, 2D backend for scale |
| R7 | Containerized Gazebo GUI via WSLg does not work when started from PowerShell | Medium | Medium | Spike on Day 1; noVNC route (§8.4); Foxglove as last resort |
| R8 | Setting model pose from ROS in Harmonic needs a plugin | Low | Medium | Verify Day 2; small system plugin fallback |
| R9 | Agents edit contracts or overlap files | Medium | Medium | Owned paths, CI interface-diff check, user merges |
| R10 | 2D backend disagrees with Gazebo | Medium | Medium (v2) | Calibration step; tipping-point claim gated on agreement |

# 18. Engineering Considerations

- **Simulation time:** all nodes `use_sim_time: true`; every record stores sim time, wall time is metadata only.
- **Determinism:** seeds and scenario timelines recorded; config, git SHA and image digests stored with results (§12.3).
- **Deadlock and starvation:** lease TTLs, release-on-exit, `OCCUPIED_UNKNOWN` handling, starvation guard (§6.5, §9.2);
  Baseline A stuck timeout (§13.3).
- **Safety:** never traded for speed; Nav2 local avoidance stays active regardless of the fleet layer; robots hold safely
  when the orchestrator is unavailable (§6.5 rule 6); not a certified safety system (§2.3).
- **Explainability:** logs expose the scheduler's actual inputs and outputs (§10).
- **Resources:** monitor Gazebo CPU, process count and Docker resources; composition for Nav2; 2D backend for scale.

**Early validation checklist** (from v1.1 §16, updated): ROS 2 Jazzy + Gazebo Harmonic + Nav2 together (Day 1) ·
multi-robot namespacing and TF (Day 2) · cross-container ROS 2 data flow (Day 1) · package pose-following (Day 2–3) ·
runtime footprint updates (v2) · reservations never grant conflicts (property tests, Day 1–2).

# 19. Future Extensions, Demo, Identity

**Future extensions (out of v1 and v2 scope):** adaptive traffic lanes · package handoffs · rotating cargo platforms ·
heterogeneous fleets · battery-aware scheduling · ML-based congestion prediction · reinforcement learning ·
decentralized coordination · cooperative transport · physical robot deployment.

**v1 demo:** `docker compose up` → three robots pick up and deliver packages in the standard warehouse → the SwarmFlow
orchestrator dispatches orders → robots meet at a one-way storage aisle: under Baseline A they jam and time out, under
FCFS reservations they wait and pass → Foxglove shows positions, plans, zones and decision events.

**v2 demo (final):** normal demand → rising demand congests shared corridors under a baseline → predictive orchestration
enabled, routes and reservations adjust → urgent order reprioritized → a worker blocks a corridor, robots stop/reroute
locally while the orchestrator replans → panel shows explanations, orders, congestion, metrics → benchmark plots compare
Baselines A/B/C with the predictive policy.

**Project identity:** *An interactive warehouse robotics digital twin and experimental fleet-optimization platform built
with ROS 2, Docker and Gazebo.* It does not recreate any company's internal warehouse software or claim to have solved
fleet congestion universally; it provides an extensible, measurable implementation of warehouse coordination techniques.

---

# Appendix A — Approved change traceability (C1–C28)

| # | Approved change (handoff) | Where |
|---|---|---|
| C1 | Replace 9 phases with v1 (5-day MVP) and v2 | §1, §4, §15, §16 |
| C2 | Cuts: fleet expansion cut; descend animation stretch; modes as weight presets; deterministic replay 2D only | §2.3, §4.2, §9.5, §12.3 |
| C3 | Open-RMF section (role, no custom nav, v1 use, v2 replacement, Baseline C, RMF nav-graph format, gate, verification) — **amended by D1**: RMF moved from v1 to v2 Baseline C; gate redefined for the SwarmFlow fleet layer | §5, §15.2 |
| C4 | Non-goals list | §2.3 |
| C5 | Warehouse topology graph as core abstraction; one layout → world, map, graph, 2D layout | §6.1 |
| C6 | Robot agent per robot as only fleet→Nav2 bridge; route → `NavigateThroughPoses`; hold at zone entry | §6.3 |
| C7 | Corridor closures via Nav2 keepout filters (handoff's "v1: RMF lane closures" → Baseline C only after D1; v1 has no closures) | §6.4 |
| C8 | Reservation protocol: request/grant/deny/release, leases + heartbeats, release on exit, orchestrator-down behaviour (implemented in v1 after D1) | §6.5 |
| C9 | `swarmflow_interfaces` frozen Day 1 with concrete definitions | §6.6 |
| C10 | Orchestrator = pure-Python library + thin ROS and 2D adapters | §6.7 |
| C11 | Custom chassis first; robot → payload → aisle table; wide infeasible in dense by construction | §7.1, §7.2 |
| C12 | v2 SmacPlannerHybrid + MPPI with footprint checking; runtime footprint via costmap `footprint` topic | §7.3 |
| C13 | Packages via pose-follower node | §7.4 |
| C14 | Workers as moving primitives; actors only if LiDAR-visible | §7.5 |
| C15 | Multi-robot bringup from Nav2 example, composition, explicit TF namespacing choice | §7.6 |
| C16 | Day-1 environment spike (headless Gazebo, WSLg GUI, `/clock` + `use_sim_time`, RMW choice, hello multi-container) | §8.1 |
| C17 | Images: `dev`, `sim`, one shared robot image via env vars | §8.2 |
| C18 | Baseline A stuck-timeout rule → order failed | §13.3 |
| C19 | Gazebo n runs with CIs; tipping point from 2D only, must agree with Gazebo at 3 robots | §13.2 |
| C20 | Collision metric = pairwise footprint distance, not physics contacts | §13.1 |
| C21 | Foxglove in v1; React panel in v2 | §11 |
| C22 | Decision logging and Foxglove view are v1 features | §10, §11.1 |
| C23 | Contract-first: interfaces, graph schema, orchestrator API, fixtures frozen Day 1 | §14.1 |
| C24 | Workstreams A–F with branches, owned folders, acceptance tests | §14.2 |
| C25 | Single `AGENTS.md` (+ `CLAUDE.md` pointer); per-task contract | §14, `AGENTS.md` |
| C26 | CI is the referee: `colcon build` + `pytest` in `dev`, interface-diff; user merges | §14.3 |
| C27 | Integration is the bottleneck: user + lead own merges and sim debugging | §14.4 |
| C28 | 24/7 guardrails: one Gazebo sim at a time (lock), thermal limit, append-only agent log | §14.5, `AGENTS.md` |

# Appendix B — Sources

Accessed 2026-10-08.

- [S1] Open-RMF main repository and install instructions — <https://github.com/open-rmf/rmf>
- [S2] ROS Index, `rmf_fleet_adapter` (Jazzy 2.7.2 released) — <https://index.ros.org/p/rmf_fleet_adapter/>
- [S2b] ROS Index, `rmf_demos` (Jazzy 2.4.0 released) — <https://index.ros.org/p/rmf_demos/>
- [S3] `rmf_demos` README, `jazzy` branch (Jazzy + Gazebo Harmonic) — <https://github.com/open-rmf/rmf_demos/blob/jazzy/README.md>; `main` (Kilted + Gazebo Ionic) — <https://github.com/open-rmf/rmf_demos>
- [S4] free_fleet README (Nav2 examples, zenoh 1.5.0, Cyclone DDS, namespacing, TODOs) — <https://github.com/open-rmf/free_fleet>
- [S5] Open Robotics Discourse, "Current free fleet package supported ROS distribution" (maintainer: rewrite for Jazzy onwards with easy-full-control adapter and zenoh bridges) — <https://discourse.openrobotics.org/t/current-free-fleet-package-supported-ros-distribution-619/44568>
- [S6] ROS Index, `rmw_zenoh_cpp` (Jazzy 0.2.11 released) — <https://index.ros.org/p/rmw_zenoh_cpp/>
- [S7] ROS Index, `rmf_building_map_tools` (Jazzy 1.9.3 released) — <https://index.ros.org/p/rmf_building_map_tools/>
- [S8] rmf-web README (api-server and dashboard Docker images, `jazzy-nightly`) — <https://github.com/open-rmf/rmf-web>
- [S9] `nav2_bringup` README, `jazzy` branch (multi-robot launch files, composition default) — <https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_bringup/README.md>
- [S10] Nav2 docs, "Navigating with Keepout Zones" and Keepout Filter parameters — <https://docs.nav2.org/tutorials/docs/navigation2_with_keepout_filter.html>, <https://docs.nav2.org/configuration/packages/costmap-plugins/keepout_filter.html>
- [S11] ROS 2 Jazzy `distribution.yaml` (foxglove_bridge release entry) — <https://github.com/ros/rosdistro/blob/master/jazzy/distribution.yaml>
- [S12] Nav2 `nav2_bringup` default parameters, `jazzy` branch (MPPI, NavFn, inflation 0.70 / scaling 3.0, resolution 0.05) — <https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_bringup/params/nav2_params.yaml>
- [S13] PX4 docs, "Gazebo container GUI" (Windows/WSLg section: mount `/mnt/wslg/.X11-unix`, `DISPLAY=:0`, run from the Ubuntu terminal) — <https://docs.px4.io/main/en/simulation/gazebo_container_gui.html>
- [S14] "WSLg with Docker" (WSLg mounts, env vars, `/dev/dxg` + `/usr/lib/wsl` for GPU, compose example; run from a WSL distro) — <https://nes.is-a.dev/out/2025/wslgdocker.html>; Microsoft WSLg container sample — <https://github.com/microsoft/wslg/blob/main/samples/container/Containers.md>
