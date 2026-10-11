# SwarmFlow v2 improvements — draft (2026-10-10, rev 2)

Status: **draft, being edited with the user.** Done so far (2026-10-11): `v1.0.0` tagged; the Foxglove view and
both dashboards (§7: V-01 … V-08, V-05b, V-10) in T021/T022, on `main`. Everything else below is still open. Nothing here is a task card yet. Once the open questions (§10)
are answered, the lead turns the items into cards under `docs/tasks/`.

Tags: **[contract]** changes a frozen contract (AGENTS.md §2) and needs a `lead/contract-*` branch and the user's
approval · **[sim]** needs Gazebo runs by the lead · **S/M/L** = rough size.

## 0. Direction (user decisions, 2026-10-10)

1. **A larger, realistic warehouse.** Two blocks of 5 shelf rows each, with a wide central aisle between them.
   Truck docks at one end of the central aisle, a shared home area at the other, and a pallet area in between.
2. **Picking = arriving.** A robot completes a shelf pick by reaching the pick point; no load/unload animation is needed.
3. **The orchestrator only assigns.** It decides *which robot does what, and where it goes*. It no longer controls
   aisles or junctions: no more aisle reservations by default.
4. **Robots handle each other locally.** Each robot plans its own path (A* on its costmap from lidar) and avoids
   other robots on its own, including predicting where a moving robot is going and probing for a way through when blocked.
5. **Know what we get for free.** List every piece of information the simulation hands the robots for free
   ("privileged" information) and measure how much it helps.
6. **Foxglove is the main view.** It must look like the Gazebo world; the Gazebo window is off by default.

## 1. New warehouse layout (`layouts/warehouse_v2/`)

Proposed shape (the long axis of the building runs south → north):

```
 north wall
 ┌──────────────────────────────────────────────┐
 │   [DOCK 1]  [DOCK 2]  [DOCK 3]  [DOCK 4]       │  truck docks (drop-off)
 │                                                │  dock apron
 │  ══════ row W5 ══════   ║   ══════ row E5 ══════  │
 │   storage aisle         ║    storage aisle      │
 │  ══════ row W4 ══════   ║   ══════ row E4 ══════  │
 │        …               CENTRAL       …            │  5 rows per block,
 │  ══════ row W1 ══════  AISLE   ══════ row E1 ══════  │  pick points on both faces
 │                         ║                      │
 │   [pallet stands]       ║     [pallet stands]  │  pallet area
 │                                                │
 │      [H] [H] [H] [H] [H] [H]  home area        │  home slots, first come first served
 └──────────────────────────────────────────────┘
 south wall
```

| ID | Item | Tags | Size |
|---|---|---|---|
| L-01 | **Dimensions (proposal).** Central aisle **4.0 m** clear: two loaded pallet robots pass each other (0.9 m padded width each, design §7.2 two-way rule ≥ 2.1 m), or three unloaded ones. Storage aisles between rows **2.2 m**: two robots can pass, so a robot standing at a pick point does not block the aisle. Rack rows 1.0 m deep × ~8 m long. **Storage aisles are closed at the outer wall (dead ends, decided)**: a robot enters from the central aisle and leaves the same way, turning around inside (2.2 m allows turning in place, which needs 0.92 m). Approximate building: ~20 m × 32 m (v1: 22 × 12.3 m). | [sim] | M |
| L-02 | **Pick points.** Route-graph vertices along both faces of every storage aisle (e.g. every 2 m), placed off the aisle centre towards the rack face so a picking robot leaves room to pass. Name pattern e.g. `PK_W3_N_04` (block W, row 3, north face, slot 4). | [contract] (layout schema: new station type) | M |
| L-03 | **Truck docks with queue lanes.** 4 dock stations on the north wall at the end of the central aisle. Each dock has a drop point and a straight **queue lane** behind it with 3 waiting spots (spaced for a loaded pallet robot, ~1.6 m apart), off the central aisle so a queue never blocks through-traffic. | [contract] (station type) | S |
| L-04 | **Home area.** N home slots (N ≥ fleet size) at the south end. **Not tied to a robot id**: an idle robot goes to the nearest free slot. The v1 park rule already works this way; robots only *spawn* at a given slot. | — | S |
| L-05 | **Pallet area.** A grid of pallet positions between the home area and the shelves (see §5). | [contract] (station type) | S |
| L-06 | **Generator support.** `tools/layoutgen` builds world/map/graph for the new layout; the v1 `standard` layout stays for comparison runs. | — | M |
| L-07 | **Schema changes, one contract branch:** station types `pick`, `dock`, `home`, `pallet`; zones become optional (no zones needed by default); zone `capacity` may be > 1 (today pinned to 1). | [contract] | S |

## 2. Orchestrator: assignment only

The orchestrator's job, precisely:

| It decides | It does **not** decide |
|---|---|
| Which robot takes which order (the order names its truck dock) | The path a robot drives |
| The task's **goals in order**: pick point(s) → dock; or pallet → dock; or → home | Who may enter an aisle or junction |
| Sending idle robots to a free home slot | How robots pass or avoid each other |
| Reacting to failures: reassign, cancel, send home | Speeds, recoveries, any wheel command |
| Keeping the record: order states, decisions with reasons, run logs | Relaying positions between robots (robots read each other directly) |

**Inputs:** orders (`/fleet/orders`: picks or pallet, truck dock, priority, deadline); robot states
(`/fleet/robot_states`: pose, velocity, mode, current task, goal reached / failed / BLOCKED); task results.

**Outputs:** one task per robot (`/robot_N/dispatch_task`: ordered goal points); order status
(`/fleet/order_status`); decisions with explanations (`/fleet/decisions`); run logs.

**Loop (5 Hz):**
1. Release orders whose release time has come → `QUEUED`.
2. Update each order from its robot's reports: goal reached → next goal; last pick reached → `IN_TRANSIT`;
   dock reached → `DELIVERED`.
3. Handle failures: a robot that reports a failed or BLOCKED task for longer than N s → if nothing is on board yet,
   the order goes back to `QUEUED` and the robot is sent home. If the robot already carries picks or a pallet, it
   gets up to K retries to its next goal (another robot can't take over its load), then the order is `FAILED`.
4. Assign: for each queued order (by priority, then age), pick the available robot with the lowest cost
   (travel to the first goal + congestion + order age). Goal order inside the task: picks nearest-next, then the
   order's truck dock.
5. Dock queues: a robot whose dock is busy goes to the next free waiting spot; waiting robots move up as the dock frees.
   Unloading: 5 s for boxes, 10 s for pallets.
6. Idle robots not at home → nearest free home slot (any slot, first come first served).
7. Log every decision with its reason.

**When it is down:** robots finish their current goal and stop there; they keep avoiding each other, because
that does not depend on the orchestrator. No new tasks until it is back; on restart it rebuilds its state
from the robots' reports.

| ID | Item | Tags | Size |
|---|---|---|---|
| O-01 | **Goal-based tasks.** A task is an ordered list of goal *points* (not a vertex-by-vertex route). The robot agent sends each to Nav2 as one `NavigateToPose`; Nav2's planner chooses the path. `DispatchTask.route` can carry the goal list without a message change, but its documented meaning changes. | [contract] (semantics) | M |
| O-02 | **Reservations off by default.** The v1 reservation authority stays in the code only as a baseline (`traffic_control: true`) for comparison runs. The robot agent drops holds and hold vertices in the default mode. | — | M |
| O-03 | **Better assignment cost.** Travel estimate from the robot to the first goal, plus a congestion penalty (robots already heading into the same aisle or dock), plus order age. Still deterministic and logged with an explanation. | — | M |
| O-04 | **Multi-pick orders.** An order = 1..N pick points + its assigned truck dock (decided: every order names its dock). Visit order chosen by the orchestrator (nearest-next, aisle by aisle). | [contract] (`Order.msg`, `OrderSpec`) | M |
| O-05 | **Dock queues (decided 2026-10-10).** Unloading takes **5 s for boxes, 10 s for pallets** (sim time). If a robot's dock is busy, the orchestrator sends it to the next free waiting spot in that dock's queue lane instead of the drop point, and moves each waiting robot up one spot when the one ahead leaves. After unloading, the robot leaves with its own planner like any other move; the orchestrator only gives it its next task or a home slot. | — | M |
| O-08 | **Failure handling stays central.** A robot that reports BLOCKED for N s gets its order requeued (nothing on board) or retried, and is sent home if needed. No other traffic rules in the orchestrator. | — | S |
| O-06 | **Leftover v1 decisions:** operator recovery override; lease ids with a generation prefix. Both only matter if reservations are kept as a baseline. | [contract] | S |
| O-07 | **Manual orders.** Add orders by hand from Foxglove (Publish panel → `/fleet/orders`) and via a small CLI script. | — | S |

**On waypoints vs a single endpoint.** A single endpoint per goal is the default; the robot's own planner finds
the path. Traffic conventions live in the map, not in the orchestrator: see the keep-right rule, N-09 (decided).

## 3. Local navigation: robots that handle each other

**Why the driving looks wrong today** (from the v1 config, `src/swarmflow_nav/`):
- **Turning in circles:** the recovery tree contains a `Spin` (90°) recovery, run whenever planning or path-following
  fails, e.g. when another robot blocks the way.
- **Driving into robots it could go around:** the *global* costmap has no obstacle layer (removed in M8 because stale
  marks of other robots caused detours), so the global path runs straight through the other robot. The path-following
  critics then pull the robot towards it. The path is only replanned every 3 s, and the local costmap is only
  4 × 4 m, too small to see a way around.
- **Extra turning at waypoints:** v1 sends a pose for every graph vertex, each with a fixed heading, so the robot
  aligns to each one.

| ID | Item | Tags | Size |
|---|---|---|---|
| N-01 | **Global path planning with obstacles.** Global costmap gets the lidar obstacle layer back, with fast clearing so moved robots do not leave stale marks. A* planner (Smac 2D or NavFn with A*). Replan at 1–2 Hz. | [sim] | M |
| N-02 | **Bigger local view.** Local costmap 6 × 6 m or more; MPPI look-ahead long enough to plan around a robot. | [sim] | S |
| N-03 | **Recovery behaviour that makes sense.** Remove `Spin`. New order: replan → wait 1–3 s (random, so two blocked robots do not move in lockstep) → **probe** (N-04) → back up → report BLOCKED to the orchestrator. | [sim] | M |
| N-04 | **"Probe" behaviour (your poking-around idea).** When blocked, drive along the best partial path as far as is collision-free, stop, replan from there; repeat a few times. Implemented as a custom behaviour-tree node. | [sim] | M |
| N-05 | **Shared position + velocity (decided 2026-10-10; replaces lidar tracking as the main prediction).** Every robot already publishes its pose and speed on `/fleet/robot_states`. A small node in each robot predicts where every *other* robot will be over the next ~3 s and feeds that into its own navigation, in two stages: **(a)** mark the predicted footprints as cost in the local costmap (simple, a bit over-cautious); **(b)** a custom MPPI critic that checks each candidate trajectory against the other robots' predicted positions *at the same moment* (accurate: a robot moving away does not block). Robots read each other's states directly; the orchestrator is not in this loop, so avoidance keeps working if it goes down. Better than velocity alone: also share each robot's current Nav2 path (`/robot_N/plan`, already published), which shows turns ahead. Latency and the pose error from localisation are part of the realism ladder (I-01). | [sim] | L |
| N-05b | **Lidar-only tracking (optional, later).** Cluster and track lidar points not in the map. Needed only for things that do not report their position (people, if v2 adds workers). | [sim] | L |
| N-06 | **360° from a single lidar.** Today the robot has **one 180° front lidar** (−90° … +90°) and is blind to the sides and behind. Decided: 360° with as few lidars as possible, i.e. **one**. A lidar on top won't work: the robot drives under pallets, and above 0.30 m it would look over the 0.30 m-high robots. So the lidar sits in the centre of the robot at ~0.20 m, in a **sensor slot**: the chassis gets a horizontal gap around its full circumference at lidar height, with the top deck carried by 4 thin posts. The posts cast 4 narrow shadows (a few degrees each), which are masked out of the scan. Pallet legs are filtered the same way while carrying (F-05). | [sim] | M |
| N-07 | **Re-tune MPPI** for the new layout: critic weights so avoidance beats path-following when blocked; inflation that lets two robots pass in a 2.2 m aisle. | [sim] | M |
| N-09 | **Keep-right rule (decided 2026-10-10).** Robots drive on the right-hand side of the central aisle and the storage aisles. Implemented as a soft cost in every robot's map: the left half of each aisle (relative to the direction of travel) costs more, so the planner prefers the right half but can still use the left to get around an obstacle. Needs a direction-dependent cost: either a Nav2 costmap filter per travel direction or a custom planner cost (mechanism to verify). The layout generator produces the lane masks. | [sim] | M |
| N-08 | **Deadlock check.** Two robots each waiting for the other: random waits + probe should break it; if not, a simple local rule (e.g. the robot with the lower id backs off). Measured in the benchmarks. | [sim] | S |

## 4. Shelf picking

| ID | Item | Tags | Size |
|---|---|---|---|
| P-01 | **Pick = arrival.** Reaching a pick point (within tolerance) counts as a successful pick, no wait. | — | S |
| P-02 | **Order generator** for shelf orders: pick points from the slot list (optionally popular slots), 1–N picks, a dock. Seeded and reproducible as today. | — | S |
| P-03 | **Visual only:** a small box appears on the robot after its first pick (optional). | — | S |

## 5. Pallet moving (forklift-style)

The robot drives **underneath** a pallet, lifts it, carries it to a dock, and sets it down.

| ID | Item | Tags | Size |
|---|---|---|---|
| F-01 | **Pallet model.** A pallet on four corner legs (a "pallet stand") with a gap wide and high enough for the robot to drive under, e.g. 0.85 m gap and 0.35 m clearance for the 0.60 m wide robot. Decided: pallet stands. | [sim] | M |
| F-02 | **Docking under the pallet.** Nav2 will not drive between legs only 0.1 m away from the robot on each side, so the final approach needs a precise docking behaviour (e.g. the Nav2 docking server, to verify for Jazzy) using the leg positions seen by lidar. | [sim] | L |
| F-03 | **Lift and carry.** A lift mode in the robot model; the pallet follows the robot (as packages do in v1). | [sim] | M |
| F-04 | **Footprint switch.** Loaded, the robot is the size of the pallet (~1.2 × 1.0 m): its Nav2 footprint must switch while carrying and switch back after drop-off. | [sim] | M |
| F-05 | **Lidar self-filter.** While carrying, the pallet's own legs are in the lidar's view; filter them out of the scan. | [sim] | S |
| F-06 | **Pallet orders.** Order type "move pallet P to dock D". New robot modes (e.g. LIFTING, CARRYING). | [contract] (`Order.msg`, `RobotState` modes) | M |

## 6. Privileged information audit

What the simulation currently hands the robots for free, and the realistic alternative:

| Information | v1 today | Realistic version |
|---|---|---|
| Robot position | **Gazebo ground truth**: perfect pose (`localization:=ground_truth` is the default) | AMCL from lidar + map (already an option, not yet used in runs) |
| Wheel odometry | perfect, no slip | add noise and slip |
| Lidar | 1 cm noise, no dropouts | more noise, dropouts, reflections (minor) |
| Map | exact, generated from the layout file | **Decided: assume the warehouse was scanned beforehand.** The generated map stands in for that scan. It holds only fixed things (walls, racks, docks); pallets and robots are **not** in it, so the lidar picks them up as obstacles wherever they are. Optional: a one-time SLAM run to show the generated map matches a scanned one. |
| Other robots, to Nav2 | **lidar only** (already honest) | same |
| Other robots, to the orchestrator | each robot's self-reported pose, exact because of ground truth | the same reports, but from AMCL, with delay |
| Network | instant, lossless | latency and packet loss |
| Pick / lift | instant (pick by design, user decision) | — |

**Sending all lidar to the orchestrator** is possible: every `/robot_N/scan` topic is already visible to any node.
The orchestrator could merge them into one fleet-wide occupancy map. But that is *more* shared knowledge, not less.
Use it only as an experiment, not as a way to reduce privilege.

| ID | Item | Tags | Size |
|---|---|---|---|
| I-01 | **"Realism ladder" experiment.** Run the same scenario at increasing realism: (1) ground truth, (2) AMCL + odometry noise, (3) + network delay/loss, (4) + lidar degradation. Measure throughput, near misses, stuck count, localisation error. This answers how much easier the privileged information makes things. | [sim] | M |
| I-02 | **Fleet-shared perception (optional experiment).** All scans → a merged map of *moving* obstacles, sent back to every robot, so a robot can "see" a robot around a corner before its own lidar does (mainly at aisle ends). Compare against lidar-only robots: if near misses at aisle ends don't drop, it isn't worth the extra traffic. | [sim] | M |
| I-03 | **Make the realistic setting the default** once AMCL is stable on the new layout. | [sim] | S |

## 7. Foxglove: the main view

| ID | Item | Tags | Size |
|---|---|---|---|
| V-01 | **Gazebo window off by default** for user runs (`SWARMFLOW_GUI=none`); the GUI only on request. | — | S |
| V-02 | **One coordinate-frame tree.** All three robots use the same frame names (`base_link`, `lidar_link`) on separate per-robot TF topics, so Foxglove cannot place lidar scans, paths or costmaps. Prefix the frames (`robot_N/base_link`) and publish one global `/tf`. | [sim] | M |
| V-03 | **Real robot models** in Foxglove from each robot's URDF (`robot_description`), in its colour. | — | S |
| V-04 | **The warehouse in 3D:** walls, racks at their real heights, stations, docks, home slots, pallets and packages, generated from the layout file, so Foxglove matches Gazebo. | — | M |
| V-05 | **Lidar per robot**, coloured like its robot. | — | S |
| V-05b | **Navigation overlay, per robot in its own colour (user request):** robot body in its colour; its **footprint outline** (the shape Nav2 plans with, bigger while carrying a pallet); the **path it is following, drawn on the floor**; its current goal. Fixed symbols for places: pick points (small dots), pallet spots (squares), truck docks (truck/arrow symbol with dock number), queue spots (chevrons), home slots (house/H). A pick point that is the goal of an active task is highlighted in the robot's colour. Shown in Foxglove (main view); the same in Gazebo's GUI only if cheap. | — | M |
| V-06 | **Less lag:** the Foxglove bridge forwards only the topics the layout uses; heavy topics (3D voxel maps, full costmaps) off by default. Measure bandwidth and CPU before and after. | [sim] | S |
| V-07 | **Layout file that imports** into current Foxglove: 3D view framing the whole warehouse, order table, decision log, a per-robot task panel (mode, task, current goal), a Publish panel preset for new orders. | — | S |
| V-08 | **Check against Gazebo** in Chrome: Foxglove and Gazebo side by side, same moment, same positions. | [sim] | S |
| V-10 | **Robot dashboard in Foxglove and Gazebo (user request).** One row per robot: mode, task, order, current goal, speed, carried load, time since last report, problems (stuck/fault) highlighted. Foxglove: a status table panel. Gazebo: a status label floating above each robot plus a status board (Gazebo has no free-floating 2D panel without a custom GUI plugin). | — | M |
| V-09 | **Web page (optional, later)** only if Foxglove still falls short. | — | L |

## 8. Performance, evaluation, v1 leftovers

- **C-01** Profile CPU per container. A bigger world adds little; each extra robot (Nav2 + MPPI) is the main cost.
  Lidar tracking (N-05) and a 360° lidar add some.
- **C-03** **Two run modes (user request: don't use all cores).** *Full mode*: Gazebo + Nav2 per robot, for realistic
  navigation and the benchmarks; 4–5 robots. No hard core cap and no reduced-quality settings (user decision). *Light mode*: the 2D kinematic sim
  (`sim2d/`) with simplified local avoidance, no Gazebo, no Nav2. Same orchestrator, same Foxglove view, about one
  core even for 10+ robots. Use it to watch and tune assignment; use full mode to check driving.
- **E-01** Benchmark on the new layout: orders/hour, pallet moves/hour, average task time, near misses (closest
  approach), stuck/blocked count. Compare: v1 reservations vs local-only navigation, ± prediction (N-05), ± realism (I-01).
- **E-02** 3+ demo runs of 10 min each.
- **v1.1 leftovers:** container healthchecks and restart policies; `ci.sh` branch names like `T018b`; no fixed
  sleeps in tests; occasional GitHub Actions failures; license.

## 9. Suggested order

0. **Clean-up and baseline first** (§12): X-02, X-03, X-05, X-08 (small tidy-ups), X-01 (cheaper `viz`),
   X-10 (baseline numbers on the current layout). X-04 and X-07 are done. X-06 (v1.1 robustness) runs alongside the later steps.
1. **Foxglove** (V-01 … V-10): **done** (T021, T022).
2. **Contract branch**, approved in one go: new station types and optional zones (L-07), goal-based tasks (O-01),
   multi-pick orders (O-04), pallet orders and modes (F-06).
3. **New layout** (L-01 … L-06).
4. **Local navigation** (N-01 … N-04, N-07, keep-right N-09), then prediction (N-05) and 360° lidar (N-06).
5. **Orchestrator simplification** (O-02, O-03, O-05) and **shelf picking** (P-01, P-02).
6. **Pallets** (F-01 … F-05).
7. **Realism ladder and benchmarks** (I-01, E-01, E-02).

The light mode (X-09 / C-03) is built before step 3, so steps 3–6 can be tried in the 2D sim before Gazebo.

## 10. Decisions (answered 2026-10-10)

1. **Storage aisle ends:** closed (dead ends at the outer wall).
2. **Pallets:** pallet stands on four legs, robot drives underneath.
3. **Fleet size:** 4–5 robots is fine. CPU: a light mode (C-03); no hard core cap and no reduced-quality settings.
4. **Docks:** every order names its truck dock.
5. **Lidar:** 360° with one lidar in a sensor slot (N-06).
6. **v1.0.0:** tag the current version before the rework (done). Rule change: the lead may create tags after explicit approval of each tag (AGENTS.md §4).
7. **Keep right** in all aisles (N-09).
8. **Map:** assume the warehouse was scanned beforehand; no SLAM needed (§6).
9. **Shared lidar:** no. Share position + velocity instead (N-05).
10. **Dock unloading:** 5 s boxes, 10 s pallets; robots queue in a lane per dock (O-05).
11. **Navigation overlay** per robot in its colour: footprint, path on the floor, goal; symbols for places (V-05b).

## 12. Further items (lead review, 2026-10-11; all accepted by the user)

| # | Suggestion | Why | Size |
|---|---|---|---|
| X-01 | Make the `viz` container cheaper (profile it; move the TF relay to C++ or drop high-rate frames at the source) | It still uses ~0.9 core with 3 robots | S–M |
| X-02 | Remove the v1 viz leftovers (`/fleet/robot_markers`, `/fleet/zone_markers`, `markers.py`, `swarmflow_v1.json`) | Duplicate work and topics now that v2 views exist | S |
| X-03 | Update stale design text: §11.1 (Foxglove v1 file, a global `/map` nobody publishes), §8.3 service table (`gazebo_gui` profile, `viz` on the sim image) | Docs no longer match the stack | S |
| X-04 | **Done** (user approved): `src/swarmflow_gz_panel/` added to WS-E in AGENTS.md §3 | New package has no owner | S |
| X-05 | Housekeeping: 2 stale worktrees, 4 empty folders in `.claude/worktrees/`, 72 unused Docker volumes (~0.4 GB), ~16 GB Docker build cache, duplicate "swarmflow_v2" layouts in your Foxglove account | Disk and clutter | S |
| X-06 | v1.1 leftovers from the retrospective: container healthchecks + restart policies, 19 fixed sleeps in tests, `ci.sh` branch names like `T018b`, batch scripts that kill their children, a "control loop missed" counter, the undiagnosed GitHub Actions failures | Robustness of runs and CI | M |
| X-07 | **Done:** MIT (user decision 2026-10-11), `LICENSE` + all package manifests | Still missing | S |
| X-08 | Clear the `[U]` (unverified) notes in `payload_node.py`; the code has worked in every run since M4 | Misleading comments | S |
| X-09 | Run the 2D sim (`sim2d/`) as the planned light mode early, before the new layout, so new assignment logic can be tested without Gazebo | Saves Gazebo time on every later step | M |
| X-10 | One benchmark scenario on the *current* layout before the rework (baseline numbers to compare v2 against) | Otherwise v2 has nothing to beat | S |

Non-blocking actions for the user live in [`docs/user_todo.md`](user_todo.md).

## 11. Your additions

<!-- add items here; give them any id, the lead will renumber -->
