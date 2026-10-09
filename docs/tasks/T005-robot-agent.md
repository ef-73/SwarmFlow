---
id: T005
title: Robot agent (core state machine + ROS node + fake Nav2 server)
workstream: D
status: review
claimed_by: "sf-implementer"
branch: ws-d/T005-robot-agent
model: sonnet
risk: safety-critical
---

## Goal
Package `src/swarmflow_robot_agent/` (ament_python): the robot-side fleet adapter of design §6.3 — a pure-Python
state machine `agent_core.AgentCore` plus a thin rclpy node `agent_node.AgentNode`, and a fake
`NavigateThroughPoses` server for tests. It is the only bridge from the fleet layer to Nav2.

## Owned files
- src/swarmflow_robot_agent/**

## Lead tests (do not edit)
- src/swarmflow_robot_agent/test/test_agent_core.py
- src/swarmflow_robot_agent/test/test_agent_node.py

## Forbidden
- AGENTS.md §4; no Gazebo, no Nav2 bringup; do not edit contracts (`swarmflow_interfaces`, `api.py`, fixtures) or
  `swarmflow_core`. `agent_core.py` must not import rclpy or any ROS package; time is always passed in.

## Commands to run
```bash
export MSYS_NO_PATHCONV=1 PATH="/c/Program Files/Docker/Docker/resources/bin:$PATH"
export COMPOSE_PROJECT_NAME=swarmflow-<your-worktree-dir-name>
scripts/lock.sh acquire build T005 20     # always release afterwards: scripts/lock.sh release build
docker compose run --rm -T dev bash -c "colcon build --symlink-install --parallel-workers 2 && source install/setup.bash && python3 -m pytest -q -p no:cacheprovider src/swarmflow_robot_agent/test"
```

## Specification
**AgentCore** (`swarmflow_robot_agent/agent_core.py`), constructor `AgentCore(robot_id, graph, traffic_control=True,
rng_seed=0)`; `graph` is a `swarmflow_core.graph.WarehouseGraph`. Event methods take `t` (sim seconds) and return a
list of action objects (dataclasses exported from the module): `Navigate(vertices: tuple[str], poses:
tuple[(x, y, yaw)])`, `CancelNav()`, `RequestReservation(request: api.ReservationRequest)`, `Release(lease_id,
reason: api.ReleaseReason)`, `TaskFinished(task_id, success, failure_reason)`.
- Methods: `start_task(task_id, order_id, route, t, pickup="", dropoff="")`, `on_pose(x, y, yaw, speed, t)`,
  `on_nav_result(success, t)`, `on_reservation_response(request_id, decision: api.ReservationDecision | None, t)`
  (None = the service call failed), `on_orchestrator_heartbeat(t)`, `tick(t)`, `cancel(t)`; queries `state()` →
  `api.RobotSnapshot`, `pending_request_id()` → outstanding request id or `""`, `heartbeat_lease_ids()` → list,
  `can_accept()`. `start_task` while busy raises `RuntimeError`; `can_accept()` is True only in IDLE or STUCK.
- **Pickup / dropoff**: the node passes `order.pickup_vertex` / `order.dropoff_vertex`. If both are empty and
  `order_id` is non-empty: pickup = first vertex of kind `loading` in the route, dropoff = last route vertex.
  Move-only task: `order_id == ""` → no dwell, finished on arrival at the route end.
- **Segments** (§6.3 item 2): a segment ends at the next of: pickup, dropoff, a **hold vertex followed by its zone
  entry**, or the route end. `Navigate.vertices` starts at the vertex after the robot's current one (if the robot
  is already at the segment end, no Navigate is emitted). Pose yaw = heading of the next edge; for the last pose
  of the route, the station yaw if it has one, else the heading of the arriving edge.
- At pickup: mode LOADING for `api.LOAD_DWELL_S`, then continue. At dropoff: UNLOADING for `LOAD_DWELL_S`, then
  `TaskFinished(success=True)` and IDLE. Arrival = `on_nav_result(True)` for a segment ending there.
- **Holds** (traffic_control True): at a hold, emit `RequestReservation` (zone = zone of edge entry→next,
  `exit_vertex` = the zone's other entry vertex along the route, `direction` FORWARD if entering via
  `entries[0]` else REVERSE, `earliest_entry_t = t`, `expected_exit_t = t + zone path length / 0.5 m/s`), mode
  WAITING_RESERVATION. Request ids: `"{robot_id}-{n}"`. Grant → store lease, emit the next segment (through the
  zone). Deny → retry after `retry_after_s` (or `api.RETRY_AFTER_S` if 0) × uniform(1 − RETRY_JITTER,
  1 + RETRY_JITTER) from `random.Random(rng_seed)`. A response to a stale request id is ignored.
- **Release** (§6.5 rule 3): once the pose is outside the zone polygon by ≥ `api.RELEASE_MARGIN_M` (distance from
  the polygon), emit `Release(lease, EXITED)` and drop the lease. `heartbeat_lease_ids()` lists held leases (the node
  publishes them every `HEARTBEAT_PERIOD_S`).
- **Orchestrator down** (§6.5 rule 6): last contact = last orchestrator heartbeat or reservation response. Down if
  no response within `ORCH_RESPONSE_TIMEOUT_S` of a request or `t − last_heartbeat ≥ ORCH_HEARTBEAT_TIMEOUT_S`.
  While down: never emit a Navigate that enters a zone; keep re-requesting at most every `RETRY_AFTER_S`; a robot
  inside a zone keeps its current segment. If `t − last_contact ≥ ORCHESTRATOR_TIMEOUT_S`: mode FAULT,
  fault_reason `ORCHESTRATOR_TIMEOUT`. When contact returns (heartbeat), FAULT → WAITING_RESERVATION, request again.
- **Nav failure** (§6.3 item 5): retry the same segment once; second failure →
  `TaskFinished(False, api.FAILURE_NAV_PREFIX + <segment end vertex>)`, mode STUCK, release held leases (FAULT).
- **Stuck timeout** (§13.5): while NAVIGATING, if the distance to the segment's last pose has not dropped by
  `STUCK_MIN_PROGRESS_M` within `STUCK_WINDOW_S` (reference reset on progress and on each new segment) →
  `CancelNav`, `TaskFinished(False, api.FAILURE_STUCK_TIMEOUT)`, mode STUCK, release leases (FAULT). Waiting is
  never stuck.
- **Baseline A** (traffic_control False): no reservations; segments end only at pickup/dropoff/route end.
- **Cancel**: `CancelNav`, release leases (TASK_CANCELLED), `TaskFinished(False, api.FAILURE_CANCELLED)`, IDLE.
- `state()`: mode, last_vertex (last vertex reached), next_vertex, remaining_route, held_lease_ids, task/order ids.

**AgentNode** (`swarmflow_robot_agent/agent_node.py`, rclpy `Node` subclass named `robot_agent`; `**kwargs` passed
to `Node`): parameters `robot_id`, `layout_dir` (generated dir with nav_graph.yaml + zones.yaml), `traffic_control`
(bool, default true), `map_frame` ("map"), `base_frame` ("base_footprint"), `tick_hz` (10.0).
- Action server `dispatch_task` (relative → `/robot_N/dispatch_task`), Nav2 client `navigate_through_poses`
  (relative), service client `/fleet/request_reservation`, publishers `/fleet/robot_states` (5 Hz,
  header.frame_id "map"), `/fleet/reservation_heartbeat` (1 Hz), `/fleet/reservation_release`; subscriber
  `/fleet/orchestrator_heartbeat` (`std_msgs/Header`). QoS reliable (see the api.py docstring).
- Pose from TF `map_frame` → `base_frame` (tf2_ros Buffer + TransformListener on this node; launch remaps `/tf`).
- Accept a goal only if `can_accept()`; the goal ends when AgentCore emits `TaskFinished`. Feedback = mode,
  current vertex, waiting zone. Must work under a `MultiThreadedExecutor`: use a ReentrantCallbackGroup or async
  calls; never block a callback waiting on a future.
- Console entry point `agent_node = swarmflow_robot_agent.agent_node:main`.

**FakeNavigateThroughPoses** (`swarmflow_robot_agent/fake_nav2.py`): rclpy Node named `fake_nav2` (`**kwargs`
passed to Node) with constructor args `start_xy`, `speed_mps`; serves `navigate_through_poses`
(`nav2_msgs/action/NavigateThroughPoses`), moves a simulated pose along the goal poses at `speed_mps`, publishes TF
`map`→`base_footprint` on `tf` at 20 Hz, sends feedback, succeeds at the end; supports cancel; method
`fail_next(n)` makes the next n goals abort.

`package.xml` exec_depends: rclpy, swarmflow_interfaces, swarmflow_core, nav2_msgs, geometry_msgs, std_msgs,
tf2_ros (verify each exists in the dev image with `ros2 pkg list`); add `setup.py`, `setup.cfg`, resource marker.

## Definition of done
- Both lead test files pass unchanged; add your own tests for anything the spec covers that they don't.
- `colcon build` + `colcon test --packages-select swarmflow_robot_agent` green. Commit on your branch
  ("T005: robot agent"), set the card status to `review`. Report exact test output and anything unverified.

## Inputs / contracts used
- docs/design.md §6.3, §6.5, §13.5; api.py (constants, ReservationRequest/Decision, RobotSnapshot);
  swarmflow_interfaces (DispatchTask, RequestReservation, RobotState, ReservationHeartbeat/Release);
  tests/fixtures/standard/
