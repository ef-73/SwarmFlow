# sim2d

Stdlib + PyYAML 2D kinematic simulator (`Sim2DBackend`, benchmark runner) plus **light mode**.

## Light mode (no Gazebo)

Kinematic robots driven by the *unchanged* `swarmflow_orchestrator`, scenario engine and one `swarmflow_robot_agent`
per robot. `sim2d.light` publishes `/clock`, serves `/<robot>/navigate_through_poses`, and publishes
`/<robot>/tf` (odom -> base_footprint), `/<robot>/tf_static` (map -> odom, identity) and `/<robot>/plan`.
The orchestrator, engine and agents run as child processes (logs in `<run-dir>/logs/`) on sim time.

```bash
# inside the dev/robot image with the workspace built and sourced
PYTHONPATH=sim2d python3 -m sim2d.light --layout-dir layouts/standard/generated \
    --robots robot_1,robot_2,robot_3 --scenario scenarios/v1_demo.yaml --run-dir runs/light-1 \
    [--rtf 1] [--duration-s 600]
python3 tools/metrics/compute.py runs/light-1
# with Foxglove: docker compose -f docker/compose.light.yaml up   (viz + foxglove_bridge on :8765)
```

The run record (`orders.jsonl`, `robot_states.jsonl`, `decisions.jsonl`, `config.yaml`, ...) is the one the Gazebo
stack leaves. Exits 0 after `--duration-s` sim seconds.

### What it simplifies
- No lidar, no Nav2, no costmaps, no walls: robots follow straight lines between the poses they are sent. Each pose
  is first expanded along the nav graph (`light_paths.expand`, the stand-in for Nav2's planner: nearest vertex,
  `shortest_path`, nearest vertex to the goal, then the goal; hops of 1 m or less go straight), so v2 goal tasks do not
  cut through racks.
- Default speed is 1.0 m/s (`--speed`; the real robot is limited to 0.6) and the step is 0.1 s (`--dt`, also the
  `/clock` and tf period). At 0.6 m/s a 300 s `v2_demo` delivers only 1 to 5 orders; at 1.0 m/s 6 to 7.
- Unicycle (turn in place when the heading error is > 0.5 rad), 1 m/s^2 acceleration limit; intermediate pose
  yaws are ignored, the last pose's yaw is honoured.
- Avoidance (`light_world.py`): a robot stops when another footprint is within 0.6 m ahead (state `blocked`); a
  lower id has right of way over a higher id on a crossing course; a head-on standoff is broken by the higher id
  side-stepping 0.85 m to its right and waiting until the other robot has passed. A hard layer never lets a robot
  move into an overlap, so clearance violations cannot occur from motion (they would only appear if robots are
  spawned or commanded overlapping).
- Deadlock breaker (free space): a robot blocked by a parked robot goes around it (side-step candidates plus a ring of
  spots, tried in a fixed order, only to spots reachable without touching anyone; recent spots are avoided) after 2 s;
  if its final goal is within 1 m of the parked robot it settles and reports arrived after 1.5 s. A robot blocked by a
  busy robot that is itself stopped yields if its id is the higher one (after 4 s, or 0.5 s head-on): it leaves the
  other robot's lane and waits until it has passed. A turn that would touch a neighbour is dropped so the robot can
  still drive; a jammed turn is replaced by a slide (0.3 m/s) to the side-step spot. Mostly resolves random dense
  traffic (the 8-robot 12 x 8 m stress test), but 3 of 40 extra seeds still end in a livelock after 400 s.
- Robot-to-robot physics (collisions, slip) and payload are not modelled.

### v2 (`policy: assign`, `warehouse_v2`, goal tasks)
```bash
PYTHONPATH=sim2d python3 -m sim2d.light --layout-dir layouts/warehouse_v2/generated     --robots robot_1,robot_2,robot_3,robot_4,robot_5 --scenario scenarios/v2_demo.yaml --run-dir runs/light-v2     --rtf 4 --duration-s 300 [--trace runs/light-v2/trace.jsonl]
```
`--trace` writes a 1 Hz world snapshot (pose, state, goals, blocker, detour per robot) for debugging stalls. The
final line also prints the true minimum footprint distance in the world. The `robot_states.jsonl` the metrics read
is sampled by the agents from tf, so on a starved host a stale pose can show a clearance violation the world did
not have: keep the host quiet (no Gazebo) for benchmark-like runs. If the host stalls, sim time slows down
instead of jumping (at most 0.5 s of catch-up per loop), so agents are not declared lost.

### CPU of 5-robot v2 (dev container, `v2_demo`, rtf 1, 150 sim s, step 0.1 s)
- 92 cpu-s over 150 s = **0.61 cores** average (light process 9 s, children 83 s). rtf 4 about 2 cores
  (300 sim s in 75 s wall). `v2_demo` for 300 sim s at rtf 4 delivers 6 to 7 orders with 0 clearance violations and
  0 stuck events (3 runs in a row; the ROS stack, not the kinematics, is the cost).

### CPU (dev container, `v1_demo`, 3 robots, rtf 1, 600 sim s)
- Whole stack (light process + orchestrator + scenario engine + 3 agents): 190 cpu-s over 600 s = **0.32 cores**
  average (light process 24.5 s, children 165.6 s). At rtf 4: about 1.0 core.
- `LightWorld.step`: about 1.5 us per robot per step with 10 robots (0.1 s steps), so the world is negligible; the
  cost is the ROS nodes.
