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
- No lidar, no Nav2, no costmaps, no walls: robots follow straight lines between the poses they are sent.
- Unicycle (turn in place when the heading error is > 0.5 rad), 1 m/s^2 acceleration limit; intermediate pose
  yaws are ignored, the last pose's yaw is honoured.
- Avoidance (`light_world.py`): a robot stops when another footprint is within 0.6 m ahead (state `blocked`); a
  lower id has right of way over a higher id on a crossing course; a head-on standoff is broken by the higher id
  side-stepping 0.85 m to its right and waiting until the other robot has passed. A hard layer never lets a robot
  move into an overlap, so clearance violations cannot occur from motion (they would only appear if robots are
  spawned or commanded overlapping). Free-space avoidance can deadlock in crowded open space without
  the orchestrator's zone reservations.
- Robot-to-robot physics (collisions, slip) and payload are not modelled.

### CPU (dev container, `v1_demo`, 3 robots, rtf 1, 600 sim s)
- Whole stack (light process + orchestrator + scenario engine + 3 agents): 190 cpu-s over 600 s = **0.32 cores**
  average (light process 24.5 s, children 165.6 s). At rtf 4: about 1.0 core.
- `LightWorld.step`: about 1.5 us per robot per step with 10 robots (0.1 s steps), so the world is negligible; the
  cost is the ROS nodes.
