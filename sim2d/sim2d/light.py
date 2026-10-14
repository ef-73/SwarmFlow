"""Light mode (T025): kinematic robots driven by the real orchestrator, scenario engine and robot agents.

    python3 -m sim2d.light --layout-dir layouts/standard/generated --robots robot_1,robot_2,robot_3 \\
        --scenario scenarios/v1_demo.yaml --run-dir runs/light-1 [--rtf 1] [--duration-s 600]

This process is the "simulator": it publishes ``/clock``, serves ``/<robot>/navigate_through_poses`` from a
:class:`sim2d.light_world.LightWorld`, and publishes ``/<robot>/tf`` (odom -> base_footprint), ``/<robot>/tf_static``
(map -> odom, identity) and ``/<robot>/plan``. The orchestrator, scenario engine and one robot agent per robot run
unchanged as child processes with ``use_sim_time:=true``; the run record is the one the Gazebo stack leaves.
Needs ROS 2 (rclpy, nav2_msgs, ...) and the built SwarmFlow workspace on the path.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import pathlib
import signal
import subprocess
import sys
import threading
import time
from typing import Dict, List, Optional, Tuple

import yaml

from .light_world import LightWorld

ROOT = pathlib.Path(__file__).resolve().parents[2]
MAX_LAG_S = 0.5          # most sim time one loop iteration may catch up
START_T = 1.0            # sim time at start (a stamp of 0 means "static" to the agent)


def _yaw_of_q(q) -> float:
    return 2.0 * math.atan2(q.z, q.w)


def spawn_poses(layout_dir: pathlib.Path, robots: List[str]) -> Dict[str, Tuple[float, float, float]]:
    """Robot id -> (x, y, yaw) from the layout's ``sim2d.json`` spawn table (free park vertices for the rest)."""
    data = json.loads((layout_dir / "sim2d.json").read_text(encoding="utf-8"))
    verts = data["vertices"]
    table = {s["robot_id"]: (s["vertex"], float(s.get("yaw", 0.0))) for s in data.get("spawn", [])}
    used = {table[r][0] for r in robots if r in table}
    free = [n for n in sorted(verts) if verts[n].get("kind") == "park" and n not in used]
    out = {}
    for rid in sorted(robots):
        vname, yaw = table[rid] if rid in table else (free.pop(0), 0.0)
        out[rid] = (float(verts[vname]["x"]), float(verts[vname]["y"]), yaw)
    return out


def _child_cmd(module: str, ros_args: List[str]) -> List[str]:
    return [sys.executable, "-c", f"from {module} import main; main()", "--ros-args", *ros_args]


def _ros_params(d: Dict[str, object]) -> List[str]:
    out: List[str] = []
    for k, v in d.items():
        if isinstance(v, bool):
            v = "true" if v else "false"
        elif isinstance(v, (list, tuple)):
            v = "[" + ",".join(str(i) for i in v) + "]"
        out += ["-p", f"{k}:={v}"]
    return out


def main(argv: Optional[List[str]] = None) -> int:
    ap = argparse.ArgumentParser(prog="sim2d.light", description=__doc__.splitlines()[0])
    ap.add_argument("--layout-dir", required=True, help="generated layout directory (sim2d.json, nav_graph.yaml, ...)")
    ap.add_argument("--robots", default="robot_1,robot_2,robot_3")
    ap.add_argument("--scenario", required=True, help="scenario yaml path or name under scenarios/")
    ap.add_argument("--run-dir", required=True)
    ap.add_argument("--rtf", type=float, default=1.0, help="sim seconds per real second")
    ap.add_argument("--duration-s", type=float, default=0.0, help="stop after this many sim seconds (0 = until ^C)")
    ap.add_argument("--dt", type=float, default=0.1, help="sim step in seconds (also /clock and tf period)")
    ap.add_argument("--speed", type=float, default=1.0,
                    help="max linear speed, m/s (the real robot is limited to 0.6, design 8.2; 1.0 lets a 300 s v2 run "
                         "deliver enough orders)")
    ap.add_argument("--turn", type=float, default=1.5)
    ap.add_argument("--trace", default="", help="write a 1 Hz world trace (poses, states, goals) to this jsonl file")
    ap.add_argument("--startup-timeout-s", type=float, default=90.0)
    args = ap.parse_args(argv)

    layout_dir = pathlib.Path(args.layout_dir).resolve()
    robots = [r.strip() for r in args.robots.split(",") if r.strip()]
    scen = pathlib.Path(args.scenario)
    if not scen.exists():
        scen = ROOT / "scenarios" / f"{args.scenario}.yaml"
    scen = scen.resolve()
    policy = str(yaml.safe_load(scen.read_text(encoding="utf-8")).get("policy", "fcfs"))
    run_dir = pathlib.Path(args.run_dir).resolve()
    (run_dir / "logs").mkdir(parents=True, exist_ok=True)

    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from rosgraph_msgs.msg import Clock

    from swarmflow_core.graph import load_layout
    from swarmflow_core.metrics.clearance import footprint_distance

    from ._light_ros import RobotNav

    graph = load_layout(layout_dir)
    spawns = spawn_poses(layout_dir, robots)
    world = LightWorld(max_speed=args.speed, max_turn=args.turn)
    for rid, (x, y, yaw) in spawns.items():
        world.add_robot(rid, x, y, yaw)

    rclpy.init()
    lock = threading.Lock()
    clock_node = Node("light_clock")
    clock_pub = clock_node.create_publisher(Clock, "/clock", 10)
    navs = {rid: RobotNav(rid, world, lock, spawns[rid], graph) for rid in robots}
    ex = MultiThreadedExecutor(num_threads=2 + len(robots))
    ex.add_node(clock_node)
    for n in navs.values():
        ex.add_node(n)
    spin = threading.Thread(target=ex.spin, daemon=True)
    spin.start()

    # unchanged SwarmFlow nodes, one process each, on sim time
    children: List[Tuple[str, subprocess.Popen]] = []
    tf_remap = ["-r", "/tf:=tf", "-r", "/tf_static:=tf_static"]
    specs = [
        ("orchestrator", "swarmflow_orchestrator.orchestrator_node",
         _ros_params({"use_sim_time": True, "layout_dir": layout_dir, "robots": robots, "policy": policy,
                      "run_dir": run_dir})),
        ("scenario_engine", "swarmflow_scenarios.scenario_engine",
         _ros_params({"use_sim_time": True, "scenario_file": scen, "layout_dir": layout_dir, "run_dir": run_dir,
                      "scenarios_tools": ROOT / "tools" / "scenarios"})),
    ]
    for rid in robots:
        specs.append((f"agent_{rid}", "swarmflow_robot_agent.agent_node",
                      ["-r", f"__ns:=/{rid}", *tf_remap,
                       *_ros_params({"use_sim_time": True, "robot_id": rid, "layout_dir": layout_dir,
                                     "traffic_control": policy != "independent"})]))
    for name, module, ros_args in specs:
        log = open(run_dir / "logs" / f"{name}.log", "wb")
        children.append((name, subprocess.Popen(_child_cmd(module, ros_args), stdout=log, stderr=subprocess.STDOUT)))

    def stop_children() -> None:
        for _, p in children:
            if p.poll() is None:
                p.send_signal(signal.SIGINT)
        deadline = time.monotonic() + 10.0
        for _, p in children:
            try:
                p.wait(timeout=max(0.1, deadline - time.monotonic()))
            except subprocess.TimeoutExpired:
                p.terminate()
                try:
                    p.wait(timeout=3.0)
                except subprocess.TimeoutExpired:
                    p.kill()

    code = 0
    trace = open(args.trace, "w", encoding="utf-8") if args.trace else None
    min_sep = float('inf')                    # true minimum footprint distance seen in the world (diagnostic)
    sim_t, t0_wall, wall, self_cpu = START_T, time.monotonic(), 0.0, 0.0
    try:
        # wait for every child node to appear on the graph before time starts
        want = {"/orchestrator", "/scenario_engine"} | {f"/{r}/robot_agent" for r in robots}
        t_end = time.monotonic() + args.startup_timeout_s
        while True:
            have = {(ns.rstrip("/") + "/" + n) for n, ns in clock_node.get_node_names_and_namespaces()}
            for name, p in children:
                if p.poll() is not None:
                    raise RuntimeError(f"{name} exited with {p.returncode}; see {run_dir / 'logs' / (name + '.log')}")
            if want <= have:
                break
            if time.monotonic() > t_end:
                raise RuntimeError(f"nodes did not start in {args.startup_timeout_s} s: missing {sorted(want - have)}")
            time.sleep(0.2)
        time.sleep(1.5)                       # let discovery settle

        t0_cpu = time.process_time()
        t0_wall = time.monotonic()
        sim_t, dt = START_T, args.dt
        ref = t0_wall                         # sim time = START_T + (now - ref) * rtf; moved forward after a stall
        while rclpy.ok():
            target = START_T + (time.monotonic() - ref) * args.rtf
            if target - sim_t > MAX_LAG_S:        # the host stalled: slow down instead of jumping, or the agents'
                ref += (target - sim_t - MAX_LAG_S) / args.rtf      # heartbeats would time out in sim time
                target = sim_t + MAX_LAG_S
            n = 0
            while sim_t + dt <= target and n < 200:
                with lock:
                    world.step(dt)
                    ids = world.robot_ids()
                    poses = [world.pose(i) for i in ids]
                if trace is not None and int(sim_t + dt) != int(sim_t):
                    with lock:
                        trace.write(json.dumps({"t": round(sim_t + dt, 2), "robots": {i: world.debug(i) for i in ids}}) + "\n")
                for ia in range(len(poses)):
                    for ib in range(ia + 1, len(poses)):
                        if abs(poses[ia][0] - poses[ib][0]) < 2.0 and abs(poses[ia][1] - poses[ib][1]) < 2.0:
                            min_sep = min(min_sep, footprint_distance(poses[ia], poses[ib]))
                sim_t += dt
                n += 1
                msg = Clock()
                msg.clock.sec, msg.clock.nanosec = int(sim_t), int((sim_t % 1.0) * 1e9)
                clock_pub.publish(msg)
                for nav in navs.values():
                    nav.publish(sim_t)
            if args.duration_s and sim_t - START_T >= args.duration_s:
                break
            for name, p in children:
                if p.poll() is not None:
                    raise RuntimeError(f"{name} exited with {p.returncode}; see {run_dir / 'logs' / (name + '.log')}")
            time.sleep(max(0.0, min(0.05, (sim_t + dt - START_T) / args.rtf - (time.monotonic() - ref))))
        wall = time.monotonic() - t0_wall
        self_cpu = time.process_time() - t0_cpu
    except KeyboardInterrupt:
        wall = time.monotonic() - t0_wall
    except Exception as exc:  # noqa: BLE001
        print(f"sim2d.light: {exc}", file=sys.stderr)
        code = 1
        wall = self_cpu = 0.0
    finally:
        for nav in navs.values():
            nav.stopping = True
        stop_children()
        time.sleep(0.3)
        ex.shutdown(timeout_sec=2.0)
        rclpy.try_shutdown()
    t = os.times()
    kids = t.children_user + t.children_system
    if wall > 0:
        print(f"sim2d.light: sim {sim_t - START_T:.1f} s in {wall:.1f} s wall (rtf {args.rtf:g}); cpu this process "
              f"{self_cpu:.1f} s, children {kids:.1f} s -> {(self_cpu + kids) / wall:.2f} cores average; "
              f"min footprint distance in the world {min_sep:.3f} m")
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(code)          # leftover executor threads must not delay the exit


if __name__ == "__main__":
    raise SystemExit(main())
