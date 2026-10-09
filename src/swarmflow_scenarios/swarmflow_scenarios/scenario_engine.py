"""scenario_engine: publishes the scenario's orders on /fleet/orders in (sim) time and writes the run record
(design 12.3, 13.5). The order generator lives in ``tools/scenarios`` and is imported via the ``scenarios_tools``
parameter (default /ws/tools/scenarios)."""

from __future__ import annotations

import dataclasses
import datetime
import json
import pathlib
import sys
from typing import List, Optional

import rclpy
import yaml
from builtin_interfaces.msg import Time
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from swarmflow_core.graph import load_layout
from swarmflow_interfaces.msg import Order

_QOS_ORDERS = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE)


def _to_time(seconds: float) -> Time:
    sec = int(seconds)
    return Time(sec=sec, nanosec=int(round((seconds - sec) * 1e9)) % 1_000_000_000)


class ScenarioEngineNode(Node):
    def __init__(self, **kwargs):
        super().__init__("scenario_engine", **kwargs)
        self.declare_parameter("scenario_file", "")
        self.declare_parameter("layout_dir", "")
        self.declare_parameter("run_dir", "")
        self.declare_parameter("git_sha", "")
        self.declare_parameter("image_digests", "")
        self.declare_parameter("start_delay_s", 5.0)
        self.declare_parameter("scenarios_tools", "/ws/tools/scenarios")

        tools = str(self.get_parameter("scenarios_tools").value)
        if tools and tools not in sys.path:
            sys.path.insert(0, tools)
        from scenarios import generate_orders, load_scenario  # noqa: E402  (path set above)

        scenario_file = str(self.get_parameter("scenario_file").value)
        self._scenario = load_scenario(scenario_file)
        graph = load_layout(str(self.get_parameter("layout_dir").value))
        self._orders = generate_orders(self._scenario, graph)
        self._delay = float(self.get_parameter("start_delay_s").value)
        self._git_sha = str(self.get_parameter("git_sha").value)
        self._image_digests = str(self.get_parameter("image_digests").value)
        run_dir = str(self.get_parameter("run_dir").value)
        self._run_dir: Optional[pathlib.Path] = pathlib.Path(run_dir) if run_dir else None

        self._pub = self.create_publisher(Order, "/fleet/orders", _QOS_ORDERS)
        self._start: Optional[float] = None
        self._next = 0
        self._timer = self.create_timer(0.1, self._tick)
        self.get_logger().info(f"scenario '{self._scenario.name}': {len(self._orders)} orders, "
                               f"start delay {self._delay:.1f} s")

    @property
    def published_count(self) -> int:
        return self._next

    @property
    def done(self) -> bool:
        return self._next >= len(self._orders)

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _write_config(self, start_s: float) -> None:
        if self._run_dir is None:
            return
        self._run_dir.mkdir(parents=True, exist_ok=True)
        cfg = {
            "scenario": dataclasses.asdict(self._scenario),
            "git_sha": self._git_sha,
            "image_digests": self._image_digests,
            "start_time_sim_s": start_s,
            "start_time_utc": datetime.datetime.now(datetime.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
            "start_delay_s": self._delay,
            "n_orders": len(self._orders),
        }
        (self._run_dir / "config.yaml").write_text(yaml.safe_dump(cfg, sort_keys=False), encoding="utf-8")

    def _append_event(self, rec: dict) -> None:
        if self._run_dir is None:
            return
        with open(self._run_dir / "events.jsonl", "a", encoding="utf-8", newline="\n") as f:
            f.write(json.dumps(rec) + "\n")

    def _tick(self) -> None:
        now = self._now_s()
        if self._start is None:
            if now <= 0.0:       # sim clock not running yet
                return
            self._start = now
            self._write_config(now)
        elapsed = now - self._start
        while self._next < len(self._orders):
            spec = self._orders[self._next]
            if elapsed < spec.release_t + self._delay:
                break
            msg = Order()
            msg.order_id = spec.order_id
            msg.release_time = self.get_clock().now().to_msg()
            msg.deadline = Time()
            msg.pickup_vertex = spec.pickup_vertex
            msg.dropoff_vertex = spec.dropoff_vertex
            msg.payload_type = spec.payload_type
            msg.priority = spec.priority
            self._pub.publish(msg)
            self._append_event({"t": now, "type": "order_released", "order_id": spec.order_id,
                                "pickup": spec.pickup_vertex, "dropoff": spec.dropoff_vertex,
                                "payload_type": spec.payload_type, "scheduled_t": spec.release_t})
            self._next += 1
        if self.done:
            self._timer.cancel()


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = ScenarioEngineNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
