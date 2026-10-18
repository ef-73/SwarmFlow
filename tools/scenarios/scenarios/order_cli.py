"""Manual orders from the command line (O-07).

    python3 -m scenarios.order_cli shelf --picks PK_W1_S_01 PK_E2_N_03 --dock DK2 [--priority 2] [--id my_1]
    python3 -m scenarios.order_cli pallet --spot PL_W1 --dock DK1
    python3 -m scenarios.order_cli transport --pickup L1 --dropoff D3
"""

from __future__ import annotations

import argparse
import datetime
import os
import secrets
import sys
import time
from typing import List, Optional, Sequence

from swarmflow_core import api


def _default_layout_dir() -> str:
    return "/ws/layouts/%s/generated" % os.environ.get("SWARMFLOW_LAYOUT", "warehouse_v2")


def _default_id() -> str:
    stamp = datetime.datetime.now(datetime.timezone.utc).strftime("%Y%m%d%H%M%S")
    return "manual_%s_%s" % (stamp, secrets.token_hex(2))


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(prog="scenarios.order_cli", description="Publish one order on /fleet/orders.")
    p.add_argument("--layout-dir", default=_default_layout_dir())
    sub = p.add_subparsers(dest="kind", required=True)
    for name in ("shelf", "pallet", "transport"):
        s = sub.add_parser(name)
        s.add_argument("--priority", type=int, default=0)
        s.add_argument("--id", dest="order_id", default=None)
        if name == "shelf":
            s.add_argument("--picks", nargs="+", required=True)
            s.add_argument("--dock", required=True)
        elif name == "pallet":
            s.add_argument("--spot", required=True)
            s.add_argument("--dock", required=True)
        else:
            s.add_argument("--pickup", required=True)
            s.add_argument("--dropoff", required=True)
            s.add_argument("--payload", default="small")
    args = p.parse_args(argv)
    if args.order_id is None:
        args.order_id = _default_id()
    return args


def _check(graph, name: str, kinds: Sequence[str], what: str) -> None:
    v = graph.vertices.get(name)
    if v is None:
        raise ValueError("unknown vertex %s (%s)" % (name, what))
    if v.kind not in kinds:
        raise ValueError("vertex %s is a %s, not a %s vertex (%s)" % (name, v.kind, "/".join(kinds), what))


def build_order(args: argparse.Namespace, graph, now: float) -> api.OrderSpec:
    if args.kind == "shelf":
        picks = tuple(args.picks)
        if len(set(picks)) != len(picks):
            raise ValueError("pick vertices must be distinct: %s" % ", ".join(picks))
        for n in picks:
            _check(graph, n, ("pick",), "pick point")
        _check(graph, args.dock, ("dock",), "dock")
        return api.OrderSpec(args.order_id, now, None, "", args.dock, "small", args.priority,
                             order_type=api.ORDER_SHELF, pick_vertices=picks)
    if args.kind == "pallet":
        _check(graph, args.spot, ("pallet",), "pallet spot")
        _check(graph, args.dock, ("dock",), "dock")
        return api.OrderSpec(args.order_id, now, None, args.spot, args.dock, "pallet", args.priority,
                             order_type=api.ORDER_PALLET, pallet_id="pallet_%s" % args.spot)
    _check(graph, args.pickup, ("loading",), "loading station")
    _check(graph, args.dropoff, ("delivery",), "delivery station")
    return api.OrderSpec(args.order_id, now, None, args.pickup, args.dropoff, args.payload, args.priority,
                         order_type=api.ORDER_TRANSPORT)


def main(argv: Optional[List[str]] = None) -> int:
    args = parse_args(argv)
    from swarmflow_core.graph import load_layout
    try:
        graph = load_layout(args.layout_dir)
        spec = build_order(args, graph, now=0.0)
    except ValueError as e:
        print("error: %s" % e, file=sys.stderr)
        return 2

    import rclpy
    from builtin_interfaces.msg import Time
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from swarmflow_interfaces.msg import Order

    rclpy.init()
    node = rclpy.create_node("order_cli", parameter_overrides=[Parameter("use_sim_time", value=True)])
    try:
        qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
        pub = node.create_publisher(Order, "/fleet/orders", qos)
        end = time.monotonic() + 5.0
        while pub.get_subscription_count() == 0 and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
        if pub.get_subscription_count() == 0:
            print("warning: no subscriber on /fleet/orders after 5 s; publishing anyway", file=sys.stderr)
        msg = Order()
        msg.order_id = spec.order_id
        msg.release_time = node.get_clock().now().to_msg()
        msg.deadline = Time()
        msg.pickup_vertex = spec.pickup_vertex
        msg.dropoff_vertex = spec.dropoff_vertex
        msg.payload_type = spec.payload_type
        msg.priority = spec.priority
        msg.order_type = api.ORDER_TYPE_CODES[spec.order_type]
        msg.pick_vertices = list(spec.pick_vertices)
        msg.pallet_id = spec.pallet_id
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.2)
        print(spec.order_id)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
