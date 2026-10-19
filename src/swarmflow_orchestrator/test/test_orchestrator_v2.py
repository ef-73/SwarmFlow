"""T030: orchestrator node with the v2 fleet core (policy "assign") on warehouse_v2.
Wall clock, rclpy only, no Gazebo/Nav2. A fake DispatchTask server walks the goal list and reports RobotStates."""

from __future__ import annotations

import json
import pathlib
import threading
import time

import pytest

rclpy = pytest.importorskip("rclpy")
from rclpy.action import ActionServer  # noqa: E402
from rclpy.callback_groups import ReentrantCallbackGroup  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402

from swarmflow_core.graph import load_layout  # noqa: E402
from swarmflow_interfaces.action import DispatchTask  # noqa: E402
from swarmflow_interfaces.msg import DecisionEvent, Order, OrderStatus, RobotState  # noqa: E402
from swarmflow_orchestrator.orchestrator_node import OrchestratorNode  # noqa: E402

LAYOUT = pathlib.Path(__file__).resolve().parents[3] / "layouts" / "warehouse_v2" / "generated"
GRAPH = load_layout(LAYOUT)


def wait(pred, timeout=15.0):
    end = time.time() + timeout
    while time.time() < end:
        if pred():
            return True
        time.sleep(0.05)
    return False


@pytest.fixture(scope="module")
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


def test_v2_shelf_order_end_to_end_and_transport_rejected(ros, tmp_path):
    orch = OrchestratorNode(parameter_overrides=[
        Parameter("layout_dir", value=str(LAYOUT)),
        Parameter("robots", value=["robot_1"]),
        Parameter("policy", value="assign"),
        Parameter("run_dir", value=str(tmp_path)),
    ])
    t = rclpy.create_node("orch_v2_test")
    cb = ReentrantCallbackGroup()
    goals, statuses, decisions = [], [], []
    st = {"task": "", "order": "", "last": "HM1", "mode": RobotState.MODE_IDLE}
    lock = threading.Lock()

    def execute(gh):
        req = gh.request
        goals.append(req)
        with lock:
            st.update(task=req.task_id, order=req.order.order_id, mode=RobotState.MODE_NAVIGATING)
        for v in list(req.route)[:-1]:           # intermediate goals are reported via last_vertex
            time.sleep(0.6)
            with lock:
                st["last"] = v
        time.sleep(0.6)
        with lock:
            st.update(last=req.route[-1], task="", order="", mode=RobotState.MODE_IDLE)
        gh.succeed()
        return DispatchTask.Result(success=True)

    ActionServer(t, DispatchTask, "/robot_1/dispatch_task", execute_callback=execute, callback_group=cb)
    latched = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    t.create_subscription(OrderStatus, "/fleet/order_status", statuses.append, latched)
    t.create_subscription(DecisionEvent, "/fleet/decisions", decisions.append, 100)
    order_pub = t.create_publisher(Order, "/fleet/orders", QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE))
    state_pub = t.create_publisher(RobotState, "/fleet/robot_states", 10)

    def publish_state():
        with lock:
            v = GRAPH.vertices[st["last"]]
            s = RobotState(robot_id="robot_1", x=v.x, y=v.y, yaw=0.0, mode=st["mode"], last_vertex=st["last"],
                           task_id=st["task"], order_id=st["order"])
        s.header.stamp = t.get_clock().now().to_msg()
        s.header.frame_id = "map"
        state_pub.publish(s)

    t.create_timer(0.2, publish_state, callback_group=cb)
    ex = MultiThreadedExecutor(num_threads=6)
    ex.add_node(orch)
    ex.add_node(t)
    threading.Thread(target=ex.spin, daemon=True).start()
    try:
        assert wait(lambda: order_pub.get_subscription_count() > 0, 10.0)
        assert wait(lambda: "robot_1" in orch._snapshots, 10.0)
        picks = ["PK_W1_S_01", "PK_W1_N_02"]
        order_pub.publish(Order(order_id="s1", dropoff_vertex="DK2", payload_type="small",
                                order_type=Order.TYPE_SHELF, pick_vertices=picks))
        order_pub.publish(Order(order_id="t1", pickup_vertex="PK_W1_S_01", dropoff_vertex="DK2",
                                payload_type="small", order_type=Order.TYPE_TRANSPORT))
        assert wait(lambda: goals, 15.0), "no DispatchTask goal"
        g = goals[0]
        assert g.route_kind == DispatchTask.Goal.ROUTE_GOALS
        assert g.order.order_id == "s1" and g.order.order_type == Order.TYPE_SHELF
        assert sorted(g.order.pick_vertices) == sorted(picks)
        # the route ends at DK2's lane entry; the dock is claimed there and a follow-up goes to DK2
        assert sorted(g.route[:-1]) == sorted(picks) and g.route[-1] == "X_1_7"
        assert wait(lambda: any(tuple(x.route) == ("DK2",) for x in goals), 15.0), [tuple(x.route) for x in goals]
        assert wait(lambda: [s.state for s in statuses if s.order_id == "s1"][-1:] == [OrderStatus.STATE_DELIVERED],
                    20.0), [s.state for s in statuses if s.order_id == "s1"]
        seq = [s.state for s in statuses if s.order_id == "s1"]
        assert seq == [OrderStatus.STATE_QUEUED, OrderStatus.STATE_ASSIGNED, OrderStatus.STATE_PICKING_UP,
                       OrderStatus.STATE_IN_TRANSIT, OrderStatus.STATE_DELIVERED]
        bad = [s for s in statuses if s.order_id == "t1"]
        assert bad and bad[-1].state == OrderStatus.STATE_FAILED and "UNSUPPORTED" in bad[-1].failure_reason
        a = [d for d in decisions if d.decision_type == "ASSIGN"]
        assert a and a[0].policy == "assign" and a[0].explanation
        # after delivery the idle robot is sent home with a goal task
        assert wait(lambda: any(x.order.order_id == "" and list(x.route) and x.route[-1].startswith("HM")
                                for x in goals), 15.0)
        assert wait(lambda: (tmp_path / "orders.jsonl").exists(), 5.0)
        recs = [json.loads(x) for x in (tmp_path / "orders.jsonl").read_text().splitlines()]
        assert [r["state"] for r in recs if r["order_id"] == "s1"][-1] == "DELIVERED"
    finally:
        ex.shutdown()
        orch.destroy_node()
        t.destroy_node()
