"""rclpy wrapper around :class:`AgentCore` (design §6.3). The core makes every decision; this node only translates.

Threading: all callbacks live in one ReentrantCallbackGroup and every core call is made under ``self._lock`` (an
RLock), so the node works under a ``MultiThreadedExecutor``. No callback waits on a future; the only blocking wait is
the ``DispatchTask`` execute callback sleeping on a ``threading.Event`` (it owns its own executor thread).
"""

from __future__ import annotations

import math
import threading
from typing import Iterable, Optional

import rclpy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateThroughPoses
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Header
from action_msgs.msg import GoalStatus

from swarmflow_core import api
from swarmflow_core.graph import load_layout
from swarmflow_interfaces.action import DispatchTask
from swarmflow_interfaces.msg import ReservationHeartbeat, ReservationRelease, RobotState
from swarmflow_interfaces.srv import RequestReservation as RequestReservationSrv

from .agent_core import AgentCore, CancelNav, Navigate, Release, RequestReservation, TaskFinished

_QOS10 = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
_RELEASE_CODES = {api.ReleaseReason.EXITED: ReservationRelease.REASON_EXITED,
                  api.ReleaseReason.TASK_CANCELLED: ReservationRelease.REASON_TASK_CANCELLED,
                  api.ReleaseReason.FAULT: ReservationRelease.REASON_FAULT}


def _to_time_msg(t: float):
    from builtin_interfaces.msg import Time
    t = max(0.0, t)
    sec = int(t)
    return Time(sec=sec, nanosec=int((t - sec) * 1e9))


def _yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class AgentNode(Node):
    def __init__(self, **kwargs):
        super().__init__("robot_agent", **kwargs)
        self.declare_parameter("robot_id", "robot_1")
        self.declare_parameter("layout_dir", "")
        self.declare_parameter("traffic_control", True)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("tick_hz", 10.0)
        self.robot_id = self.get_parameter("robot_id").value
        self._map_frame = self.get_parameter("map_frame").value
        self._base_frame = self.get_parameter("base_frame").value
        graph = load_layout(self.get_parameter("layout_dir").value)
        self.core = AgentCore(self.robot_id, graph, traffic_control=bool(self.get_parameter("traffic_control").value))

        self._lock = threading.RLock()
        self._cb = ReentrantCallbackGroup()
        self._nav_seq = 0
        self._nav_goal = None
        self._goal_reserved = False
        self._done = threading.Event()
        self._result: Optional[TaskFinished] = None
        self._last_xy = None
        self._last_pose_t = 0.0
        self._speed = 0.0
        self._waiting_zone_last = ""

        # TF (launch remaps /tf -> tf per robot namespace)
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self, spin_thread=False)

        cb = self._cb
        self._nav_client = ActionClient(self, NavigateThroughPoses, "navigate_through_poses", callback_group=cb)
        self._res_client = self.create_client(RequestReservationSrv, "/fleet/request_reservation",
                                              callback_group=cb)
        self._state_pub = self.create_publisher(RobotState, "/fleet/robot_states", _QOS10)
        self._hb_pub = self.create_publisher(ReservationHeartbeat, "/fleet/reservation_heartbeat", _QOS10)
        self._rel_pub = self.create_publisher(ReservationRelease, "/fleet/reservation_release", _QOS10)
        self.create_subscription(Header, "/fleet/orchestrator_heartbeat", self._on_orch_heartbeat, _QOS10,
                                 callback_group=cb)
        self._server = ActionServer(self, DispatchTask, "dispatch_task", self._execute,
                                    goal_callback=self._on_goal, cancel_callback=lambda _h: CancelResponse.ACCEPT,
                                    callback_group=cb)
        self.create_timer(1.0 / float(self.get_parameter("tick_hz").value), self._on_tick, callback_group=cb)
        self.create_timer(api.ROBOT_STATE_PERIOD_S, self._publish_state, callback_group=cb)
        self.create_timer(api.HEARTBEAT_PERIOD_S, self._publish_heartbeat, callback_group=cb)

    # -- helpers -------------------------------------------------------------------------------------------------
    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _run(self, actions: Iterable[object]) -> None:
        """Execute core actions. Caller holds ``self._lock`` (re-entrant, so nested core calls are fine)."""
        for a in actions:
            if isinstance(a, Navigate):
                self._send_nav(a)
            elif isinstance(a, CancelNav):
                self._cancel_nav()
            elif isinstance(a, RequestReservation):
                self._call_reservation(a.request)
            elif isinstance(a, Release):
                m = ReservationRelease()
                m.header.stamp = self.get_clock().now().to_msg()
                m.robot_id, m.lease_id, m.reason = self.robot_id, a.lease_id, _RELEASE_CODES[a.reason]
                self._rel_pub.publish(m)
            elif isinstance(a, TaskFinished):
                self._result = a
                self._done.set()

    # -- Nav2 ----------------------------------------------------------------------------------------------------
    def _send_nav(self, nav: Navigate) -> None:
        self._nav_seq += 1
        seq = self._nav_seq
        if not self._nav_client.server_is_ready():
            self.get_logger().warn("navigate_through_poses server not available")
            self._run(self.core.on_nav_result(False, self._now()))
            return
        goal = NavigateThroughPoses.Goal()
        for x, y, yaw in nav.poses:
            p = PoseStamped()
            p.header.frame_id = self._map_frame
            p.pose.position.x, p.pose.position.y = x, y
            p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
            goal.poses.append(p)
        fut = self._nav_client.send_goal_async(goal)
        fut.add_done_callback(lambda f, s=seq: self._on_nav_accepted(f, s))

    def _on_nav_accepted(self, fut, seq: int) -> None:
        with self._lock:
            handle = fut.result()
            if seq != self._nav_seq:
                if handle is not None and handle.accepted:
                    handle.cancel_goal_async()
                return
            if handle is None or not handle.accepted:
                self._run(self.core.on_nav_result(False, self._now()))
                return
            self._nav_goal = handle
            handle.get_result_async().add_done_callback(lambda f, s=seq: self._on_nav_result(f, s))

    def _on_nav_result(self, fut, seq: int) -> None:
        with self._lock:
            if seq != self._nav_seq:
                return
            self._nav_goal = None
            res = fut.result()
            ok = res is not None and res.status == GoalStatus.STATUS_SUCCEEDED
            self._run(self.core.on_nav_result(ok, self._now()))

    def _cancel_nav(self) -> None:
        self._nav_seq += 1                              # invalidate callbacks of the in-flight goal
        if self._nav_goal is not None:
            self._nav_goal.cancel_goal_async()
            self._nav_goal = None

    # -- reservation service -------------------------------------------------------------------------------------
    def _call_reservation(self, r: api.ReservationRequest) -> None:
        if not self._res_client.service_is_ready():
            self._run(self.core.on_reservation_response(r.request_id, None, self._now()))
            return
        req = RequestReservationSrv.Request()
        req.request_id, req.robot_id, req.zone_id = r.request_id, r.robot_id, r.zone_id
        req.entry_vertex, req.exit_vertex = r.entry_vertex, r.exit_vertex
        req.direction = api.DIRECTION_CODES[r.direction]
        req.earliest_entry = _to_time_msg(r.earliest_entry_t)
        req.expected_exit = _to_time_msg(r.expected_exit_t)
        req.priority = r.priority
        fut = self._res_client.call_async(req)
        fut.add_done_callback(lambda f, rid=r.request_id: self._on_reservation_done(f, rid))

    def _on_reservation_done(self, fut, rid: str) -> None:
        with self._lock:
            try:
                res = fut.result()
            except Exception as exc:  # noqa: BLE001 - any service failure counts as "no answer"
                self.get_logger().warn(f"reservation call failed: {exc}")
                res = None
            if res is None:
                dec = None
            else:
                granted = res.result == RequestReservationSrv.Response.RESULT_GRANTED
                dec = api.ReservationDecision(
                    granted, lease_id=res.lease_id if granted else "",
                    lease_expiry_t=res.lease_expiry.sec + res.lease_expiry.nanosec * 1e-9 if granted else 0.0,
                    reason=res.reason, retry_after_s=res.retry_after_s)
            self._run(self.core.on_reservation_response(rid, dec, self._now()))

    # -- subscriptions / timers ----------------------------------------------------------------------------------
    def _on_orch_heartbeat(self, _msg: Header) -> None:
        with self._lock:
            self._run(self.core.on_orchestrator_heartbeat(self._now()))

    def _on_tick(self) -> None:
        with self._lock:
            t = self._now()
            try:
                tf = self._tf_buffer.lookup_transform(self._map_frame, self._base_frame, rclpy.time.Time())
            except tf2_ros.TransformException:
                tf = None
            if tf is not None:
                x, y = tf.transform.translation.x, tf.transform.translation.y
                if self._last_xy is not None and t > self._last_pose_t:
                    self._speed = math.hypot(x - self._last_xy[0], y - self._last_xy[1]) / (t - self._last_pose_t)
                self._last_xy, self._last_pose_t = (x, y), t
                self._run(self.core.on_pose(x, y, _yaw_of(tf.transform.rotation), self._speed, t))
            self._run(self.core.tick(t))

    def _publish_state(self) -> None:
        with self._lock:
            s = self.core.state()
            m = RobotState()
            m.header.stamp = self.get_clock().now().to_msg()
            m.header.frame_id = "map"
            m.robot_id = s.robot_id
            m.x, m.y, m.yaw, m.linear_speed = s.x, s.y, s.yaw, s.linear_speed
            m.mode = api.MODE_CODES[s.mode]
            m.fault_reason, m.task_id, m.order_id = s.fault_reason, s.task_id, s.order_id
            m.last_vertex, m.next_vertex = s.last_vertex, s.next_vertex
            m.remaining_route, m.held_lease_ids = list(s.remaining_route), list(s.held_lease_ids)
            self._state_pub.publish(m)

    def _publish_heartbeat(self) -> None:
        with self._lock:
            m = ReservationHeartbeat()
            m.header.stamp = self.get_clock().now().to_msg()
            m.robot_id = self.robot_id
            m.lease_ids = self.core.heartbeat_lease_ids()
            self._hb_pub.publish(m)

    # -- DispatchTask action -------------------------------------------------------------------------------------
    def _on_goal(self, goal_request) -> GoalResponse:
        with self._lock:
            if self._goal_reserved or not self.core.can_accept():
                return GoalResponse.REJECT
            self._goal_reserved = True
            return GoalResponse.ACCEPT

    def _execute(self, goal_handle):
        goal = goal_handle.request
        result = DispatchTask.Result()
        with self._lock:
            self._done.clear()
            self._result = None
            try:
                self._run(self.core.start_task(goal.task_id, goal.order.order_id, list(goal.route), self._now(),
                                               pickup=goal.order.pickup_vertex, dropoff=goal.order.dropoff_vertex))
            except (ValueError, RuntimeError) as exc:
                self._goal_reserved = False
                goal_handle.abort()
                result.success, result.failure_reason = False, f"INVALID_TASK: {exc}"
                return result
        cancelled = False
        while not self._done.wait(0.2) and rclpy.ok():
            if goal_handle.is_cancel_requested and not cancelled:
                cancelled = True
                with self._lock:
                    self._run(self.core.cancel(self._now()))
                continue
            with self._lock:
                s = self.core.state()
                fb = DispatchTask.Feedback()
                fb.mode = api.MODE_CODES[s.mode]
                fb.current_vertex = s.last_vertex
                fb.waiting_for_zone = self.core.waiting_zone()
            goal_handle.publish_feedback(fb)
        with self._lock:
            fin = self._result
            self._goal_reserved = False
        if fin is None:                                   # shutdown while running
            goal_handle.abort()
            result.success, result.failure_reason = False, "SHUTDOWN"
            return result
        result.success, result.failure_reason = fin.success, fin.failure_reason
        result.completion_time = self.get_clock().now().to_msg()
        if cancelled:
            goal_handle.canceled()
        elif fin.success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AgentNode()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
