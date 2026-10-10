"""rclpy adapter around :class:`swarmflow_core.fleet.FleetCore` and the FCFS reservation authority
(design §6.5, §6.6, §10, §13.5). The library makes every decision; this node translates ROS <-> library types.

Threading: all callbacks share one ReentrantCallbackGroup and every library / bookkeeping access is made under
``self._lock`` (an RLock), so the node works under a ``MultiThreadedExecutor``. No callback waits on a future.
Time: the node clock (follows ``use_sim_time``) in float seconds.
"""

from __future__ import annotations

import dataclasses
import enum
import json
import os
import threading
import time
from typing import Dict, List, Optional, Tuple

import rclpy
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Time
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Header

from swarmflow_core import api, decisions
from swarmflow_core.api import (Direction, OrderSpec, ReleaseReason, ReservationRequest, RobotMode, RobotSnapshot)
from swarmflow_core.fleet import FleetCore
from swarmflow_core.graph import load_layout
from swarmflow_core.policies.fcfs import FcfsPolicy
from swarmflow_core.reservations import FcfsReservationAuthority
from swarmflow_interfaces.action import DispatchTask
from swarmflow_interfaces.msg import (DecisionEvent, Order, OrderStatus, ReservationHeartbeat, ReservationRelease,
                                      RobotState, ZoneReservation)
from swarmflow_interfaces.srv import ClearZone, RequestReservation

REJECTED = "REJECTED"
CANCEL_HOLD_S = 5.0               #: hold dispatches to a robot this long while its cancelled goal finishes
AGENT_LOST_S = 10.0               #: dispatched task, no RobotState from its robot this long -> AGENT_LOST
RECOVERY_WARN_S = 5.0
CLOCK_JUMP_S = 5.0                #: node clock going back by more than this = simulator restarted
UNKNOWN_POSE_REASONS = frozenset({"NO_POSE", "STALE_POSE"})  #: FAULT reasons whose x/y must not reach the authority
DISPATCH_SERVER_TIMEOUT_S = 10.0  #: action server of a robot not up this long after dispatch -> REJECTED

_RELIABLE10 = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
_LATCHED100 = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
_RELIABLE100 = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)

_MODE_OF_CODE = {code: mode for mode, code in api.MODE_CODES.items()}
_DIRECTION_OF_CODE = {code: d for d, code in api.DIRECTION_CODES.items()}
_REASON_OF_CODE = {code: r for r, code in api.RELEASE_REASON_CODES.items()}


def to_time_msg(t: float) -> Time:
    t = max(0.0, t)
    sec = int(t)
    return Time(sec=sec, nanosec=int((t - sec) * 1e9))


def time_msg_to_s(m) -> float:
    return m.sec + m.nanosec * 1e-9


def _json_default(o):
    if isinstance(o, enum.Enum):
        return o.value
    raise TypeError(f"not serialisable: {type(o)}")


class RunRecord:
    """JSON-lines run record (design §13.5). A non-empty earlier file is moved to ``<name>.prev`` when opened, so a
    reused run id never mixes two runs and never destroys the earlier one. ``run_dir == ""`` disables all files."""

    FILES = ("decisions", "orders", "robot_states", "reservations")

    def __init__(self, run_dir: str):
        self._files: Dict[str, object] = {}
        if run_dir:
            os.makedirs(run_dir, exist_ok=True)
            for name in self.FILES:
                path = os.path.join(run_dir, name + ".jsonl")
                if os.path.exists(path) and os.path.getsize(path) > 0:
                    os.replace(path, path + ".prev")   # keep the earlier run (replacing an older .prev)
                self._files[name] = open(path, "w", encoding="utf-8", newline="\n")

    def write(self, name: str, obj: dict) -> None:
        f = self._files.get(name)
        if f is not None:
            f.write(json.dumps(obj, sort_keys=True, default=_json_default) + "\n")
            f.flush()

    def close(self) -> None:
        for f in self._files.values():
            f.close()
        self._files.clear()


class OrchestratorNode(Node):
    def __init__(self, **kwargs):
        super().__init__("orchestrator", **kwargs)
        self.declare_parameter("layout_dir", "")
        self.declare_parameter("robots", ["robot_1", "robot_2", "robot_3"])
        self.declare_parameter("policy", api.POLICY_FCFS)
        self.declare_parameter("run_dir", "")
        self.declare_parameter("tick_hz", 5.0)
        self.declare_parameter("state_log_hz", 2.0)
        self.robots: List[str] = list(self.get_parameter("robots").value)
        self.policy_name: str = self.get_parameter("policy").value
        if self.policy_name not in (api.POLICY_FCFS, api.POLICY_INDEPENDENT):
            raise ValueError(f"policy must be 'fcfs' or 'independent', got {self.policy_name!r}")
        traffic_control = self.policy_name != api.POLICY_INDEPENDENT

        self.graph = load_layout(self.get_parameter("layout_dir").value)
        self.authority = FcfsReservationAuthority(self.graph)
        self.core = FleetCore(self.graph, FcfsPolicy(name=self.policy_name), self.authority,
                              traffic_control=traffic_control)
        self.record = RunRecord(self.get_parameter("run_dir").value)

        self._lock = threading.RLock()
        self._cb = ReentrantCallbackGroup()
        self._specs: Dict[str, OrderSpec] = {}
        self._snapshots: Dict[str, RobotSnapshot] = {}
        self._dispatch_clients: Dict[str, ActionClient] = {}
        self._pending: List[Tuple[api.Assignment, float]] = []     # dispatches waiting for the action server
        self._inflight: Dict[str, object] = {}                     # task_id -> goal future, goal not yet answered
        self._goal_handles: Dict[str, object] = {}                      # task_id -> accepted ClientGoalHandle
        self._cancel_requested = set()

        self._active: Dict[str, Tuple[str, float]] = {}            # task_id -> (robot_id, dispatch t)
        self._cancelling: Dict[str, Tuple[str, float]] = {}        # robot_id -> (cancelled task_id, since t)
        self._state_stamp: Dict[str, float] = {}                   # robot_id -> newest RobotState header stamp
        self._last_seen: Dict[str, float] = {}                     # robot_id -> node time of last RobotState
        self._valid_snapshots: Dict[str, RobotSnapshot] = {}       # latest snapshot with a valid pose, per robot
        self._reported = set()                                     # robots reported since the recovery window start
        self._window_start: Optional[float] = None
        self._last_warn_t = 0.0
        self._recovered = False
        self._last_clock_t: Optional[float] = None
        # Epoch is an identity, not a measurement: wall clock (not the possibly-frozen sim clock) guarantees that a
        # restarted orchestrator never reuses an epoch, even if sim time restarts at the same value.
        self.epoch = f"epoch:{time.time_ns()}-{os.getpid()}"

        cb = self._cb
        for rid in self.robots:
            self._dispatch_clients[rid] = ActionClient(self, DispatchTask, f"/{rid}/dispatch_task", callback_group=cb)
        self._hb_pub = self.create_publisher(Header, "/fleet/orchestrator_heartbeat", _RELIABLE10)
        self._status_pub = self.create_publisher(OrderStatus, "/fleet/order_status", _LATCHED100)
        self._res_pub = self.create_publisher(ZoneReservation, "/fleet/reservations", _LATCHED100)
        self._dec_pub = self.create_publisher(DecisionEvent, "/fleet/decisions", _RELIABLE100)
        self.create_subscription(Order, "/fleet/orders", self._on_order, _RELIABLE100, callback_group=cb)
        self.create_subscription(RobotState, "/fleet/robot_states", self._on_robot_state, _RELIABLE10,
                                 callback_group=cb)
        self.create_subscription(ReservationHeartbeat, "/fleet/reservation_heartbeat", self._on_heartbeat,
                                 _RELIABLE10, callback_group=cb)
        self.create_subscription(ReservationRelease, "/fleet/reservation_release", self._on_release, _RELIABLE10,
                                 callback_group=cb)
        self.create_service(RequestReservation, "/fleet/request_reservation", self._on_request, callback_group=cb)
        self.create_service(ClearZone, "/fleet/clear_zone", self._on_clear_zone, callback_group=cb)
        self.create_timer(1.0 / float(self.get_parameter("tick_hz").value), self._on_tick, callback_group=cb)
        self.create_timer(1.0 / float(self.get_parameter("state_log_hz").value), self._log_states, callback_group=cb)
        self.create_timer(1.0, self._publish_heartbeat, callback_group=cb)

    # -- helpers -------------------------------------------------------------------------------------------------
    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _publish_heartbeat(self) -> None:
        m = Header()
        m.stamp = self.get_clock().now().to_msg()
        m.frame_id = self.epoch
        self._hb_pub.publish(m)

    def _publish_decision(self, d: api.Decision) -> None:
        m = DecisionEvent()
        m.header.stamp = to_time_msg(d.t)
        m.header.frame_id = "map"
        m.event_id, m.policy, m.decision_type = d.event_id, d.policy, d.decision_type.value
        m.robot_id, m.order_id, m.trigger = d.robot_id, d.order_id, d.trigger
        m.previous_decision, m.new_decision = d.previous_decision, d.new_decision
        m.cost_keys, m.cost_values = list(d.cost_keys), [float(v) for v in d.cost_values]
        m.predicted_improvement_s = float(d.predicted_improvement_s)
        m.explanation = d.explanation or decisions.render(d)
        self._dec_pub.publish(m)
        rec = dataclasses.asdict(d)
        rec["explanation"] = m.explanation
        self.record.write("decisions", rec)

    def _publish_status(self, c: api.OrderStatusChange) -> None:
        m = OrderStatus()
        m.header.stamp = to_time_msg(c.t)
        m.header.frame_id = "map"
        m.order_id, m.state = c.order_id, api.ORDER_STATE_CODES[c.state]
        m.robot_id, m.failure_reason = c.robot_id, c.failure_reason
        self._status_pub.publish(m)
        self.record.write("orders", {"t": c.t, "order_id": c.order_id, "state": c.state.value,
                                     "robot_id": c.robot_id, "failure_reason": c.failure_reason})

    def _flush_leases(self) -> None:
        """Publish and record every pending lease state change. Caller holds the lock."""
        for lease in self.authority.pop_events():
            m = ZoneReservation()
            m.header.stamp = self.get_clock().now().to_msg()
            m.header.frame_id = "map"
            m.lease_id, m.robot_id, m.zone_id = lease.lease_id, lease.robot_id, lease.zone_id
            m.state = api.LEASE_STATE_CODES[lease.state]
            if lease.state == api.LeaseState.GRANTED:
                m.lease_expiry = to_time_msg(lease.expiry_t)
            m.reason = lease.reason
            self._res_pub.publish(m)
            self.record.write("reservations", {
                "t": time_msg_to_s(m.header.stamp), "lease_id": lease.lease_id, "robot_id": lease.robot_id,
                "zone_id": lease.zone_id, "state": lease.state.value, "granted_t": lease.granted_t,
                "expiry_t": lease.expiry_t, "hard_expiry_t": lease.hard_expiry_t, "reason": lease.reason})

    # -- subscriptions -------------------------------------------------------------------------------------------
    def _on_order(self, msg: Order) -> None:
        now = self._now()
        release = time_msg_to_s(msg.release_time)
        deadline = time_msg_to_s(msg.deadline)
        spec = OrderSpec(order_id=msg.order_id, release_t=release if release > 0.0 else now,
                         deadline_t=deadline if deadline > 0.0 else None, pickup_vertex=msg.pickup_vertex,
                         dropoff_vertex=msg.dropoff_vertex, payload_type=msg.payload_type or "small",
                         priority=int(msg.priority))
        with self._lock:
            if spec.order_id in self._specs:
                return
            self._specs[spec.order_id] = spec
            self.core.add_order(spec)

    def _on_robot_state(self, msg: RobotState) -> None:
        snap = RobotSnapshot(
            robot_id=msg.robot_id, x=msg.x, y=msg.y, yaw=msg.yaw, mode=_MODE_OF_CODE.get(msg.mode, RobotMode.FAULT),
            last_vertex=msg.last_vertex, task_id=msg.task_id, order_id=msg.order_id,
            held_lease_ids=tuple(msg.held_lease_ids), linear_speed=msg.linear_speed, fault_reason=msg.fault_reason,
            next_vertex=msg.next_vertex, remaining_route=tuple(msg.remaining_route))
        stamp = time_msg_to_s(msg.header.stamp)
        with self._lock:
            now = self._now()
            self._check_clock(now)
            if stamp < self._state_stamp.get(msg.robot_id, -1.0) - CLOCK_JUMP_S:
                # this robot's time base went back (agent restart): forget its newest stamp only; a simulator
                # restart is detected by the node clock in _check_clock, which revokes fleet-wide (S7)
                self.get_logger().warn(f"{msg.robot_id}: RobotState stamp went back by more than {CLOCK_JUMP_S} s")
                self._state_stamp.pop(msg.robot_id, None)
            if stamp < self._state_stamp.get(msg.robot_id, -1.0):
                return  # stale / out-of-order sample never overwrites a newer one
            self._state_stamp[msg.robot_id] = stamp
            self._last_seen[msg.robot_id] = self._now()
            unknown_pose = snap.mode == RobotMode.FAULT and snap.fault_reason in UNKNOWN_POSE_REASONS
            if self._window_start is not None and stamp >= self._window_start and not unknown_pose:
                self._reported.add(msg.robot_id)
            self._snapshots[msg.robot_id] = snap
            if not unknown_pose:
                self._valid_snapshots[msg.robot_id] = snap
            self.core.update_robot(snap)
            if not unknown_pose:
                # an unknown / old pose must never clear a blocked zone (S4)
                self.authority.observe_robot(msg.robot_id, msg.x, msg.y, stamp)
            self._flush_leases()

    def _on_heartbeat(self, msg: ReservationHeartbeat) -> None:
        with self._lock:
            self._check_clock(self._now())
            self.authority.heartbeat(msg.robot_id, list(msg.lease_ids), self._now())

    def _on_release(self, msg: ReservationRelease) -> None:
        with self._lock:
            self._check_clock(self._now())
            self.authority.release(msg.robot_id, msg.lease_id, self._now(),
                                   _REASON_OF_CODE.get(msg.reason, ReleaseReason.FAULT))
            self._flush_leases()

    # -- services ------------------------------------------------------------------------------------------------
    def _on_request(self, req, res):
        res.result = RequestReservation.Response.RESULT_DENIED
        if self.policy_name == api.POLICY_INDEPENDENT:
            res.reason = "TRAFFIC_CONTROL_OFF"
            return res
        now = self._now()
        r = ReservationRequest(
            request_id=req.request_id, robot_id=req.robot_id, zone_id=req.zone_id, entry_vertex=req.entry_vertex,
            exit_vertex=req.exit_vertex, direction=_DIRECTION_OF_CODE.get(req.direction, Direction.UNSPECIFIED),
            earliest_entry_t=time_msg_to_s(req.earliest_entry), expected_exit_t=time_msg_to_s(req.expected_exit),
            priority=int(req.priority))
        with self._lock:
            self._check_clock(now)
            dec = self.authority.request(r, now)
            self._publish_decision(decisions.reservation_decision(now, self.policy_name, r, dec))
            self._flush_leases()
        if dec.granted:
            res.result = RequestReservation.Response.RESULT_GRANTED
            res.lease_id = dec.lease_id
            res.lease_expiry = to_time_msg(dec.lease_expiry_t)
        else:
            res.reason = dec.reason
        res.retry_after_s = float(dec.retry_after_s)
        return res

    def _on_clear_zone(self, req, res):
        with self._lock:
            res.success = bool(self.authority.clear_zone(req.zone_id, req.reason, self._now()))
            self._flush_leases()
        res.message = "" if res.success else f"unknown zone {req.zone_id}"
        return res

    # -- tick ----------------------------------------------------------------------------------------------------
    def _on_tick(self) -> None:
        with self._lock:
            t = self._now()
            self._check_clock(t)
            self._check_recovery(t)
            self.authority.expire(t)
            out = self.core.tick(t)
            for task_id in out.cancels:
                self._cancel_task(task_id, t)
            for a in out.dispatches:
                self._pending.append((a, t))
                self._active[a.task_id] = (a.robot_id, t)
            for c in out.status_changes:
                self._publish_status(c)
            for d in out.decisions:
                self._publish_decision(d)
            self._flush_leases()
            self._send_pending(t)
            self._watchdog(t)

    def _check_clock(self, t: float) -> None:
        """Detect a clock reset (Gazebo restarted under a running orchestrator, S7): forget the per-robot stamps,
        restart the recovery window with a fresh authority. Orders and tasks stay. Caller holds the lock."""
        last, self._last_clock_t = self._last_clock_t, t
        if last is None or t >= last - CLOCK_JUMP_S:
            return
        self._reset_clock_state(t, last)

    def _reset_clock_state(self, t: float, t_old: float) -> None:
        """Caller holds the lock. ``t_old`` is the last time the old authority saw."""
        self.get_logger().warn(f"node clock went backwards ({t_old:.1f}s -> {t:.1f}s): resetting recovery state")
        old = self.authority
        for zone in sorted(self.graph.zones):      # robots must drop every lease of the old time base
            old.clear_zone(zone, "CLOCK_RESET", t_old)
        self._flush_leases()
        self._state_stamp.clear()
        self._last_seen.clear()
        self._reported.clear()
        self._window_start = None
        self._last_warn_t = t
        self._recovered = False
        self._last_clock_t = t
        self.authority = FcfsReservationAuthority(self.graph)
        self.core.authority = self.authority
        self._active = {tid: (rid, t) for tid, (rid, _t0) in self._active.items()}
        self._pending = [(a, t) for a, _t0 in self._pending]
        self._cancelling = {rid: (tid, t) for rid, (tid, _since) in self._cancelling.items()}

    def _check_recovery(self, t: float) -> None:
        """Recovery window starts at the first tick with a running clock (sim clock is 0 until /clock arrives);
        recover once every configured robot has reported since then and RECOVERY_GRACE_S has passed. Until then the
        authority denies RECOVERING. Caller holds the lock."""
        if self._recovered or t <= 0.0:
            return
        if self._window_start is None:
            self._window_start = t
            self._last_warn_t = t
            return
        missing = [r for r in self.robots if r not in self._reported or r not in self._valid_snapshots
                   or self._snapshots[r] is not self._valid_snapshots[r]]
        if not missing and t - self._window_start >= api.RECOVERY_GRACE_S:
            self.authority.recover(list(self._valid_snapshots.values()), t)
            self._recovered = True
        elif missing and t - self._last_warn_t >= RECOVERY_WARN_S:
            self._last_warn_t = t
            self.get_logger().warn(f"recovering: no RobotState with a valid pose yet from {', '.join(missing)}")

    def _watchdog(self, t: float) -> None:
        for task_id, (rid, t0) in list(self._active.items()):
            if t - max(t0, self._last_seen.get(rid, 0.0)) > AGENT_LOST_S:
                self._finish(task_id, False, "AGENT_LOST", t)

    def _finish(self, task_id: str, ok: bool, reason: str, t: float) -> None:
        """Report a task result to the core exactly once; ignored for tasks no longer active (cancelled)."""
        if self._active.pop(task_id, None) is not None:
            self.core.task_result(task_id, ok, reason, t)

    def _log_states(self) -> None:
        with self._lock:
            t = self._now()
            for rid in sorted(self._snapshots):
                s = self._snapshots[rid]
                self.record.write("robot_states", {
                    "t": t, "robot_id": rid, "x": s.x, "y": s.y, "yaw": s.yaw, "mode": s.mode.value,
                    "linear_speed": s.linear_speed, "task_id": s.task_id, "order_id": s.order_id,
                    "held_lease_ids": list(s.held_lease_ids)})

    # -- DispatchTask --------------------------------------------------------------------------------------------
    def _goal_of(self, a: api.Assignment):
        goal = DispatchTask.Goal()
        goal.task_id, goal.robot_id, goal.route = a.task_id, a.robot_id, list(a.route)
        spec = self._specs.get(a.order_id) if a.order_id else None
        if spec is not None:
            o = goal.order
            o.order_id, o.release_time = spec.order_id, to_time_msg(spec.release_t)
            if spec.deadline_t is not None:
                o.deadline = to_time_msg(spec.deadline_t)
            o.pickup_vertex, o.dropoff_vertex = spec.pickup_vertex, spec.dropoff_vertex
            o.payload_type, o.priority = spec.payload_type, max(0, min(255, spec.priority))
        return goal

    def _send_pending(self, t: float) -> None:
        """Send queued dispatches whose action server is up and whose robot is not still finishing a cancelled goal
        (the agent frees its goal slot only then); time out the rest. Caller holds the lock."""
        for rid, (_tid, since) in list(self._cancelling.items()):
            if t - since >= CANCEL_HOLD_S:
                del self._cancelling[rid]
        still: List[Tuple[api.Assignment, float]] = []
        for a, t0 in self._pending:
            client = self._dispatch_clients.get(a.robot_id)
            if a.robot_id in self._cancelling:
                still.append((a, t0))
            elif client is not None and client.server_is_ready():
                fut = client.send_goal_async(self._goal_of(a))
                self._inflight[a.task_id] = fut
                fut.add_done_callback(lambda f, tid=a.task_id: self._on_goal_response(tid, f))
            elif t - t0 >= DISPATCH_SERVER_TIMEOUT_S:
                self._finish(a.task_id, False, REJECTED, t)
            else:
                still.append((a, t0))
        self._pending = still

    def _release_cancel_hold(self, task_id: str) -> None:
        for rid, (tid, _since) in list(self._cancelling.items()):
            if tid == task_id:
                del self._cancelling[rid]

    def _on_goal_response(self, task_id: str, fut) -> None:
        with self._lock:
            self._inflight.pop(task_id, None)
            try:
                handle = fut.result()
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f"goal for {task_id} failed to send: {exc}")
                handle = None
            if handle is None or not handle.accepted:
                self._cancel_requested.discard(task_id)
                self._release_cancel_hold(task_id)
                self._finish(task_id, False, REJECTED, self._now())
                return
            self._goal_handles[task_id] = handle
            if task_id in self._cancel_requested:
                self._cancel_requested.discard(task_id)
                handle.cancel_goal_async()
            handle.get_result_async().add_done_callback(lambda f, tid=task_id: self._on_goal_result(tid, f))

    def _on_goal_result(self, task_id: str, fut) -> None:
        with self._lock:
            self._goal_handles.pop(task_id, None)
            self._cancel_requested.discard(task_id)
            self._release_cancel_hold(task_id)
            try:
                wrapper = fut.result()
            except Exception as exc:  # noqa: BLE001
                self._finish(task_id, False, f"RESULT_ERROR:{exc}", self._now())
                return
            ok = wrapper.status == GoalStatus.STATUS_SUCCEEDED and bool(wrapper.result.success)
            reason = "" if ok else (wrapper.result.failure_reason or
                                    ("CANCELLED" if wrapper.status == GoalStatus.STATUS_CANCELED else "ABORTED"))
            self._finish(task_id, ok, reason, self._now())

    def _cancel_task(self, task_id: str, t: float) -> None:
        """Cancel a (preempted park) task's goal wherever it is. Caller holds the lock."""
        self._pending = [(a, t0) for a, t0 in self._pending if a.task_id != task_id]
        robot = self._active.pop(task_id, ("", 0.0))[0]
        handle = self._goal_handles.get(task_id)
        if handle is not None:
            handle.cancel_goal_async()
        elif task_id in self._inflight:
            self._cancel_requested.add(task_id)
        else:
            return  # never sent: nothing to wait for
        if robot:
            self._cancelling[robot] = (task_id, t)

    def close(self) -> None:
        self.record.close()

    def destroy_node(self):
        self.close()
        return super().destroy_node()


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = OrchestratorNode()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
