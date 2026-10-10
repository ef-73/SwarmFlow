"""FCFS zone reservation authority (design §6.5). Pure Python, deterministic, time passed in by the caller.

The lease state machine is specified in the ``api.ReservationAuthority`` docstring. Safety invariant: at most
``capacity`` GRANTED leases per zone, and never a grant while the zone holds a blocking (OCCUPIED_UNKNOWN) lease.

Pose evidence on release: the robot's stored pose counts as evidence only if it is fresh, i.e. its stamp is >= the
lease's ``granted_t`` (GRANTED lease) or ``blocking_since`` (blocking lease) AND >= ``t - POSE_FRESH_S``. Evidence
inside the zone: a GRANTED lease becomes OCCUPIED_UNKNOWN (``RELEASED_INSIDE:<reason>``), a blocking lease stays
blocking. Evidence outside: RELEASED (``<reason>`` / ``LATE_RELEASE``). A pose that exists but is not evidence is
treated as possibly inside (GRANTED becomes OCCUPIED_UNKNOWN, blocking stays). If no pose was ever observed for the
robot the release is trusted. ``observe_robot`` only clears blocking leases using the robot's newest pose; a sample
older than the stored newest stamp never clears anything.

M9 hardening (independent review):

* S2, owner re-grant (liveness): a robot stuck inside a zone owns the blocking lease (e.g. FAULT release with an inside
  pose); its park task requests that zone and would be denied forever. If the zone has no GRANTED lease of another
  robot and every blocking lease there belongs to the requester, those leases end RELEASED/``SUPERSEDED`` and the
  requester gets a fresh GRANTED lease. The owner is the only possible occupant, so exclusivity is preserved. This
  rule runs before the FIFO check, otherwise the queue head (blocked) and the owner (queued) would deadlock.
* S3, unauthorized entry (safety): a newest-stamp pose inside a zone polygon from a robot without a live lease there
  creates a blocking ``unauthorized_<robot>_<zone>_<n>`` lease (``UNAUTHORIZED_ENTRY``), cleared like any blocking
  lease. Never a duplicate for the same robot and zone. Not created before ``recover`` (recovery covers that).
  Its creation is returned by ``observe_robot`` and it is visible in ``active_leases``, but it is kept out of
  ``pop_events``: the lead property test requires every lease's first event to be GRANTED (change request filed).
* S6, FIFO fairness: robots denied for a zone queue in order of first denial (with the time of their latest request).
  A non-head robot is denied ``QUEUED_BEHIND:<head>`` while the head's last request is younger than
  ``QUEUE_STALE_S``; a stale head is dropped. Granting removes the robot from the queue. Invariant "never a new grant
  while another robot blocks the zone" is unaffected (the queue only adds denials).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

from . import api
from .api import (Lease, LeaseState, ReleaseReason, ReservationDecision, ReservationRequest, RobotSnapshot)
from .graph import point_in_polygon


POSE_FRESH_S = 1.0  #: max age of a pose (relative to the release time) to count as release evidence
QUEUE_STALE_S = 2.5  #: a queue head that has not re-requested for this long is dropped (agents retry about every 1 s)


@dataclass
class _Rec:
    """Mutable internal lease record; ``snapshot()`` produces the public frozen :class:`Lease`."""

    lease_id: str
    robot_id: str
    zone_id: str
    state: LeaseState
    granted_t: float
    expiry_t: float
    hard_expiry_t: float
    reason: str = ""
    blocking_since: float = 0.0
    quiet: bool = False  #: unauthorized-entry lease: returned by observe_robot, not queued in pop_events (see docstring)

    def snapshot(self) -> Lease:
        return Lease(self.lease_id, self.robot_id, self.zone_id, self.state, self.granted_t, self.expiry_t,
                     self.hard_expiry_t, self.reason)

    @property
    def blocking(self) -> bool:
        return self.state == LeaseState.OCCUPIED_UNKNOWN


class FcfsReservationAuthority:
    def __init__(self, graph, lease_ttl_s: float = api.LEASE_TTL_S, retry_after_s: float = api.RETRY_AFTER_S):
        self._graph = graph
        self._ttl = lease_ttl_s
        self._retry = retry_after_s
        self._recovering = True
        self._seq = 0
        self._live: Dict[str, _Rec] = {}                       # GRANTED + blocking leases by id
        self._poses: Dict[str, Tuple[float, float, float]] = {}  # robot -> (x, y, pose stamp)
        self._events: List[Lease] = []
        self._queues: Dict[str, Dict[str, float]] = {}           # zone -> robot -> latest request t (insertion order)
        self._unauth_seq = 0

    # ---- internals -------------------------------------------------------------------------------------------

    def _set(self, rec: _Rec, state: LeaseState, reason: str, t: float) -> None:
        rec.state = state
        rec.reason = reason
        if state == LeaseState.OCCUPIED_UNKNOWN:
            rec.blocking_since = t
        else:
            self._live.pop(rec.lease_id, None)
        if not rec.quiet:
            self._events.append(rec.snapshot())

    def _expire(self, t: float) -> List[Lease]:
        changed: List[Lease] = []
        for rec in sorted(self._live.values(), key=lambda r: r.lease_id):
            if rec.state == LeaseState.GRANTED and t >= rec.expiry_t:
                self._set(rec, LeaseState.OCCUPIED_UNKNOWN, "EXPIRED", rec.expiry_t)
                changed.append(rec.snapshot())
        return changed

    def _inside(self, rec: _Rec, t: float) -> bool:
        """Is the lease owner possibly inside the zone at release time ``t``? A fresh pose is evidence; a stale pose
        is treated as possibly inside; no pose ever observed = trusted outside."""
        pose = self._poses.get(rec.robot_id)
        if pose is None:
            return False
        since = rec.granted_t if rec.state == LeaseState.GRANTED else rec.blocking_since
        if pose[2] < since or pose[2] < t - POSE_FRESH_S:
            return True
        return point_in_polygon(pose[0], pose[1], self._graph.zones[rec.zone_id].polygon)

    def _zone_recs(self, zone_id: str) -> List[_Rec]:
        return [r for r in self._live.values() if r.zone_id == zone_id]

    # ---- ReservationAuthority --------------------------------------------------------------------------------

    def recover(self, robots: Sequence[RobotSnapshot], t: float) -> Sequence[Lease]:
        self._expire(t)
        created: List[Lease] = []
        for robot in sorted(robots, key=lambda r: r.robot_id):
            self._poses[robot.robot_id] = (robot.x, robot.y, t)
            for name in sorted(self._graph.zones):
                if not point_in_polygon(robot.x, robot.y, self._graph.zones[name].polygon):
                    continue
                lease_id = f"recovered_{robot.robot_id}_{name}"
                if lease_id in self._live:
                    continue
                rec = _Rec(lease_id, robot.robot_id, name, LeaseState.OCCUPIED_UNKNOWN, t, t, t,
                           "RECOVERED", blocking_since=t)
                self._live[lease_id] = rec
                self._events.append(rec.snapshot())
                created.append(rec.snapshot())
        self._recovering = False
        return created

    def request(self, req: ReservationRequest, t: float) -> ReservationDecision:
        self._expire(t)

        def deny(reason: str) -> ReservationDecision:
            return ReservationDecision(False, "", 0.0, reason, self._retry)

        if self._recovering:
            return deny(api.DENY_RECOVERING)
        zone = self._graph.zones.get(req.zone_id)
        if zone is None:
            return deny(api.DENY_UNKNOWN_ZONE)
        if req.entry_vertex not in {e.entry for e in zone.entries}:
            return deny(api.DENY_BAD_ENTRY)
        recs = self._zone_recs(req.zone_id)
        for rec in sorted(recs, key=lambda r: r.lease_id):
            if rec.state == LeaseState.GRANTED and rec.robot_id == req.robot_id:
                rec.expiry_t = min(t + self._ttl, rec.hard_expiry_t)
                return ReservationDecision(True, rec.lease_id, rec.expiry_t)
        queue = self._queues.setdefault(req.zone_id, {})
        blocking = [r for r in recs if r.blocking]
        if (blocking and all(r.robot_id == req.robot_id for r in blocking)
                and not any(r.state == LeaseState.GRANTED for r in recs)):
            for rec in sorted(blocking, key=lambda r: r.lease_id):   # S2: owner re-grant
                self._set(rec, LeaseState.RELEASED, "SUPERSEDED", t)
            recs = self._zone_recs(req.zone_id)
            blocking = []
        else:
            while queue:                                              # S6: drop stale heads, honour the live one
                head = next(iter(queue))
                if head != req.robot_id and t - queue[head] >= QUEUE_STALE_S:
                    del queue[head]
                else:
                    break
            head = next(iter(queue), None)
            if head is not None and head != req.robot_id:
                queue[req.robot_id] = t
                return deny(f"QUEUED_BEHIND:{head}")
            if blocking:
                queue[req.robot_id] = t
                return deny(api.DENY_OCCUPIED_UNKNOWN)
            if sum(1 for r in recs if r.state == LeaseState.GRANTED) >= zone.capacity:
                queue[req.robot_id] = t
                return deny(api.DENY_ZONE_LEASED)
        queue.pop(req.robot_id, None)
        self._seq += 1
        hard = api.lease_hard_expiry(max(req.earliest_entry_t, t), max(req.expected_exit_t, t))
        rec = _Rec(f"lease_{self._seq:05d}", req.robot_id, req.zone_id, LeaseState.GRANTED, t,
                   min(t + self._ttl, hard), hard)
        self._live[rec.lease_id] = rec
        self._events.append(rec.snapshot())
        return ReservationDecision(True, rec.lease_id, rec.expiry_t)

    def heartbeat(self, robot_id: str, lease_ids: Sequence[str], t: float) -> None:
        # Renews only live (not yet expired) GRANTED leases of the caller. The expiry transition itself is left to
        # the next expire()/request()/... so that its event is reported there.
        for lease_id in lease_ids:
            rec = self._live.get(lease_id)
            if rec and rec.robot_id == robot_id and rec.state == LeaseState.GRANTED and t < rec.expiry_t:
                rec.expiry_t = min(t + self._ttl, rec.hard_expiry_t)

    def release(self, robot_id: str, lease_id: str, t: float,
                reason: ReleaseReason = ReleaseReason.EXITED) -> None:
        self._expire(t)
        rec = self._live.get(lease_id)
        if rec is None or rec.robot_id != robot_id:
            return
        inside = self._inside(rec, t)
        if rec.state == LeaseState.GRANTED:
            if inside:
                self._set(rec, LeaseState.OCCUPIED_UNKNOWN, f"RELEASED_INSIDE:{reason.value}", t)
            else:
                self._set(rec, LeaseState.RELEASED, reason.value, t)
        elif not inside:  # blocking lease, late release while the robot is not (known to be) inside
            self._set(rec, LeaseState.RELEASED, "LATE_RELEASE", t)

    def expire(self, t: float) -> Sequence[Lease]:
        return self._expire(t)

    def observe_robot(self, robot_id: str, x: float, y: float, t: float) -> Sequence[Lease]:
        changed = self._expire(t)
        known = self._poses.get(robot_id)
        if known is not None and t < known[2]:
            return changed  # out-of-order sample: never overwrites the newest pose, never clears
        self._poses[robot_id] = (x, y, t)
        for rec in sorted(self._live.values(), key=lambda r: r.lease_id):
            if (rec.blocking and rec.robot_id == robot_id and t >= rec.blocking_since
                    and not point_in_polygon(x, y, self._graph.zones[rec.zone_id].polygon)):
                self._set(rec, LeaseState.RELEASED, "OBSERVED_OUTSIDE", t)
                changed.append(rec.snapshot())
        if not self._recovering:                                      # S3: unauthorized entry
            for name in sorted(self._graph.zones):
                if not point_in_polygon(x, y, self._graph.zones[name].polygon):
                    continue
                if any(r.robot_id == robot_id for r in self._zone_recs(name)):
                    continue
                self._unauth_seq += 1
                rec = _Rec(f"unauthorized_{robot_id}_{name}_{self._unauth_seq}", robot_id, name,
                           LeaseState.OCCUPIED_UNKNOWN, t, t, t, "UNAUTHORIZED_ENTRY", blocking_since=t)
                rec.quiet = True
                self._live[rec.lease_id] = rec
                changed.append(rec.snapshot())
        return changed

    def clear_zone(self, zone_id: str, reason: str, t: float) -> bool:
        if zone_id not in self._graph.zones:
            return False
        self._expire(t)
        for rec in sorted(self._zone_recs(zone_id), key=lambda r: r.lease_id):
            self._set(rec, LeaseState.REVOKED, f"CLEAR_ZONE:{reason}", t)
        return True

    def active_leases(self) -> Sequence[Lease]:
        return [r.snapshot() for r in sorted(self._live.values(), key=lambda r: r.lease_id)]

    def pop_events(self) -> Sequence[Lease]:
        events, self._events = self._events, []
        return events
