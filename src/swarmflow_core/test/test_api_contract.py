"""Frozen API smoke test (design §6.7). Lead test."""
import dataclasses

from swarmflow_core import api


def test_api_version_and_constants():
    assert api.API_VERSION == 1
    assert api.LEASE_TTL_S == 5.0 and api.HEARTBEAT_PERIOD_S == 1.0
    assert api.ORCHESTRATOR_TIMEOUT_S == 10.0
    assert api.lease_hard_expiry(100.0, 110.0) == 120.0       # max(10, 5) = 10
    assert api.lease_hard_expiry(100.0, 140.0) == 160.0       # max(10, 20) = 20


def test_codes_match_interface_constants():
    # RobotState.MODE_*, OrderStatus.STATE_*, ZoneReservation.STATE_*, ReservationRelease.REASON_*, direction
    assert [api.MODE_CODES[m] for m in api.RobotMode] == list(range(7))
    assert api.MODE_CODES[api.RobotMode.FAULT] == 6
    assert api.ORDER_STATE_CODES[api.OrderState.FAILED] == 5
    assert api.LEASE_STATE_CODES[api.LeaseState.OCCUPIED_UNKNOWN] == 4
    assert api.RELEASE_REASON_CODES[api.ReleaseReason.FAULT] == 2
    assert api.DIRECTION_CODES[api.Direction.REVERSE] == 2


def test_dataclasses_frozen():
    o = api.OrderSpec("o1", 0.0, None, "L1", "D1")
    try:
        o.order_id = "x"  # type: ignore[misc]
        raise AssertionError("OrderSpec must be frozen")
    except dataclasses.FrozenInstanceError:
        pass
    r = api.RobotSnapshot("robot_1", 0.0, 0.0, 0.0, api.RobotMode.IDLE)
    assert r.held_lease_ids == () and r.payload_type == ""
