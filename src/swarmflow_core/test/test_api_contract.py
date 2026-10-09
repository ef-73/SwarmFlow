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


def _msg_constants(rel):
    import pathlib
    import re
    f = pathlib.Path(__file__).resolve().parents[2] / "swarmflow_interfaces" / rel
    out = {}
    for line in f.read_text(encoding="utf-8").splitlines():
        m = re.match(r"^uint8 ([A-Z_]+)=(\d+)", line)
        if m:
            out[m.group(1)] = int(m.group(2))
    return out


def test_codes_parsed_from_interface_files():
    """api *_CODES must equal the constants in the .msg/.srv files (review finding 9a)."""
    rs = _msg_constants("msg/RobotState.msg")
    assert {f"MODE_{m.value}": c for m, c in api.MODE_CODES.items()} == rs
    os_ = _msg_constants("msg/OrderStatus.msg")
    assert {f"STATE_{s.value}": c for s, c in api.ORDER_STATE_CODES.items()} == os_
    zr = _msg_constants("msg/ZoneReservation.msg")
    assert {f"STATE_{s.value}": c for s, c in api.LEASE_STATE_CODES.items()} == zr
    rr = _msg_constants("msg/ReservationRelease.msg")
    assert {f"REASON_{r.value}": c for r, c in api.RELEASE_REASON_CODES.items()} == rr
    req = _msg_constants("srv/RequestReservation.srv")
    assert {f"DIRECTION_{d.value}": c for d, c in api.DIRECTION_CODES.items()} == \
        {k: v for k, v in req.items() if k.startswith("DIRECTION_")}
    assert {k: v for k, v in req.items() if k.startswith("RESULT_")} == {"RESULT_GRANTED": 0, "RESULT_DENIED": 1}
