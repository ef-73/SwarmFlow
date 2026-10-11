"""Frozen API smoke test (design §6.7). Lead test."""
import dataclasses

from swarmflow_core import api


def test_api_version_and_constants():
    assert api.API_VERSION == 2
    assert api.LEASE_TTL_S == 5.0 and api.HEARTBEAT_PERIOD_S == 1.0
    assert api.ORCHESTRATOR_TIMEOUT_S == 10.0
    assert api.lease_hard_expiry(100.0, 110.0) == 120.0       # max(10, 5) = 10
    assert api.lease_hard_expiry(100.0, 140.0) == 160.0       # max(10, 20) = 20


def test_codes_match_interface_constants():
    # RobotState.MODE_*, OrderStatus.STATE_*, ZoneReservation.STATE_*, ReservationRelease.REASON_*, direction
    assert [api.MODE_CODES[m] for m in api.RobotMode] == list(range(9))
    assert api.MODE_CODES[api.RobotMode.FAULT] == 6
    assert api.MODE_CODES[api.RobotMode.CARRYING] == 8           # v2 modes appended: v1 codes unchanged
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


def test_v2_codes_parsed_from_interface_files():
    """v2 additions (decision 002): order types and route kinds match the .msg/.action constants."""
    order = _msg_constants("msg/Order.msg")
    assert {f"TYPE_{k.upper()}": c for k, c in api.ORDER_TYPE_CODES.items()} == order
    task = _msg_constants("action/DispatchTask.action")
    assert {f"ROUTE_{k.upper()}": c for k, c in api.ROUTE_KIND_CODES.items()} == task


def test_v2_defaults_keep_v1_behaviour():
    o = api.OrderSpec("o1", 0.0, None, "L1", "D1")
    assert o.order_type == api.ORDER_TRANSPORT and o.pick_vertices == () and o.pallet_id == ""
    a = api.Assignment("robot_1", "o1", "t1", ("P1", "L1", "D1"))
    assert a.route_kind == api.ROUTE_WAYPOINTS
    assert api.RobotSnapshot("robot_1", 0.0, 0.0, 0.0, api.RobotMode.IDLE).angular_speed == 0.0
    assert api.UNLOAD_DWELL_BOX_S == 5.0 and api.UNLOAD_DWELL_PALLET_S == 10.0


def test_v2_sample_orders_load_as_orderspecs():
    import pathlib
    import yaml
    f = pathlib.Path(__file__).resolve().parents[3] / "tests" / "fixtures" / "orders_v2_sample.yaml"
    specs = [api.OrderSpec(**{**o, "pick_vertices": tuple(o.get("pick_vertices", ()))})
             for o in yaml.safe_load(f.read_text(encoding="utf-8"))["orders"]]
    assert {s.order_type for s in specs} == {api.ORDER_SHELF, api.ORDER_PALLET}
    for s in specs:
        if s.order_type == api.ORDER_SHELF:
            assert s.pickup_vertex == "" and s.pick_vertices and len(set(s.pick_vertices)) == len(s.pick_vertices)
        assert s.dropoff_vertex.startswith("DK")
