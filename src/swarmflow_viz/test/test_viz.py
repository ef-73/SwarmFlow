"""swarmflow_viz marker builders + Foxglove layout (design §11.1; WS-E "layout shows robots, plans, zones,
decisions from fixtures"). Lead test: do not edit."""

from __future__ import annotations

import json
import pathlib

import pytest

pytest.importorskip("visualization_msgs")
from visualization_msgs.msg import Marker  # noqa: E402

from swarmflow_core.api import LeaseState  # noqa: E402
from swarmflow_core.graph import load_layout  # noqa: E402
from swarmflow_viz.markers import ZONE_COLORS, robot_markers, station_markers, zone_markers  # noqa: E402

ROOT = pathlib.Path(__file__).resolve().parents[3]
FIX = ROOT / "tests" / "fixtures"
GRAPH = load_layout(FIX / "standard")


def fixture_states(t):
    rows = [json.loads(x) for x in (FIX / "robot_states_standard.jsonl").read_text().splitlines()]
    return [r for r in rows if abs(r["t"] - t) < 1e-6]


def test_robot_markers_from_fixture_stream():
    states = fixture_states(30.0)
    assert len(states) == 3
    ma = robot_markers(states, stamp_sec=30.0)
    bodies = [m for m in ma.markers if m.ns == "robots"]
    labels = [m for m in ma.markers if m.ns == "robot_labels"]
    assert len(bodies) == 3 and len(labels) == 3
    for m in bodies + labels:
        assert m.header.frame_id == "map" and m.action == Marker.ADD
    b = {m.id: m for m in bodies}
    s0 = sorted(states, key=lambda s: s["robot_id"])[0]
    m0 = b[1]                                             # id = robot number
    assert m0.type == Marker.CUBE
    assert (m0.scale.x, m0.scale.y) == pytest.approx((0.60, 0.50))
    assert (m0.pose.position.x, m0.pose.position.y) == pytest.approx((s0["x"], s0["y"]))
    assert any(s0["mode"] in m.text and "robot_1" in m.text for m in labels)


def test_zone_markers_colour_by_state():
    leases = {"Z_aisle_1": LeaseState.GRANTED, "Z_aisle_2": LeaseState.OCCUPIED_UNKNOWN}
    ma = zone_markers(GRAPH, leases, stamp_sec=1.0)
    zones = [m for m in ma.markers if m.ns == "zones"]
    assert len(zones) == len(GRAPH.zones)
    by_text = {}
    for m in ma.markers:
        if m.ns == "zone_labels":
            by_text[m.text.split()[0]] = m
    assert set(by_text) == set(GRAPH.zones)
    col = {name: zones[i].color for i, name in enumerate(sorted(GRAPH.zones))}
    def rgb(c):
        return (round(c.r, 2), round(c.g, 2), round(c.b, 2))
    assert rgb(col["Z_aisle_1"]) == ZONE_COLORS["GRANTED"][:3]
    assert rgb(col["Z_aisle_2"]) == ZONE_COLORS["OCCUPIED_UNKNOWN"][:3]
    assert rgb(col["Z_aisle_3"]) == ZONE_COLORS["FREE"][:3]
    for m in zones:
        assert m.type in (Marker.LINE_STRIP, Marker.TRIANGLE_LIST) and len(m.points) >= 4


def test_station_markers():
    ma = station_markers(GRAPH, stamp_sec=0.0)
    names = {m.text for m in ma.markers if m.type == Marker.TEXT_VIEW_FACING}
    assert {"L1", "L2", "L3", "D1", "D2", "D3", "P1", "P2", "P3", "P4"} <= names


def test_foxglove_layout_references_v1_topics():
    p = ROOT / "viz" / "foxglove" / "swarmflow_v1.json"
    layout = json.loads(p.read_text(encoding="utf-8"))
    assert "configById" in layout and "layout" in layout
    text = json.dumps(layout)
    for topic in ("/map", "/fleet/robot_markers", "/fleet/zone_markers", "/fleet/decisions", "/fleet/order_status",
                  "/robot_1/plan", "/robot_2/plan", "/robot_3/plan", "/fleet/robot_states"):
        assert topic in text, topic
    kinds = {k.split("!")[0] for k in layout["configById"]}
    assert {"3D", "RawMessages", "Plot"} <= kinds or {"3D", "Table", "Plot"} <= kinds
