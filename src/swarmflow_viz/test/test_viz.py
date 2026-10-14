"""swarmflow_viz zone overlay (design §11.1). v1 robot/station markers and the v1 Foxglove layout were removed in X-02;
the v2 view is tested in test_fleet_view_t021.py. Lead test."""

from __future__ import annotations

import pathlib

import pytest

pytest.importorskip("visualization_msgs")
from visualization_msgs.msg import Marker  # noqa: E402

from swarmflow_core.api import LeaseState  # noqa: E402
from swarmflow_core.graph import load_layout  # noqa: E402
from swarmflow_viz.zones import ZONE_COLORS, zone_markers  # noqa: E402

ROOT = pathlib.Path(__file__).resolve().parents[3]
FIX = ROOT / "tests" / "fixtures"
GRAPH = load_layout(FIX / "standard")


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
