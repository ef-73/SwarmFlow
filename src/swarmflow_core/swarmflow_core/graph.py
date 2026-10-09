"""Warehouse graph loader and queries (design §6.1). Implements :class:`swarmflow_core.api.Graph`.

Loads the generated RMF-format nav graph plus the SwarmFlow sidecar (``zones.yaml``). Vertices are always referred to
by name; nav-graph indices are only used while loading.
"""

from __future__ import annotations

import heapq
import math
import pathlib
from typing import Dict, List, Mapping, Optional, Sequence, Tuple

import yaml

from .api import Edge, EdgeKey, Vertex, Zone, ZoneEntry


class WarehouseGraph:
    """Immutable directed graph with zones. Construct with :func:`load_graph` or :func:`load_layout`."""

    def __init__(self, vertices: Mapping[str, Vertex], edges: Mapping[EdgeKey, Edge], zones: Mapping[str, Zone],
                 payloads: Optional[Mapping[str, Mapping[str, float]]] = None, name: str = ""):
        self.name = name
        self.vertices: Dict[str, Vertex] = dict(vertices)
        self.edges: Dict[EdgeKey, Edge] = dict(edges)
        self.zones: Dict[str, Zone] = dict(zones)
        self.payloads: Dict[str, Mapping[str, float]] = dict(payloads or {})
        self._succ: Dict[str, List[str]] = {v: [] for v in self.vertices}
        for (a, b) in self.edges:
            if a not in self.vertices or b not in self.vertices:
                raise ValueError(f"edge {a}->{b} references an unknown vertex")
            self._succ[a].append(b)
        for v in self._succ:
            self._succ[v].sort()

    # -- api.Graph -----------------------------------------------------------------------------------------------
    def successors(self, vertex: str) -> Sequence[str]:
        return tuple(self._succ[vertex])

    def shortest_path(self, start: str, goal: str, payload_type: str = "small") -> Sequence[str]:
        if start not in self.vertices:
            raise KeyError(start)
        if goal not in self.vertices:
            raise KeyError(goal)
        if start == goal:
            return (start,)
        # Dijkstra; ties broken by path as a tuple of names (deterministic).
        best: Dict[str, Tuple[float, Tuple[str, ...]]] = {start: (0.0, (start,))}
        pq: List[Tuple[float, Tuple[str, ...]]] = [(0.0, (start,))]
        done = set()
        while pq:
            d, path = heapq.heappop(pq)
            u = path[-1]
            if u in done:
                continue
            done.add(u)
            if u == goal:
                return path
            for v in self._succ[u]:
                e = self.edges[(u, v)]
                if payload_type and e.feasible and not e.feasible.get(payload_type, True):
                    continue
                nd = round(d + e.length_m, 9)
                cand = (nd, path + (v,))
                if v not in best or cand < best[v]:
                    best[v] = cand
                    heapq.heappush(pq, cand)
        raise ValueError(f"no route {start} -> {goal} for payload {payload_type!r}")

    def path_length(self, route: Sequence[str]) -> float:
        total = 0.0
        for a, b in zip(route, route[1:]):
            total += self.edges[(a, b)].length_m
        return total

    def zone_of_edge(self, start: str, end: str) -> Optional[str]:
        e = self.edges.get((start, end))
        return e.zone if e else None

    def zone_at(self, x: float, y: float) -> Optional[str]:
        for name in sorted(self.zones):
            if point_in_polygon(x, y, self.zones[name].polygon):
                return name
        return None

    # -- helpers (not part of the frozen API) -----------------------------------------------------------------------
    def distance(self, a: str, b: str, payload_type: str = "small") -> float:
        """Route length between two vertices (inf if unreachable)."""
        try:
            return self.path_length(self.shortest_path(a, b, payload_type))
        except ValueError:
            return math.inf

    def nearest_vertex(self, x: float, y: float, kinds: Optional[Sequence[str]] = None) -> str:
        """Closest vertex by Euclidean distance (ties by name), optionally restricted to vertex kinds."""
        cands = [v for v in self.vertices.values() if kinds is None or v.kind in kinds]
        if not cands:
            raise ValueError("no candidate vertices")
        return min(cands, key=lambda v: (math.hypot(v.x - x, v.y - y), v.name)).name

    def vertices_of_kind(self, kind: str) -> Sequence[str]:
        return tuple(sorted(n for n, v in self.vertices.items() if v.kind == kind))

    def zone_entry(self, zone: str, entry_vertex: str) -> Optional[ZoneEntry]:
        for e in self.zones[zone].entries:
            if e.entry == entry_vertex:
                return e
        return None


def point_in_polygon(x: float, y: float, poly: Sequence[Tuple[float, float]], eps: float = 1e-9) -> bool:
    """True if (x, y) is inside or on the boundary of the polygon."""
    n = len(poly)
    for i in range(n):  # boundary
        (x1, y1), (x2, y2) = poly[i], poly[(i + 1) % n]
        cross = (x2 - x1) * (y - y1) - (y2 - y1) * (x - x1)
        if abs(cross) <= eps * max(1.0, math.hypot(x2 - x1, y2 - y1)) and \
                min(x1, x2) - eps <= x <= max(x1, x2) + eps and min(y1, y2) - eps <= y <= max(y1, y2) + eps:
            return True
    inside = False
    for i in range(n):
        (x1, y1), (x2, y2) = poly[i], poly[(i + 1) % n]
        if (y1 > y) != (y2 > y) and x < x1 + (y - y1) * (x2 - x1) / (y2 - y1):
            inside = not inside
    return inside


def load_graph(nav_graph_path, zones_path) -> WarehouseGraph:
    """Load ``generated/nav_graph.yaml`` + ``generated/zones.yaml`` (schemas in ``layouts/schema/``)."""
    ng = yaml.safe_load(pathlib.Path(nav_graph_path).read_text(encoding="utf-8"))
    side = yaml.safe_load(pathlib.Path(zones_path).read_text(encoding="utf-8"))
    level = ng["levels"]["L1"]
    names = [v[2]["name"] for v in level["vertices"]]
    sv = side["vertices"]
    vertices = {}
    for (x, y, props) in level["vertices"]:
        n = props["name"]
        meta = sv.get(n, {})
        vertices[n] = Vertex(name=n, x=float(x), y=float(y), kind=meta.get("kind", "intersection"),
                             yaw=(float(meta["yaw"]) if meta.get("yaw") is not None else None))
    side_edges = {(e["from"], e["to"]): e for e in side["edges"]}
    edges = {}
    for (i, j, _props) in level["lanes"]:
        a, b = names[i], names[j]
        se = side_edges.get((a, b))
        if se is None:
            raise ValueError(f"lane {a}->{b} missing from zones.yaml edges")
        edges[(a, b)] = Edge(start=a, end=b, length_m=float(se["length_m"]),
                             clear_width_m=float(se["clear_width_m"]), bidirectional=bool(se["bidirectional"]),
                             zone=se.get("zone"), feasible=dict(se.get("feasible", {})))
    zones = {}
    for z in side["zones"]:
        zones[z["name"]] = Zone(name=z["name"], capacity=int(z["capacity"]),
                                edges=tuple((a, b) for a, b in z["edges"]), vertices=tuple(z["vertices"]),
                                entries=tuple(ZoneEntry(e["entry"], e["hold"]) for e in z["entries"]),
                                polygon=tuple((float(x), float(y)) for x, y in z["polygon"]))
    return WarehouseGraph(vertices, edges, zones, side.get("payloads"), name=side.get("layout", ""))


def load_layout(generated_dir) -> WarehouseGraph:
    """Load a layout's ``generated/`` directory."""
    d = pathlib.Path(generated_dir)
    return load_graph(d / "nav_graph.yaml", d / "zones.yaml")
