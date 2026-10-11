"""Robot overlay and status board inside the Gazebo window (T021, user request: a dashboard in Gazebo too).

In the 3D view (gz markers): a ring under each robot coloured by its status (green OK, amber waiting or stale, red
stuck / fault), its path on the floor and its goal disc in the robot's colour. Text markers do not render with
Gazebo's ogre2 engine, so the status board (one line per robot) is published as text on the gz topic ``/echo`` and
shown, one message per robot, by the "SwarmFlow robots" TopicEcho panel (``config/gazebo_gui.config``). Markers are served by the Gazebo
GUI; without a GUI the requests fail quietly and are retried every few seconds, so a headless run pays almost
nothing.

``build_overlay`` is pure (plain dicts, testable without Gazebo); ``GzOverlay`` sends them with the gz-transport
Python bindings from a background thread so the ROS executor never blocks on Gazebo.
"""

from __future__ import annotations

import threading
import time
from typing import Dict, List, Mapping, Optional, Sequence, Tuple

from .fleet_view import ERROR, OK, WARN, robot_rgba

RGBA = Tuple[float, float, float, float]
NS = "swarmflow"
BOARD_TOPIC = "/echo"           #: TopicEcho's default topic: the panel works without typing a topic name
LEVEL_RGBA = {OK: (0.15, 0.75, 0.25, 0.9), WARN: (0.95, 0.70, 0.10, 0.9), ERROR: (0.90, 0.15, 0.10, 0.95)}


def build_overlay(robots: Sequence[str], states: Mapping[str, Mapping], levels: Mapping[str, int],
                  paths: Mapping[str, Sequence[Tuple[float, float]]],
                  goals: Mapping[str, Optional[Tuple[float, float]]]) -> List[Dict]:
    """Marker descriptions: dicts with ns, id, type (cylinder|line_strip|delete), pose (x, y, z), rgba, points,
    scale. Per robot ``idx``: id 10*idx status ring, +1 path, +2 goal disc."""
    out: List[Dict] = []
    for idx, rid in enumerate(robots):
        s = states.get(rid)
        rgba = robot_rgba(rid)
        base = idx * 10
        if s is None:
            out += [{"ns": NS, "id": base + k, "type": "delete"} for k in range(3)]
            continue
        out.append({"ns": NS, "id": base, "type": "cylinder", "pose": (float(s["x"]), float(s["y"]), 0.01),
                    "rgba": LEVEL_RGBA.get(levels.get(rid, WARN), LEVEL_RGBA[WARN]), "scale": (1.1, 1.1, 0.02)})
        pts = list(paths.get(rid, ()))
        if len(pts) >= 2 and s.get("task_id"):
            out.append({"ns": NS, "id": base + 1, "type": "line_strip", "pose": (0.0, 0.0, 0.0), "rgba": rgba,
                        "points": [(float(x), float(y), 0.03) for x, y in pts], "scale": (1.0, 1.0, 1.0)})
        else:
            out.append({"ns": NS, "id": base + 1, "type": "delete"})
        g = goals.get(rid)
        if g is not None and s.get("task_id"):
            out.append({"ns": NS, "id": base + 2, "type": "cylinder", "pose": (g[0], g[1], 0.02),
                        "rgba": (rgba[0], rgba[1], rgba[2], 0.7), "scale": (0.9, 0.9, 0.02)})
        else:
            out.append({"ns": NS, "id": base + 2, "type": "delete"})
    return out


def board_lines(lines: Sequence[str]) -> List[str]:
    """One plain-ASCII message per robot: TopicEcho prints each message on one line and escapes newlines and
    non-ASCII characters. With the panel's buffer set to the number of robots it reads as one line per robot."""
    table = str.maketrans({"·": "|", "→": "->"})
    return [ln.translate(table).encode("ascii", "replace").decode("ascii") for ln in lines]


class GzOverlay:
    """Sends overlay markers to the Gazebo GUI (``/marker_array``) from its own thread."""

    def __init__(self, period_s: float = 0.5, retry_s: float = 5.0, timeout_ms: int = 1000, logger=None):
        self._period, self._retry, self._timeout = period_s, retry_s, timeout_ms
        self._log = logger
        self._latest: Optional[Tuple[List[Dict], List[str]]] = None
        self._cv = threading.Condition()
        self._stop = False
        self._thread = threading.Thread(target=self._run, name="gz_overlay", daemon=True)
        self._ok = None

    def start(self) -> bool:
        try:
            from gz.msgs10.boolean_pb2 import Boolean  # noqa: F401
            from gz.msgs10.marker_pb2 import Marker  # noqa: F401
            from gz.msgs10.marker_v_pb2 import Marker_V  # noqa: F401
            from gz.transport13 import Node  # noqa: F401
        except ImportError:
            if self._log:
                self._log.info("gz-transport Python bindings not found: no overlay in the Gazebo window")
            return False
        self._thread.start()
        return True

    def update(self, markers: List[Dict], board: List[str]) -> None:
        with self._cv:
            self._latest = (markers, board)
            self._cv.notify()

    def stop(self) -> None:
        with self._cv:
            self._stop = True
            self._cv.notify()

    def _run(self) -> None:
        from gz.msgs10.boolean_pb2 import Boolean
        from gz.msgs10.marker_v_pb2 import Marker_V
        from gz.msgs10.stringmsg_pb2 import StringMsg
        from gz.transport13 import Node

        node = Node()
        board_pub = node.advertise(BOARD_TOPIC, StringMsg)
        next_try = 0.0
        fails = 0
        while True:
            with self._cv:
                self._cv.wait(timeout=self._period)
                if self._stop:
                    return
                latest, self._latest = self._latest, None
            if latest is None:
                continue
            markers, board = latest
            for line in board:
                board_pub.publish(StringMsg(data=line))
            if time.monotonic() < next_try:
                continue
            req = Marker_V()
            for d in markers:
                _fill(req.marker.add(), d)
            try:
                ok, _rep = node.request("/marker_array", req, Marker_V, Boolean, self._timeout)
            except Exception:  # noqa: BLE001
                ok = False
            fails = 0 if ok else fails + 1
            state = True if ok else (False if fails >= 3 or self._ok is None else self._ok)
            if state != self._ok and self._log:
                self._log.info("Gazebo overlay " + ("active" if state else "inactive (no Gazebo window)"))
            self._ok = state
            if fails >= 3:                       # a busy GUI may miss one deadline; back off only when it is gone
                next_try = time.monotonic() + self._retry


def _fill(m, d: Dict) -> None:
    from gz.msgs10.marker_pb2 import Marker

    m.ns, m.id = d["ns"], int(d["id"])
    if d["type"] == "delete":
        m.action = Marker.DELETE_MARKER
        return
    m.action = Marker.ADD_MODIFY
    # Re-sent every 0.5 s; a short lifetime removes copies left behind when a modify does not move the old one
    # (seen in M10/T021 runs as trails of status rings).
    m.lifetime.sec, m.lifetime.nsec = 3, 0
    m.type = {"line_strip": Marker.LINE_STRIP, "cylinder": Marker.CYLINDER}[d["type"]]
    m.visibility = Marker.GUI
    x, y, z = d.get("pose", (0.0, 0.0, 0.0))
    m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, z
    m.pose.orientation.w = 1.0
    sx, sy, sz = d.get("scale", (1.0, 1.0, 1.0))
    m.scale.x, m.scale.y, m.scale.z = sx, sy, sz
    r, g, b, a = d.get("rgba", (1.0, 1.0, 1.0, 1.0))
    for col in (m.material.ambient, m.material.diffuse):          # no emissive: it washes the colours out
        col.r, col.g, col.b, col.a = r, g, b, a
    for px, py, pz in d.get("points", ()):
        p = m.point.add()
        p.x, p.y, p.z = px, py, pz
