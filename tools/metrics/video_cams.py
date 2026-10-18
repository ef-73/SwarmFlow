"""Camera rig and cut planning for the 3D demo video (pure Python, no ROS, no OpenCV; unit-tested).

record_cams.py spawns the cameras from ``CAMERAS`` and records their frames with sim-time stamps;
make_demo_video.py finds the pallet events in the run record and turns the frames into one edited video
with ``plan_cuts`` and ``frames_for``.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Sequence, Tuple


@dataclass(frozen=True)
class Camera:
    name: str
    eye: Tuple[float, float, float]      #: camera position, map frame (m)
    target: Tuple[float, float, float]   #: point the camera looks at
    width: int
    height: int
    hfov: float                          #: horizontal field of view (rad)
    rate_hz: float                       #: frames per SIM second


# warehouse_v2: 20 x 32 m, racks in two blocks (x 0-8, 12-20; y 8-22), pallet stands at y 5.5, docks at y 30.6.
CAMERAS: Dict[str, Camera] = {
    # whole warehouse, 3/4 view from beyond the south-west corner
    "wide": Camera("wide", (-6.0, -11.0, 19.0), (10.0, 12.0, 0.0), 1920, 1080, math.radians(60), 6.0),
    # pallet stands and home area, from just inside the south wall
    "pallets": Camera("pallets", (10.0, 0.4, 5.0), (10.0, 7.5, 0.0), 1920, 1080, math.radians(80), 10.0),
    # queue lanes and truck docks, looking north from the end of the racks
    "docks": Camera("docks", (10.0, 22.0, 7.5), (10.0, 28.0, 0.0), 1920, 1080, math.radians(84), 10.0),
}


def look_at_rpy(eye: Sequence[float], target: Sequence[float]) -> Tuple[float, float, float]:
    """Roll, pitch, yaw for a Gazebo camera (looks along +x) at ``eye`` looking at ``target``."""
    dx, dy, dz = (target[0] - eye[0], target[1] - eye[1], target[2] - eye[2])
    return 0.0, math.atan2(-dz, math.hypot(dx, dy)), math.atan2(dy, dx)


def camera_sdf(cam: Camera, topic: str) -> str:
    """A static model with one camera sensor publishing on gz topic ``topic``."""
    r, p, y = look_at_rpy(cam.eye, cam.target)
    x, yy, z = cam.eye
    return (
        f'<?xml version="1.0"?><sdf version="1.9"><model name="video_cam_{cam.name}"><static>true</static>'
        f'<pose>{x} {yy} {z} {r} {p} {y}</pose><link name="link">'
        f'<sensor name="cam" type="camera"><always_on>1</always_on><update_rate>{cam.rate_hz}</update_rate>'
        f'<topic>{topic}</topic><camera><horizontal_fov>{cam.hfov:.4f}</horizontal_fov>'
        f'<image><width>{cam.width}</width><height>{cam.height}</height><format>R8G8B8</format></image>'
        f'<clip><near>0.1</near><far>120</far></clip>'
        f'<anti_aliasing>4</anti_aliasing></camera></sensor></link></model></sdf>')


# ---- run record → events ---------------------------------------------------------------------------------------

@dataclass(frozen=True)
class PalletMove:
    order_id: str
    robot_id: str
    lift_start: float      #: first LIFTING sample of the pick-up
    lift_end: float        #: first CARRYING sample
    setdown_start: float   #: first LIFTING sample after carrying (setting down at the dock)
    setdown_end: float     #: last LIFTING sample of the set-down
    delivered: float       #: order DELIVERED


def pallet_moves(states: Iterable[dict], orders: Iterable[dict]) -> List[PalletMove]:
    """Completed pallet moves, from robot_states.jsonl and orders.jsonl rows."""
    delivered: Dict[str, Tuple[str, float]] = {}
    for o in orders:
        if o.get("order_type") == "pallet" and o.get("state") == "DELIVERED":
            delivered[o["order_id"]] = (o["robot_id"], float(o["t"]))
    by_order: Dict[str, List[Tuple[float, str]]] = {}
    for s in states:
        oid = s.get("order_id", "")
        if oid in delivered and s.get("robot_id") == delivered[oid][0]:
            by_order.setdefault(oid, []).append((float(s["t"]), s.get("mode", "")))
    moves = []
    for oid, rows in by_order.items():
        rows.sort()
        lifts = [t for t, m in rows if m == "LIFTING"]
        carry = [t for t, m in rows if m == "CARRYING"]
        if not lifts or not carry:
            continue
        pick = [t for t in lifts if t < carry[0]]
        drop = [t for t in lifts if t > carry[0]]
        if not pick:
            continue
        moves.append(PalletMove(oid, delivered[oid][0], pick[0], carry[0], drop[0] if drop else carry[-1],
                                drop[-1] if drop else carry[-1], delivered[oid][1]))
    return sorted(moves, key=lambda m: m.lift_start)


def deliveries_until(orders: Sequence[dict], t: float) -> int:
    return sum(1 for o in orders if o.get("state") == "DELIVERED" and float(o["t"]) <= t)


# ---- cut planning --------------------------------------------------------------------------------------------

@dataclass(frozen=True)
class Cut:
    cam: str
    t0: float              #: sim time of the first frame
    t1: float              #: sim time of the last frame
    speed: float           #: sim seconds per video second
    caption: str = ""


def plan_cuts(move: Optional[PalletMove], t_start: float, t_end: float) -> List[Cut]:
    """Wide shot of the fleet, the pallet pick-up close up, the carry wide, the set-down close up, wide to the end."""
    if move is None:
        return [Cut("wide", t_start, t_end, 6.0)]
    a = max(t_start, move.lift_start - 12.0)
    b = move.lift_end + 6.0
    c = max(b, move.setdown_start - 12.0)
    d = min(t_end, move.setdown_end + 8.0)
    cuts = []
    if a - t_start > 1.0:
        cuts.append(Cut("wide", t_start, a, 6.0, "Four robots fulfil a seeded order stream"))
    cuts.append(Cut("pallets", a, b, 3.0, "Pallet move: drive under, lift"))
    if c - b > 1.0:
        cuts.append(Cut("wide", b, c, 6.0, "Carrying the pallet to a truck dock"))
    cuts.append(Cut("docks", c, d, 3.0, "Set down at the dock"))
    if t_end - d > 1.0:
        cuts.append(Cut("wide", d, t_end, 8.0, "Shelf orders: pick in the aisles, unload at the docks"))
    return cuts


def frames_for(stamps: Sequence[float], t0: float, t1: float, speed: float, fps: float) -> List[int]:
    """Indices into ``stamps`` (sorted sim times of recorded frames) for an output clip at ``fps``: for each output
    frame, the recorded frame nearest in sim time. Dropped frames repeat their neighbour instead of shifting time."""
    out: List[int] = []
    if not stamps:
        return out
    n = max(1, int(round((t1 - t0) / speed * fps)))
    j = 0
    for k in range(n):
        t = t0 + k * speed / fps
        while j + 1 < len(stamps) and abs(stamps[j + 1] - t) <= abs(stamps[j] - t):
            j += 1
        out.append(j)
    return out
