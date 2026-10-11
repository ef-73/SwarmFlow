"""Read the generated Gazebo world (``generated/world.sdf``) into plain shapes, so Foxglove draws exactly what Gazebo
draws (T021). Pure Python (stdlib only): no ROS, no Gazebo.

Supported: models with a model pose, links with an optional link pose, visuals with an optional visual pose and a
``box`` or ``cylinder`` geometry plus a material colour. Poses compose in 2D (x, y, z, yaw); roll and pitch are
ignored (the generated worlds use none). Planes (the ground) are skipped; models whose name starts with one of
``dynamic_prefixes`` (packages moved at runtime) are returned separately.
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

Pose = Tuple[float, float, float, float]          # x, y, z, yaw
RGBA = Tuple[float, float, float, float]
DEFAULT_RGBA: RGBA = (0.7, 0.7, 0.7, 1.0)


@dataclass(frozen=True)
class Shape:
    model: str
    kind: str                                     # "box" | "cylinder"
    pose: Pose                                    # world frame
    size: Tuple[float, float, float]              # box: x, y, z; cylinder: 2r, 2r, length
    rgba: RGBA


@dataclass(frozen=True)
class World:
    name: str
    static: Tuple[Shape, ...]
    dynamic: Tuple[Shape, ...]                    # at their initial poses


def _floats(text: Optional[str], n: int) -> List[float]:
    vals = [float(v) for v in (text or "").split()]
    return (vals + [0.0] * n)[:n]


def _pose(el) -> Pose:
    p = el.find("pose") if el is not None else None
    x, y, z, _r, _p, yaw = _floats(p.text if p is not None else "", 6)
    return (x, y, z, yaw)


def compose(a: Pose, b: Pose) -> Pose:
    """Pose ``b`` expressed in frame ``a`` → world pose (2D rotation about z)."""
    c, s = math.cos(a[3]), math.sin(a[3])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], a[2] + b[2], a[3] + b[3])


def _rgba(visual) -> RGBA:
    for tag in ("material/diffuse", "material/ambient"):
        el = visual.find(tag)
        if el is not None and el.text:
            v = _floats(el.text, 4)
            if len(el.text.split()) == 3:
                v[3] = 1.0
            return (v[0], v[1], v[2], v[3])
    return DEFAULT_RGBA


def _shape(model: str, visual, pose: Pose) -> Optional[Shape]:
    geom = visual.find("geometry")
    if geom is None:
        return None
    box = geom.find("box")
    if box is not None:
        sx, sy, sz = _floats(box.findtext("size"), 3)
        return Shape(model, "box", pose, (sx, sy, sz), _rgba(visual))
    cyl = geom.find("cylinder")
    if cyl is not None:
        r = float(cyl.findtext("radius") or 0.0)
        length = float(cyl.findtext("length") or 0.0)
        return Shape(model, "cylinder", pose, (2 * r, 2 * r, length), _rgba(visual))
    return None                                   # plane, mesh, …: not drawn


def parse_world(text: str, dynamic_prefixes: Sequence[str] = ("pkg_",)) -> World:
    root = ET.fromstring(text)
    world = root.find("world")
    if world is None:
        raise ValueError("no <world> element")
    static: List[Shape] = []
    dynamic: List[Shape] = []
    for model in world.findall("model"):
        name = model.get("name", "")
        m_pose = _pose(model)
        out = dynamic if any(name.startswith(p) for p in dynamic_prefixes) else static
        for link in model.findall("link"):
            l_pose = compose(m_pose, _pose(link))
            for visual in link.findall("visual"):
                shape = _shape(name, visual, compose(l_pose, _pose(visual)))
                if shape is not None:
                    out.append(shape)
    return World(world.get("name", ""), tuple(static), tuple(dynamic))


def load_world(path: str, dynamic_prefixes: Sequence[str] = ("pkg_",)) -> World:
    with open(path, encoding="utf-8") as f:
        return parse_world(f.read(), dynamic_prefixes)
