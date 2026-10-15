"""x, y of every pose of a serialized ``nav_msgs/Path`` (X-01b, improvements_v2 §15).

Deserializing a Nav2 plan in rclpy builds six Python objects per pose (PoseStamped, Header, Time, Pose, Point,
Quaternion); with four robots replanning every second that was the largest cost of the viz node (profiled
2026-10-15). The viz only needs x, y, so it subscribes raw and reads them straight from the CDR bytes.

CDR (XCDR1, as rmw_fastrtps / rmw_cyclonedds write it): a 4-byte encapsulation header, then fields aligned to their
size relative to the end of that header; strings are a uint32 length (including the NUL) and the bytes.
"""

from __future__ import annotations

import struct
from typing import List, Tuple

_BASE = 4   # alignment is relative to the end of the encapsulation header


def _align(off: int, n: int) -> int:
    r = (off - _BASE) % n
    return off if r == 0 else off + n - r


def path_xy(data: bytes) -> List[Tuple[float, float]]:
    """(x, y) of each pose; raises ValueError on a truncated or malformed message."""
    if len(data) < 8:
        raise ValueError("Path: too short")
    end = "<" if data[1] in (1, 3) else ">"          # 0x0001 / 0x0003 = little endian (CDR_LE / PL_CDR_LE)
    u32 = struct.Struct(end + "I")
    xy = struct.Struct(end + "dd")
    try:
        off = _BASE + 8                               # header.stamp: int32 sec, uint32 nanosec
        n = u32.unpack_from(data, off)[0]             # header.frame_id
        off += 4 + n
        off = _align(off, 4)
        count = u32.unpack_from(data, off)[0]         # poses: sequence length
        off += 4
        out = []
        for _ in range(count):
            off = _align(off, 4) + 8                  # pose.header.stamp
            n = u32.unpack_from(data, off)[0]
            off += 4 + n
            off = _align(off, 8)                      # pose.pose.position (float64 x, y, z) + orientation
            out.append(xy.unpack_from(data, off))
            off += 56
    except struct.error as e:
        raise ValueError(f"Path: malformed ({e})") from e
    if off > len(data):
        raise ValueError("Path: truncated")
    return out
