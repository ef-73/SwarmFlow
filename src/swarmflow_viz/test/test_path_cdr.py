"""X-01b: the raw Path reader returns exactly what rclpy deserialization gives."""

import pytest
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.serialization import serialize_message

from swarmflow_viz.path_cdr import path_xy


def _path(points, frame="map", pose_frame="map"):
    p = Path()
    p.header.frame_id = frame
    p.header.stamp.sec, p.header.stamp.nanosec = 123, 456
    for i, (x, y) in enumerate(points):
        ps = PoseStamped()
        ps.header.frame_id = pose_frame
        ps.header.stamp.sec = i
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = x, y, 0.5
        ps.pose.orientation.w = 1.0
        p.poses.append(ps)
    return p


@pytest.mark.parametrize("frame,pose_frame", [("map", "map"), ("", ""), ("odom_x", "m"), ("abc", "abcdefg")])
def test_matches_rclpy(frame, pose_frame):
    pts = [(1.0 + 0.05 * i, -2.5 + 0.01 * i * i) for i in range(300)]
    data = serialize_message(_path(pts, frame, pose_frame))
    assert path_xy(data) == pts


def test_empty_and_malformed():
    assert path_xy(serialize_message(_path([]))) == []
    data = serialize_message(_path([(1.0, 2.0), (3.0, 4.0)]))
    with pytest.raises(ValueError):
        path_xy(data[:-20])
    with pytest.raises(ValueError):
        path_xy(b"\x00\x01")
