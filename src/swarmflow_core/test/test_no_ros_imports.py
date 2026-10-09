"""swarmflow_core must not import rclpy or any ROS package (design §6.7, AGENTS.md §9). Lead test."""
import ast
import pathlib

ROOT = pathlib.Path(__file__).resolve().parents[1] / "swarmflow_core"
FORBIDDEN = ("rclpy", "rosidl", "rclcpp", "std_msgs", "geometry_msgs", "nav_msgs", "nav2_msgs",
             "builtin_interfaces", "visualization_msgs", "swarmflow_interfaces", "launch", "launch_ros",
             "ament_index_python", "tf2_ros", "rosbag2_py", "ros_gz")


def test_no_ros_imports():
    files = sorted(ROOT.rglob("*.py"))
    assert files, f"no sources under {ROOT}"
    offenders = []
    for f in files:
        tree = ast.parse(f.read_text(encoding="utf-8"), filename=str(f))
        for node in ast.walk(tree):
            names = []
            if isinstance(node, ast.Import):
                names = [a.name for a in node.names]
            elif isinstance(node, ast.ImportFrom) and node.module and node.level == 0:
                names = [node.module]
            for n in names:
                if n.split(".")[0] in FORBIDDEN or n.endswith("_msgs"):
                    offenders.append(f"{f.relative_to(ROOT.parent)}:{node.lineno} imports {n}")
    assert not offenders, "\n".join(offenders)
