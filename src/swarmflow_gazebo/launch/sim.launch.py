"""Headless Gazebo server for SwarmFlow (design §7.6, §8.1): world, /clock bridge, robot spawning.

Runs in the `gazebo` service (sim image). Robots are spawned from the layout's `spawn` list; their Nav2 stacks and
bridges run in the `robot_N` containers (swarmflow_nav robot.launch.py).

Args: layout (standard), layouts_root (/ws/layouts), robots (comma list, default = every spawn entry),
      rtf (real-time factor cap, design §8.5; 1.0 = real time).
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

COLORS = {"robot_1": "0.10 0.45 0.85 1", "robot_2": "0.95 0.55 0.10 1", "robot_3": "0.20 0.70 0.30 1",
          "robot_4": "0.75 0.20 0.70 1"}


def robot_xacro(robot_id: str) -> str:
    import xacro
    path = os.path.join(get_package_share_directory("swarmflow_description"), "urdf", "swarmflow_bot.urdf.xacro")
    return xacro.process_file(path, mappings={"robot_id": robot_id, "namespace": robot_id,
                                              "color": COLORS.get(robot_id, "0.5 0.5 0.5 1")}).toxml()


def setup(context):
    layout = LaunchConfiguration("layout").perform(context)
    root = LaunchConfiguration("layouts_root").perform(context)
    wanted = [r for r in LaunchConfiguration("robots").perform(context).split(",") if r]
    rtf = float(LaunchConfiguration("rtf").perform(context))
    lay = yaml.safe_load(open(os.path.join(root, layout, "layout.yaml"), encoding="utf-8"))
    world = os.path.join(root, layout, "generated", "world.sdf")
    if rtf != 1.0:  # cap the real-time factor (thermal budget) on a copy of the world
        txt = open(world, encoding="utf-8").read().replace(
            "<real_time_factor>1.0</real_time_factor>", f"<real_time_factor>{rtf}</real_time_factor>")
        world = "/tmp/swarmflow_world.sdf"
        open(world, "w", encoding="utf-8").write(txt)

    poses = {s["name"]: s["pose"] for s in lay["stations"]}
    poses.update({i["name"]: {**i["pose"], "yaw": 0.0} for i in lay["intersections"]})
    actions = [
        ExecuteProcess(cmd=["gz", "sim", "-s", "-r", "-v", "2", world], output="screen"),
        Node(package="ros_gz_bridge", executable="parameter_bridge", name="clock_bridge", output="screen",
             arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"]),
    ]
    spawns = []
    for s in lay["spawn"]:
        rid = s["robot_id"]
        if wanted and rid not in wanted:
            continue
        p = poses[s["vertex"]]
        spawns.append(Node(
            package="ros_gz_sim", executable="create", name=f"spawn_{rid}", output="screen",
            arguments=["-world", lay["name"], "-name", rid, "-string", robot_xacro(rid),
                       "-x", str(p["x"]), "-y", str(p["y"]), "-z", "0.02", "-Y", str(s.get("yaw", 0.0))]))
    actions.append(TimerAction(period=3.0, actions=spawns))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("layout", default_value="standard"),
        DeclareLaunchArgument("layouts_root", default_value="/ws/layouts"),
        DeclareLaunchArgument("robots", default_value=""),
        DeclareLaunchArgument("rtf", default_value="1.0"),
        OpaqueFunction(function=setup),
    ])
