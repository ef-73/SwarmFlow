"""Launch the payload pose-follower. Robots from env SWARMFLOW_ROBOTS (comma/space separated) unless given."""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context, *args, **kwargs):
    layout = LaunchConfiguration("layout").perform(context)
    root = LaunchConfiguration("layouts_root").perform(context)
    robots = LaunchConfiguration("robots").perform(context).replace(",", " ").split()
    return [Node(
        package="swarmflow_payload",
        executable="payload_node",
        name="payload",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "layout_dir": f"{root}/{layout}/generated",
            "layout_file": f"{root}/{layout}/layout.yaml",
            "world": layout,
            "robots": robots,
        }],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("layout", default_value=os.environ.get("SWARMFLOW_LAYOUT", "standard")),
        DeclareLaunchArgument("layouts_root", default_value="/ws/layouts"),
        DeclareLaunchArgument("robots", default_value=os.environ.get("SWARMFLOW_ROBOTS", "robot_1 robot_2 robot_3")),
        OpaqueFunction(function=_setup),
    ])
