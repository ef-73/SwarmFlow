import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    layout_dir = PathJoinSubstitution([LaunchConfiguration("layouts_root"), LaunchConfiguration("layout"),
                                       "generated"])
    return LaunchDescription([
        DeclareLaunchArgument("layout", default_value="standard"),
        DeclareLaunchArgument("layouts_root", default_value=os.environ.get("SWARMFLOW_LAYOUTS_ROOT", "layouts")),
        Node(package="swarmflow_viz", executable="viz_node", name="swarmflow_viz", output="screen",
             parameters=[{"use_sim_time": True, "layout_dir": layout_dir}]),
    ])
