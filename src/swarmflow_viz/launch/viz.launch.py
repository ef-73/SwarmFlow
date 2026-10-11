"""Viz stack for Foxglove (T021): viz node and TF relay. Runs in the sim image (gz-transport Python for packages)."""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    layout = LaunchConfiguration("layout").perform(context)
    root = LaunchConfiguration("layouts_root").perform(context)
    robots = [r for r in LaunchConfiguration("robots").perform(context).split(",") if r]
    sim = {"use_sim_time": True}
    return [
        Node(package="swarmflow_viz", executable="viz_node", name="swarmflow_viz", output="screen",
             parameters=[sim, {"layout_dir": os.path.join(root, layout, "generated"), "robots": robots}]),
        Node(package="swarmflow_viz", executable="tf_relay", name="swarmflow_tf_relay", output="screen",
             parameters=[sim, {"robots": robots}]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("layout", default_value="standard"),
        DeclareLaunchArgument("layouts_root", default_value=os.environ.get("SWARMFLOW_LAYOUTS_ROOT", "layouts")),
        DeclareLaunchArgument("robots", default_value=os.environ.get("SWARMFLOW_ROBOTS", "robot_1,robot_2,robot_3")),
        OpaqueFunction(function=setup),
    ])
