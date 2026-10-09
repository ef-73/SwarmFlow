"""Orchestrator + global map server (design 6.5, 13.5)."""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context, *_args, **_kwargs):
    layout = LaunchConfiguration("layout").perform(context)
    layouts_root = LaunchConfiguration("layouts_root").perform(context)
    policy = LaunchConfiguration("policy").perform(context)
    robots = [r.strip() for r in LaunchConfiguration("robots").perform(context).split(",") if r.strip()]
    run_id = LaunchConfiguration("run_id").perform(context)
    runs_root = LaunchConfiguration("runs_root").perform(context)
    if not run_id:
        run_id = f"{os.environ.get('SWARMFLOW_SCENARIO') or 'adhoc'}-{policy}-latest"
    generated = os.path.join(layouts_root, layout, "generated")
    return [
        Node(package="swarmflow_orchestrator", executable="orchestrator_node", name="orchestrator",
             output="screen",
             parameters=[{"use_sim_time": True, "layout_dir": generated, "robots": robots, "policy": policy,
                          "run_dir": os.path.join(runs_root, run_id)}]),
        Node(package="nav2_map_server", executable="map_server", name="global_map_server", output="screen",
             parameters=[{"use_sim_time": True, "yaml_filename": os.path.join(generated, "map.yaml"),
                          "topic_name": "map"}]),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager", name="global_map_lifecycle",
             output="screen",
             parameters=[{"use_sim_time": True, "autostart": True, "node_names": ["global_map_server"]}]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("layout", default_value="standard"),
        DeclareLaunchArgument("layouts_root", default_value="/ws/layouts"),
        DeclareLaunchArgument("policy", default_value=EnvironmentVariable("SWARMFLOW_POLICY", default_value="fcfs")),
        DeclareLaunchArgument("robots", default_value=EnvironmentVariable(
            "SWARMFLOW_ROBOTS", default_value="robot_1,robot_2,robot_3")),
        DeclareLaunchArgument("run_id", default_value=EnvironmentVariable("SWARMFLOW_RUN_ID", default_value="")),
        DeclareLaunchArgument("runs_root", default_value="/ws/runs"),
        OpaqueFunction(function=_launch_setup),
    ])
