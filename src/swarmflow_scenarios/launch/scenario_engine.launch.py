"""Launch the scenario_engine. Run id rule (same as the orchestrator): env SWARMFLOW_RUN_ID, else
``<scenario>-<policy>-latest``; an explicit ``run_id`` launch argument wins over both."""

import os
import pathlib

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context, *args, **kwargs):
    scenario = LaunchConfiguration("scenario").perform(context)
    root = pathlib.Path(LaunchConfiguration("scenarios_root").perform(context))
    runs_root = pathlib.Path(LaunchConfiguration("runs_root").perform(context))
    layouts_root = pathlib.Path(LaunchConfiguration("layouts_root").perform(context))
    scenario_file = root / f"{scenario}.yaml"
    data = yaml.safe_load(scenario_file.read_text(encoding="utf-8"))
    run_id = (LaunchConfiguration("run_id").perform(context) or os.environ.get("SWARMFLOW_RUN_ID", "")
              or f"{scenario}-{data['policy']}-latest")
    return [Node(
        package="swarmflow_scenarios",
        executable="scenario_engine",
        name="scenario_engine",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "scenario_file": str(scenario_file),
            "layout_dir": str(layouts_root / data["layout"] / "generated"),
            "run_dir": str(runs_root / run_id),
            "git_sha": os.environ.get("SWARMFLOW_GIT_SHA", ""),
            "image_digests": os.environ.get("SWARMFLOW_IMAGE_DIGESTS", ""),
            "scenarios_tools": LaunchConfiguration("scenarios_tools").perform(context),
        }],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("scenario", default_value=os.environ.get("SWARMFLOW_SCENARIO", "v1_demo")),
        DeclareLaunchArgument("scenarios_root", default_value="/ws/scenarios"),
        DeclareLaunchArgument("layouts_root", default_value="/ws/layouts"),
        DeclareLaunchArgument("scenarios_tools", default_value="/ws/tools/scenarios"),
        DeclareLaunchArgument("run_id", default_value=""),
        DeclareLaunchArgument("runs_root", default_value="/ws/runs"),
        OpaqueFunction(function=_setup),
    ])
