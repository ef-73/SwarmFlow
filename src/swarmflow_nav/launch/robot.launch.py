"""One robot's stack in its own namespace (design §7.3, §7.6 Option N): robot_state_publisher, ros_gz bridges,
map server, localization, Nav2 (composed into one container process), and the SwarmFlow robot agent.

Runs in a `robot_N` service (robot image). Every node uses sim time and the namespaced TF topics
(/robot_N/tf, /robot_N/tf_static) with unprefixed frame names.

Args: robot_id (robot_1), layout (standard), layouts_root (/ws/layouts),
      localization (ground_truth | amcl; design §15.3 cut 1 = ground_truth),
      agent (true|false), traffic_control (true|false, Baseline A = false).
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml

TF = [("/tf", "tf"), ("/tf_static", "tf_static")]
NAV_NODES = ["controller_server", "planner_server", "behavior_server", "bt_navigator", "velocity_smoother"]


def setup(context):
    rid = LaunchConfiguration("robot_id").perform(context)
    layout = LaunchConfiguration("layout").perform(context)
    root = LaunchConfiguration("layouts_root").perform(context)
    localization = LaunchConfiguration("localization").perform(context)
    agent = LaunchConfiguration("agent").perform(context).lower() == "true"
    traffic_control = LaunchConfiguration("traffic_control").perform(context).lower() == "true"
    gen = os.path.join(root, layout, "generated")
    lay = yaml.safe_load(open(os.path.join(root, layout, "layout.yaml"), encoding="utf-8"))
    spawn = next(s for s in lay["spawn"] if s["robot_id"] == rid)
    poses = {s["name"]: s["pose"] for s in lay["stations"]}
    poses.update({i["name"]: i["pose"] for i in lay["intersections"]})
    sx, sy, syaw = poses[spawn["vertex"]]["x"], poses[spawn["vertex"]]["y"], float(spawn.get("yaw", 0.0))

    import xacro
    desc = xacro.process_file(
        os.path.join(get_package_share_directory("swarmflow_description"), "urdf", "swarmflow_bot.urdf.xacro"),
        mappings={"robot_id": rid, "namespace": rid}).toxml()

    params = ParameterFile(RewrittenYaml(
        source_file=os.path.join(get_package_share_directory("swarmflow_nav"), "config", "nav2_params.yaml"),
        root_key=rid,
        param_rewrites={"use_sim_time": "True", "yaml_filename": os.path.join(gen, "map.yaml"),
                        },
        convert_types=True), allow_substs=True)
    sim = {"use_sim_time": True}

    ns = f"/{rid}"
    bridge_args = [
        f"{ns}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
        f"{ns}/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
        f"{ns}/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
        f"{ns}/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
        f"{ns}/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
        f"{ns}/wheel_odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
    ]
    actions = [
        Node(package="robot_state_publisher", executable="robot_state_publisher", namespace=rid, output="screen",
             parameters=[{"robot_description": desc, "use_sim_time": True}], remappings=TF),
        Node(package="ros_gz_bridge", executable="parameter_bridge", name="gz_bridge", namespace=rid,
             output="screen", arguments=bridge_args, parameters=[sim]),
    ]

    loc_nodes = [ComposableNode(package="nav2_map_server", plugin="nav2_map_server::MapServer", name="map_server",
                                namespace=rid, parameters=[params, sim], remappings=TF)]
    loc_names = ["map_server"]
    if localization == "amcl":
        loc_nodes.append(ComposableNode(package="nav2_amcl", plugin="nav2_amcl::AmclNode", name="amcl",
                                        namespace=rid, parameters=[params, sim, {
                                            "initial_pose": {"x": sx, "y": sy, "z": 0.0, "yaw": syaw}}],
                                        remappings=TF))
        loc_names.append("amcl")
    else:  # ground truth (design §7.6 fallback): Gazebo OdometryPublisher reports the absolute world pose
        # (verified M4: odom at spawn = (1.3, 1.1) for P1), so map → odom is the identity.
        actions.append(Node(package="tf2_ros", executable="static_transform_publisher", name="map_to_odom",
                            namespace=rid, parameters=[sim], remappings=TF,
                            arguments=["--frame-id", "map", "--child-frame-id", "odom"]))

    nav = [
        ComposableNode(package="nav2_controller", plugin="nav2_controller::ControllerServer", name="controller_server",
                       namespace=rid, parameters=[params, sim], remappings=TF + [("cmd_vel", "cmd_vel_nav")]),
        ComposableNode(package="nav2_planner", plugin="nav2_planner::PlannerServer", name="planner_server",
                       namespace=rid, parameters=[params, sim], remappings=TF),
        ComposableNode(package="nav2_behaviors", plugin="behavior_server::BehaviorServer", name="behavior_server",
                       namespace=rid, parameters=[params, sim], remappings=TF + [("cmd_vel", "cmd_vel_nav")]),
        ComposableNode(package="nav2_bt_navigator", plugin="nav2_bt_navigator::BtNavigator", name="bt_navigator",
                       namespace=rid, parameters=[params, sim], remappings=TF),
        ComposableNode(package="nav2_velocity_smoother", plugin="nav2_velocity_smoother::VelocitySmoother",
                       name="velocity_smoother", namespace=rid, parameters=[params, sim],
                       remappings=TF + [("cmd_vel", "cmd_vel_nav"), ("cmd_vel_smoothed", "cmd_vel")]),
        ComposableNode(package="nav2_lifecycle_manager", plugin="nav2_lifecycle_manager::LifecycleManager",
                       name="lifecycle_manager_localization", namespace=rid,
                       parameters=[sim, {"autostart": True, "node_names": loc_names, "bond_timeout": 8.0}]),
        ComposableNode(package="nav2_lifecycle_manager", plugin="nav2_lifecycle_manager::LifecycleManager",
                       name="lifecycle_manager_navigation", namespace=rid,
                       parameters=[sim, {"autostart": True, "node_names": NAV_NODES, "bond_timeout": 8.0}]),
    ]
    actions.append(ComposableNodeContainer(
        name="nav2_container", namespace=rid, package="rclcpp_components", executable="component_container_isolated",
        output="screen", parameters=[sim], remappings=TF, composable_node_descriptions=loc_nodes + nav))

    if agent:
        actions.append(Node(package="swarmflow_robot_agent", executable="agent_node", namespace=rid, output="screen",
                            remappings=TF, parameters=[sim, {"robot_id": rid, "layout_dir": gen,
                                                             "traffic_control": traffic_control}]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot_id", default_value="robot_1"),
        DeclareLaunchArgument("layout", default_value="standard"),
        DeclareLaunchArgument("layouts_root", default_value="/ws/layouts"),
        DeclareLaunchArgument("localization", default_value="ground_truth"),
        DeclareLaunchArgument("agent", default_value="false"),
        DeclareLaunchArgument("traffic_control", default_value="true"),
        OpaqueFunction(function=setup),
    ])
