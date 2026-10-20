"""One robot's stack in its own namespace (design §7.3, §7.6 Option N): robot_state_publisher, ros_gz bridges,
map server, localization, Nav2 (composed into one container process), and the SwarmFlow robot agent.

Runs in a `robot_N` service (robot image). Every node uses sim time and the namespaced TF topics
(/robot_N/tf, /robot_N/tf_static) with unprefixed frame names.

Args: robot_id (robot_1), layout (standard), layouts_root (/ws/layouts),
      localization (ground_truth | amcl; design §15.3 cut 1 = ground_truth),
      agent (true|false), traffic_control (true|false, Baseline A = false).
"""

import os
import tempfile

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


class _NoAliasDumper(yaml.SafeDumper):
    """rcl's parameter-file parser rejects YAML anchors/aliases ("Will not support aliasing")."""

    def ignore_aliases(self, data):
        return True


def _deep_merge(base: dict, over: dict) -> None:
    """Overlay values win; nested dicts merge; lists and scalars are replaced."""
    for k, v in over.items():
        if isinstance(v, dict) and isinstance(base.get(k), dict):
            _deep_merge(base[k], v)
        else:
            base[k] = v


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
    profile = LaunchConfiguration("nav_profile").perform(context)
    if profile == "auto":                     # v2 layouts get the v2 navigation profile (M14)
        profile = "v2" if int(lay.get("schema_version", 1)) >= 2 else "v1"
    share = get_package_share_directory("swarmflow_nav")

    import xacro
    desc = xacro.process_file(
        os.path.join(get_package_share_directory("swarmflow_description"), "urdf", "swarmflow_bot.urdf.xacro"),
        mappings={"robot_id": rid, "namespace": rid, "lidar": "slot360" if profile == "v2" else "front"}).toxml()

    source = os.path.join(share, "config", "nav2_params.yaml")
    container_args = []          # v1 keeps running exactly as recorded in the baseline (see the comment below)
    if profile == "v2":
        # one merged file: a second parameter file did not replace list parameters such as costmap `plugins`
        # (M14 session 5: both costmaps fell back to Nav2's default layers)
        base = yaml.safe_load(open(source, encoding="utf-8"))
        _deep_merge(base, yaml.safe_load(open(os.path.join(share, "config", "nav2_params_v2.yaml"), encoding="utf-8")))
        # keep-right aisles (N-09) from the layout sidecar: x_min, y_min, x_max, y_max, axis (0 = x, 1 = y) per aisle
        side = yaml.safe_load(open(os.path.join(gen, "zones.yaml"), encoding="utf-8"))
        flat = [float(v) for a in side.get("aisles", []) for v in
                (a["x_min"], a["y_min"], a["x_max"], a["y_max"], 0.0 if a["axis"] == "x" else 1.0)]
        for cm in ("local_costmap", "global_costmap"):
            base[cm][cm]["ros__parameters"]["keep_right"]["aisles"] = list(flat)
        # E-01 "± prediction" (N-05): SWARMFLOW_PREDICTION_HORIZON=0 stamps only the other robots' current footprints
        horizon = os.environ.get("SWARMFLOW_PREDICTION_HORIZON")
        if horizon is not None and horizon != "":
            for cm in ("local_costmap", "global_costmap"):
                base[cm][cm]["ros__parameters"]["other_robots"]["horizon_s"] = float(horizon)
        # J-02 (v2.1): SWARMFLOW_FOOTPRINT_CRITIC=1 makes MPPI score the whole footprint, not only the
        # centre cell (pick points are 0.45 m from the rack; turning in place needs 0.47 m)
        fc = os.environ.get("SWARMFLOW_FOOTPRINT_CRITIC", "")
        if fc != "":
            critic = base["controller_server"]["ros__parameters"]["FollowPath"]["CostCritic"]
            critic["consider_footprint"] = fc in ("1", "true")
        # J-04 (v2.1): SWARMFLOW_PREDICT_ALONG_PLAN=1 predicts other robots along their published plans
        if os.environ.get("SWARMFLOW_PREDICT_ALONG_PLAN", "") in ("1", "true"):
            for cm in ("local_costmap", "global_costmap"):
                base[cm][cm]["ros__parameters"]["other_robots"]["use_plans"] = True
        # T069: SWARMFLOW_SMOOTH=1 smooths the motion: realistic acceleration limits in MPPI and in the velocity
        # smoother (the 3 m/s^2, 3.5 rad/s^2 limits let every MPPI correction reach the wheels at once) and less
        # turn-rate sampling noise. Not used (run s069_1, 0 deliveries): Nav2's TwirlingCritic penalises the turn rate
        # itself (meant for omnidirectional robots), so robots could not U-turn at the aisle ends and cut rack corners;
        # a stronger Smac path smoother (w_smooth 0.5) also lets the path cut corners.
        if os.environ.get("SWARMFLOW_SMOOTH", "") in ("1", "true"):
            apply_smoothing(base)
        source = os.path.join(tempfile.gettempdir(), f"swarmflow_{rid}_nav2_params_v2.yaml")
        with open(source, "w", encoding="utf-8") as f:
            yaml.dump(base, f, Dumper=_NoAliasDumper, sort_keys=False)
        # Composed Nav2: launch_ros hands a parameter file only to the composable node it is attached to, so the
        # costmap sub-nodes (<rid>/local_costmap/local_costmap, .../global_costmap) never saw their parameters and ran
        # on Nav2 defaults (found in M14: v1 included). A --params-file on the container process applies to every
        # node in it, sub-nodes too.
        for sect in base.values():
            if isinstance(sect, dict):
                for node in [sect] + [v for v in sect.values() if isinstance(v, dict)]:
                    if "ros__parameters" in node:
                        node["ros__parameters"]["use_sim_time"] = True
        base.setdefault("map_server", {}).setdefault("ros__parameters", {})["yaml_filename"] = os.path.join(gen, "map.yaml")
        container_params = os.path.join(tempfile.gettempdir(), f"swarmflow_{rid}_nav2_container.yaml")
        with open(container_params, "w", encoding="utf-8") as f:
            yaml.dump({rid: base}, f, Dumper=_NoAliasDumper, sort_keys=False)
        container_args = ["--params-file", container_params]
    params = ParameterFile(RewrittenYaml(
        source_file=source, root_key=rid,
        param_rewrites={"use_sim_time": "True", "yaml_filename": os.path.join(gen, "map.yaml")},
        convert_types=True), allow_substs=True)
    sim = {"use_sim_time": True}
    param_files = [params]
    bt_xml = os.path.join(share, "behavior_trees", "navigate_through_poses.xml")
    if profile == "v2":
        # per-robot recovery wait (N-03): robot_1 1.0 s, robot_2 1.7 s, ... so blocked robots do not move in lockstep
        n = int("".join(c for c in rid if c.isdigit()) or 1)
        tree = open(os.path.join(share, "behavior_trees", "navigate_v2.xml"), encoding="utf-8").read()
        bt_xml = os.path.join(tempfile.gettempdir(), f"swarmflow_{rid}_navigate_v2.xml")
        with open(bt_xml, "w", encoding="utf-8") as f:
            f.write(tree.replace("{recovery_wait_s}", f"{1.0 + 0.7 * ((n - 1) % 4):.1f}"))

    ns = f"/{rid}"
    bridge_args = [
        f"{ns}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
        f"{ns}/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
        f"{ns}/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
        f"{ns}/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
        f"{ns}/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
        f"{ns}/wheel_odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
    ]
    bridge_remaps = []
    if localization == "amcl":
        # realism ladder rung 2 (I-01): Nav2 and AMCL see the DiffDrive wheel odometry (it drifts with slip, M4) instead
        # of Gazebo's ground-truth odometry, which stays available on <ns>/odom_gt for the localisation-error metric
        bridge_args.append(f"{ns}/wheel_tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V")
        bridge_args.remove(f"{ns}/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V")
        bridge_remaps = [(f"{ns}/odom", f"{ns}/odom_gt"), (f"{ns}/wheel_odom", f"{ns}/odom"),
                         (f"{ns}/wheel_tf", f"{ns}/tf")]
    actions = [
        Node(package="robot_state_publisher", executable="robot_state_publisher", namespace=rid, output="screen",
             parameters=[{"robot_description": desc, "use_sim_time": True}], remappings=TF),
        Node(package="ros_gz_bridge", executable="parameter_bridge", name="gz_bridge", namespace=rid,
             output="screen", arguments=bridge_args, parameters=[sim], remappings=bridge_remaps),
    ]

    loc_nodes = [ComposableNode(package="nav2_map_server", plugin="nav2_map_server::MapServer", name="map_server",
                                namespace=rid, parameters=[*param_files, sim], remappings=TF)]
    loc_names = ["map_server"]
    if localization == "amcl":
        loc_nodes.append(ComposableNode(package="nav2_amcl", plugin="nav2_amcl::AmclNode", name="amcl",
                                        namespace=rid, parameters=[*param_files, sim, {
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
                       namespace=rid, parameters=[*param_files, sim], remappings=TF + [("cmd_vel", "cmd_vel_nav")]),
        ComposableNode(package="nav2_planner", plugin="nav2_planner::PlannerServer", name="planner_server",
                       namespace=rid, parameters=[*param_files, sim], remappings=TF),
        ComposableNode(package="nav2_behaviors", plugin="behavior_server::BehaviorServer", name="behavior_server",
                       namespace=rid, parameters=[*param_files, sim], remappings=TF + [("cmd_vel", "cmd_vel_nav")]),
        ComposableNode(package="nav2_bt_navigator", plugin="nav2_bt_navigator::BtNavigator", name="bt_navigator",
                       namespace=rid, remappings=TF, parameters=[*param_files, sim, {
                           "default_nav_through_poses_bt_xml": bt_xml}]),
        ComposableNode(package="nav2_velocity_smoother", plugin="nav2_velocity_smoother::VelocitySmoother",
                       name="velocity_smoother", namespace=rid, parameters=[*param_files, sim],
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
        output="screen", parameters=[sim], remappings=TF, ros_arguments=container_args,
        composable_node_descriptions=loc_nodes + nav))

    if agent:
        actions.append(Node(package="swarmflow_robot_agent", executable="agent_node", namespace=rid, output="screen",
                            remappings=TF, parameters=[sim, {"robot_id": rid, "layout_dir": gen,
                                                             "traffic_control": traffic_control,
                                                             # T068: reserved trajectories / deadlock breaker
                                                             "yield_to_others": os.environ.get(
                                                                 "SWARMFLOW_YIELD", "") in ("1", "true"),
                                                             "unjam": os.environ.get(
                                                                 "SWARMFLOW_UNJAM", "") in ("1", "true")}]))
    return actions


SMOOTH_ACCEL = (1.5, 2.5)        #: T069: m/s^2, rad/s^2 (MPPI and velocity smoother; were 3.0 / 3.5 and 2.5 / 3.2)
SMOOTH_WZ_STD = 0.3               #: T069: MPPI turn-rate sampling noise (was 0.4)


def apply_smoothing(base: dict) -> None:
    """T069 smoothing profile, in place on the merged v2 parameter dict."""
    fp = base["controller_server"]["ros__parameters"]["FollowPath"]
    a, aw = SMOOTH_ACCEL
    fp.update({"ax_max": a, "ax_min": -a, "az_max": aw, "wz_std": SMOOTH_WZ_STD})
    vs = base["velocity_smoother"]["ros__parameters"]
    vs["max_accel"] = [a, 0.0, aw]
    vs["max_decel"] = [-a, 0.0, -aw]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot_id", default_value="robot_1"),
        DeclareLaunchArgument("layout", default_value="standard"),
        DeclareLaunchArgument("layouts_root", default_value="/ws/layouts"),
        DeclareLaunchArgument("localization", default_value="ground_truth"),
        DeclareLaunchArgument("agent", default_value="false"),
        DeclareLaunchArgument("traffic_control", default_value="true"),
        DeclareLaunchArgument("nav_profile", default_value="auto"),   # auto | v1 | v2 (M14)
        OpaqueFunction(function=setup),
    ])
