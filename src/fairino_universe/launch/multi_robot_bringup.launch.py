"""
Fairino MULTI-robot bring-up launch file.

Loads N robot arms (possibly different models: fairino5 + fairino10, etc.)
into a single combined robot_description / TF tree, so they can share one
MoveIt planning scene. Driven by a `robots.yaml` config, e.g.:

    robots:
      - name: arm1
        robot_model: fairino5
        prefix: arm1_
        mount: world
        origin_xyz: [0.0, 0.0, 0.0]
        origin_rpy: [0.0, 0.0, 0.0]
        gripper: dh_ag95
        hardware_connected: false
        robot_ip_address: 192.168.58.2
      - name: arm2
        robot_model: fairino10
        prefix: arm2_
        mount: world
        origin_xyz: [1.2, 0.0, 0.0]
        origin_rpy: [0.0, 0.0, 3.14159]
        gripper: robotiq_2f_85
        hardware_connected: false
        robot_ip_address: 192.168.58.3

-------------------------------------------------------------------------
WHAT THIS FILE DOES AND DOES NOT SOLVE
-------------------------------------------------------------------------
SOLVES (generic, works for any N / any combo of models):
  - Merges N single-robot xacro outputs into ONE <robot> element, each
    wrapped with a unique link/joint `prefix` and welded to /world (or a
    shared rail) via a fixed joint at the given origin. One robot_state_
    publisher, one TF tree, no name collisions.
  - Spawns one controller_manager PER ROBOT, in that robot's own ROS 2
    namespace, so ros2_control node names / controller names never clash
    even though everything shares the same TF tree.

DOES NOT SOLVE (must be authored once per combo you actually use):
  - The MoveIt SRDF: planning groups (one per arm), kinematics solver per
    group, joint/velocity limits per group, and the self-collision matrix
    (so arm1's links aren't flagged as permanently colliding with arm2's
    mount, and vice versa). This is inherently static/topology-specific,
    so it has to be built with the MoveIt Setup Assistant (or hand-edited
    SRDF) against the *merged* URDF this script produces for a *specific*
    set of robots+prefixes. See the checklist at the bottom of this file.

  Practically: since you'll typically only run 2-3 robots at a time out of
  a larger pool of models, the workflow is:
    1. Pick the combo (e.g. fairino5+fairino10).
    2. Run this launch file once with `moveit:=false` just to dump the
       merged URDF (see `--dump-urdf` note below).
    3. Feed that URDF into the MoveIt Setup Assistant, build a
       `<combo_name>_moveit_config` package with groups
       "arm1_manipulator", "arm2_manipulator", etc.
    4. Point `moveit_config_pkg` (declared below) at that package.
    5. Re-run this launch file with `moveit:=true`.
-------------------------------------------------------------------------
"""

import os

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from xml.etree import ElementTree as ET


def _truthy(value: str) -> bool:
    return value.strip().lower() == "true"


def load_robots_config(path_or_name):
    """
    Loads the robots.yaml config. `path_or_name` may be an absolute path,
    or a bare filename that will be looked up under
    fairino_universe/config/<name>.
    """
    if os.path.isabs(path_or_name) and os.path.exists(path_or_name):
        config_path = path_or_name
    else:
        config_path = os.path.join(
            get_package_share_directory("fairino_universe"), "config", path_or_name
        )
    with open(config_path) as f:
        data = yaml.safe_load(f) or {}
    robots = data.get("robots", [])
    if not robots:
        raise RuntimeError(f"No robots defined in {config_path}")
    return robots


def build_single_robot_xacro(robot_cfg, control_system, listen_only_mode):
    """
    Processes the existing per-robot xacro file for ONE robot entry and
    returns the resulting <robot>...</robot> Element, already namespaced
    with this robot's prefix.

    PREREQUISITE: test_fairino.urdf.xacro (and whatever it includes) must
    accept a `prefix` xacro arg and apply it to every link/joint name it
    generates (this is the standard pattern -- e.g. xacro:property
    name="prefix" and "${prefix}link_name" everywhere). If the current
    xacro doesn't yet support a prefix, that's a one-time addition to the
    xacro itself, not to this launch file.
    """
    xacro_path = os.path.join(
        get_package_share_directory("fairino_description"),
        "robots",
        "test_fairino.urdf.xacro",
    )

    mappings = {
        "robot_model": robot_cfg["robot_model"],
        "robot_mount": robot_cfg.get("mount", "world"),
        "control_system": control_system,
        "gripper": robot_cfg.get("gripper", "none"),
        "rail_length": str(robot_cfg.get("rail_length", "0.2")),
        "rail_width": str(robot_cfg.get("rail_width", "0.2")),
        "gripper_hardware_connected": str(robot_cfg.get("gripper_hardware_connected", "false")),
        "gripper_hardware_plugin": robot_cfg.get(
            "gripper_hardware_plugin", "mock_components/GenericSystem"
        ),
        "listen_only_mode": listen_only_mode,
        "robot_ip_address": robot_cfg.get("robot_ip_address", "192.168.58.2"),
        # NEW arg this multi-robot flow depends on:
        "prefix": robot_cfg["prefix"],
    }

    xml_string = xacro.process_file(xacro_path, mappings=mappings).toxml()
    return ET.fromstring(xml_string)


def merge_robot_descriptions(robot_cfgs, control_system, listen_only_mode):
    """
    Processes each robot's xacro separately, then splices all of them into
    a single <robot name="multi_robot_cell"> element:
      - all links/joints/gazebo tags from every robot are copied in
        (already prefixed, so no name collisions)
      - a fixed joint is added from `world` to each robot's
        `<prefix>base_link` at that robot's configured origin, so every
        arm ends up correctly placed in one shared TF tree.
    Returns the merged description as an XML string.
    """
    merged_root = ET.Element("robot", {"name": "multi_robot_cell"})

    # single shared world link
    ET.SubElement(merged_root, "link", {"name": "world"})

    for robot_cfg in robot_cfgs:
        robot_elem = build_single_robot_xacro(robot_cfg, control_system, listen_only_mode)
        prefix = robot_cfg["prefix"]

        # copy every child (links, joints, gazebo plugins, transmissions...)
        for child in list(robot_elem):
            merged_root.append(child)

        # weld this robot's base_link to world at its configured pose
        xyz = robot_cfg.get("origin_xyz", [0.0, 0.0, 0.0])
        rpy = robot_cfg.get("origin_rpy", [0.0, 0.0, 0.0])
        fixed_joint = ET.SubElement(
            merged_root,
            "joint",
            {"name": f"{prefix}world_joint", "type": "fixed"},
        )
        ET.SubElement(fixed_joint, "parent", {"link": "world"})
        ET.SubElement(fixed_joint, "child", {"link": f"{prefix}base_link"})
        ET.SubElement(
            fixed_joint,
            "origin",
            {"xyz": " ".join(str(v) for v in xyz), "rpy": " ".join(str(v) for v in rpy)},
        )

    return ET.tostring(merged_root, encoding="unicode")


def launch_setup(context, *args, **kwargs):
    robots_config_file = LaunchConfiguration("robots_config").perform(context)
    moveit_enabled = _truthy(LaunchConfiguration("moveit").perform(context))
    hardware_enabled = _truthy(LaunchConfiguration("robot_hardware_connected").perform(context))
    listen_only_mode = LaunchConfiguration("listen_only_mode").perform(context)
    rviz_enabled = _truthy(LaunchConfiguration("rviz_enabled").perform(context))
    moveit_config_pkg = LaunchConfiguration("moveit_config_pkg").perform(context)
    dump_urdf = _truthy(LaunchConfiguration("dump_urdf").perform(context))

    control_system = "hardware" if hardware_enabled else "moveit"

    robots = load_robots_config(robots_config_file)
    # apply a default prefix "<name>_" if not explicitly set
    for r in robots:
        r.setdefault("prefix", f"{r['name']}_")

    actions = [
        LogInfo(
            msg="Loading robots: "
            + ", ".join(f"{r['name']}({r['robot_model']}, prefix={r['prefix']})" for r in robots)
        )
    ]

    # ---------------------------------------------------------------
    # Merged robot_description (one TF tree for every robot)
    # ---------------------------------------------------------------
    merged_urdf_xml = merge_robot_descriptions(robots, control_system, listen_only_mode)

    if dump_urdf:
        dump_path = "/tmp/multi_robot_merged.urdf"
        with open(dump_path, "w") as f:
            f.write(merged_urdf_xml)
        actions.append(
            LogInfo(
                msg=f"[dump_urdf] Wrote merged URDF to {dump_path} -- "
                f"feed this into the MoveIt Setup Assistant to build "
                f"'{moveit_config_pkg}'. Not launching anything else."
            )
        )
        return actions

    robot_description = {"robot_description": merged_urdf_xml}

    actions.append(
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            respawn=True,
            output="screen",
            parameters=[robot_description, {"use_sim_time": False}],
        )
    )

    # ---------------------------------------------------------------
    # One controller_manager PER ROBOT, each in its own namespace, so
    # controller names / node names never collide even though they all
    # publish into the same shared TF tree above.
    # ---------------------------------------------------------------
    for robot_cfg in robots:
        ns = robot_cfg["name"]
        moveit_pkg_single = f"{robot_cfg['robot_model']}_v6_moveit2_config"
        controllers_yaml_path = os.path.join(
            get_package_share_directory(moveit_pkg_single), "config", "ros2_controllers.yaml"
        )

        actions.append(
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                namespace=ns,
                parameters=[{"use_sim_time": False}, controllers_yaml_path],
                remappings=[
                    (f"/{ns}/controller_manager/robot_description", "/robot_description"),
                ],
                output="screen",
            )
        )

        actions.append(
            TimerAction(
                period=1.0,
                actions=[
                    Node(
                        package="controller_manager",
                        executable="spawner",
                        namespace=ns,
                        arguments=[
                            "joint_state_broadcaster",
                            f"{robot_cfg['robot_model']}_controller",
                            "-c", f"/{ns}/controller_manager",
                            "--controller-manager-timeout", "10",
                            "--param-file", controllers_yaml_path,
                        ],
                        output="screen",
                    )
                ],
            )
        )

    # ---------------------------------------------------------------
    # MoveIt 2 -- ONE move_group over the merged description, using the
    # combined SRDF/config package you build for this specific robot
    # combination (see module docstring + checklist below).
    # ---------------------------------------------------------------
    if moveit_enabled:
        actions.append(
            LogInfo(
                msg=f"[INFO] Launching move_group with moveit_config_pkg='{moveit_config_pkg}'. "
                f"This package's SRDF must define one planning group per robot prefix "
                f"({', '.join(r['prefix'] + 'manipulator' for r in robots)}) "
                f"and a self-collision matrix covering cross-robot link pairs."
            )
        )
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("fairino_universe"),
                        "launch",
                        "move_group.launch.py",
                    )
                ),
                launch_arguments={
                    "use_sim_time": "false",
                    "moveit_pkg": moveit_config_pkg,
                    # NOTE: move_group.launch.py must be updated to accept a
                    # pre-built robot_description string (this merged one)
                    # rather than re-processing xacro itself, since the
                    # merged description no longer corresponds to any
                    # single robot_model.
                    "robot_description_override": merged_urdf_xml,
                }.items(),
            )
        )
        if rviz_enabled:
            actions.append(
                Node(
                    package="rviz2",
                    executable="rviz2",
                    arguments=[
                        "-d",
                        os.path.join(
                            get_package_share_directory(moveit_config_pkg), "config", "moveit.rviz"
                        ),
                    ],
                    parameters=[{"use_sim_time": False}, robot_description],
                )
            )

    return actions


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "robots_config",
            default_value="robots.yaml",
            description="Path (or bare filename under fairino_universe/config/) to the "
                         "multi-robot YAML config listing every robot instance to load.",
        ),
        DeclareLaunchArgument(
            "moveit",
            default_value="false",
            description="Set to true to launch the combined move_group + RViz.",
        ),
        DeclareLaunchArgument(
            "moveit_config_pkg",
            default_value="multi_robot_moveit_config",
            description="Name of the hand-built MoveIt config package covering this "
                         "specific combination of robots (see docstring at top of file).",
        ),
        DeclareLaunchArgument(
            "robot_hardware_connected",
            default_value="false",
            description="Set to true if ALL listed robots should run against real hardware "
                         "(per-robot hardware toggles can be added to robots.yaml if you need "
                         "a mixed sim/real cell).",
        ),
        DeclareLaunchArgument(
            "listen_only_mode",
            default_value="true",
            description="If true, ros2_control interfaces are read-only for all arms.",
        ),
        DeclareLaunchArgument(
            "rviz_enabled",
            default_value="true",
            description="Set to true to launch RViz alongside MoveIt.",
        ),
        DeclareLaunchArgument(
            "dump_urdf",
            default_value="false",
            description="If true, just write the merged URDF to /tmp/multi_robot_merged.urdf "
                         "and exit -- use this to generate input for the MoveIt Setup "
                         "Assistant when building moveit_config_pkg for a new combo.",
        ),
    ]

    return LaunchDescription(
        [
            *declared_arguments,
            OpaqueFunction(function=launch_setup),
        ]
    )


# -------------------------------------------------------------------------
# CHECKLIST: building `multi_robot_moveit_config` for a given combo
# -------------------------------------------------------------------------
# 1. `ros2 launch fairino_universe multi_robot_bringup.launch.py
#     robots_config:=my_combo.yaml dump_urdf:=true`
#    -> writes /tmp/multi_robot_merged.urdf
#
# 2. Open the MoveIt Setup Assistant against that URDF:
#    `ros2 run moveit_setup_assistant moveit_setup_assistant`
#
# 3. In "Planning Groups", add one group per robot prefix, e.g.
#    "arm1_manipulator" (kinematic chain arm1_base_link -> arm1_tool0) and
#    "arm2_manipulator" (arm2_base_link -> arm2_tool0). Add matching
#    "arm1_gripper" / "arm2_gripper" groups if grippers are attached.
#
# 4. In "Self-Collisions", regenerate the default collision matrix --the
#    assistant will correctly disable adjacent-link pairs on each arm and
#    flag any always-colliding pairs between the two arms' mounts/fixtures.
#    Double check any pairs it left "Never" between the two robots'
#    static base plates if they're physically close.
#
# 5. Generate the config package as `multi_robot_moveit_config` (or a more
#    descriptive combo-specific name) and pass that name to
#    `moveit_config_pkg:=` above.
#
# 6. Repeat steps 1-5 once per combination you plan to run regularly (this
#    is normal practice for multi-arm MoveIt cells -- it's a one-time
#    authoring cost per topology, not per launch).
# -------------------------------------------------------------------------
