import os
import sys
from collections import namedtuple

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import xacro  # noqa: E402
from ament_index_python.packages import get_package_share_directory  
from launch.actions import ( 
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit  
from launch.launch_description_sources import PythonLaunchDescriptionSource  
from launch_ros.actions import Node  

from fr_universe_launch_utils import load_yaml  


ControllerConfig = namedtuple(
    "ControllerConfig", ["parameters", "controllers_yaml", "rail_yaml", "gripper_yaml"]
)

# SDK-based joint-state forwarder used only when robot_hardware_connected=true
# AND listen_only_mode=true (no ros2_control command interface for the arm in
# that mode, so this script -- talking to the SDK directly -- is the only
# thing actually driving/reading the real robot).
FORWARDER_SCRIPT_PATH = os.path.join(
    get_package_share_directory("fairino_universe"), "scripts", "ros2_forwarder.py"
)



PLANNING_SCENE_RELAY_PATH = os.path.join(
    get_package_share_directory("fairino_universe"), "scripts", "planning_scene_relay.py"
)


# joint(s) actuated by each supported gripper, kept for reference / future use
# if per-gripper controller parameter generation is reintroduced. (not useful right now)
GRIPPER_JOINT_MAP = {
    "dh_ag95": ["dh_ag95_left_outer_knuckle_joint"],
    "dh_ag145": ["dh_ag145_gripper_finger1_joint"],
    "robotiq_2f_85": ["robotiq_2f_85_finger_joint"],
    "dh_pgc140": ["dh_pgc140_gripper_finger1_joint"],
    "dh_ag3": ["gripper_joint_X", "gripper_joint_X2"],
}
# which controller group to use / default is gripper_controller,
# for some gripper where the arm have have 2 joints we need a different controller.
GRIPPER_CONTROLLER_NAME_MAP = {
    "dh_ag95": "gripper_controller",
    "dh_ag145": "gripper_controller",
    "robotiq_2f_85": "gripper_controller",
    "dh_pgc140": "gripper_controller",
    "dh_ag3": "gripper_controller2",
}

def gripper_controller_name(gripper):
    """Controller name for a gripper, matching the keys in the gripper yaml."""
    return f"{gripper}_gripper_controller"

def append_robot_description_actions(cfg, actions,use_sim_time):
    """
    Appends the robot_state_publisher node to `actions` in place.

    Returns robot_description so downstream builders — ros2_control, the Gazebo
    spawn, move_group — reuse the same parsed URDF instead of re-running xacro.

    Raises RuntimeError if the xacro fails to process, since every later step
    depends on it and a traceback from inside xacro is harder to read than a
    message naming the file.
    """
    try:
        robot_description_xml = xacro.process_file(
            cfg.xacro_path,
            mappings=cfg.xacro_mappings,
        ).toxml()
    except Exception as exc:
        raise RuntimeError(f"failed to process xacro {cfg.xacro_path}: {exc}") from exc

    robot_description = {"robot_description": robot_description_xml}

    actions.append(
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            respawn=True,
            output="screen",
            parameters=[robot_description, {"use_sim_time": False}],
        )
    )

    return robot_description



def resolve_controller_parameters(cfg):
    """
    Resolves the controller yaml paths and builds ros2_control_node's parameter list.

    Controller manager config comes from the selected moveit_pkg; the base/rail
    and gripper controller yamls come from fairino_universe/config, with
    filenames chosen in launch_params.yaml.

    Raises RuntimeError on a missing yaml rather than skipping it -- a controller
    manager that comes up without its controllers fails later, at spawner
    timeout, where the cause is much harder to see.
    """
    controllers_yaml_path = os.path.join(
        get_package_share_directory(cfg.moveit_pkg), "config", "ros2_controllers.yaml"
    )

    fairino_universe_config_dir = os.path.join(
        get_package_share_directory("fairino_universe"), "config"
    )
    rail_yaml_path = os.path.join(fairino_universe_config_dir, cfg.base_controller_filename)
    gripper_yaml_path = os.path.join(
        fairino_universe_config_dir, cfg.gripper_controller_filename
    )

    for path in (controllers_yaml_path, rail_yaml_path):
        if not os.path.exists(path):
            raise RuntimeError(f"controller config not found: {path}")

    parameters = [{"use_sim_time": False}, controllers_yaml_path, rail_yaml_path]

    if cfg.gripper != "none":
        if not os.path.exists(gripper_yaml_path):
            raise RuntimeError(
                f"gripper='{cfg.gripper}' but controller config not found: {gripper_yaml_path}"
            )
        parameters.append(gripper_yaml_path)

    return ControllerConfig(
        parameters=parameters,
        controllers_yaml=controllers_yaml_path,
        rail_yaml=rail_yaml_path,
        gripper_yaml=gripper_yaml_path,
    )


def append_controller_actions(cfg, actions, ctrl):
    """
    Appends ros2_control_node (hardware only) and the controller spawners.

    The arm spawner is chained to by the rail and gripper spawners via
    OnProcessExit rather than fixed TimerActions: that spawner exiting is the
    real "controller_manager is up and joint_state_broadcaster is active"
    signal, and a fixed delay races the Gazebo plugin's startup.

    Returns arm_broadcaster_spawner so later builders can hang their own
    event handlers off the same signal.
    """
    if cfg.control_system != "gazebo":
        actions.append(
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                parameters=ctrl.parameters,
                remappings=[("/controller_manager/robot_description", "/robot_description")],
                output="screen",
            )
        )
    else:
        actions.append(
            LogInfo(msg="control_system='gazebo': controller_manager comes from the "
                        "gz_ros2_control plugin, skipping standalone ros2_control_node")
        )

    arm_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            f"{cfg.robot_model}_controller",
            "-c", "/controller_manager",
            "--controller-manager-timeout", "10",
            "--param-file", ctrl.controllers_yaml,
        ],
        output="screen",
    )
    actions.append(TimerAction(period=1.0, actions=[arm_broadcaster_spawner]))

    # SDK-based joint-state forwarder (ros2_forwarder.py): only when
    # robot_hardware_connected=true AND listen_only_mode=true. In that mode the
    # xacro loads mock_components/GenericSystem instead of the real
    # FairinoHardwareInterface and drops the arm's command_interface entirely,
    # so this script -- going through the vendor SDK directly -- is the only
    # real connection. It must never run alongside a live
    # FairinoHardwareInterface, since both would race to command the arm.
    if cfg.hardware_enabled and cfg.listen_only_mode_enabled:
        forwarder_process = ExecuteProcess(
            cmd=[
                "python3", FORWARDER_SCRIPT_PATH,
                "--ros-args",
                "-p", f"robot_ip:={cfg.robot_ip_address}",
            ],
            output="screen",
        )
        actions.append(
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=arm_broadcaster_spawner,
                    on_exit=[forwarder_process],
                )
            )
        )

    # Rail / mount controller (only if the robot isn't mounted directly to 'world')
    if cfg.mount != "world":
        rail_spawner = Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                f"{cfg.mount}_controller",
                "-c", "/controller_manager",
                "-t", "joint_trajectory_controller/JointTrajectoryController",
                "--controller-manager-timeout", "10",
                "--param-file", ctrl.rail_yaml,
            ],
            output="screen",
        )
        actions.append(
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=arm_broadcaster_spawner,
                    on_exit=[rail_spawner],
                )
            )
        )

    # Gripper controller spawner
    if cfg.gripper != "none":
        if cfg.gripper not in GRIPPER_CONTROLLER_NAME_MAP:
            raise RuntimeError(
                f"unknown gripper '{cfg.gripper}'; expected one of "
                f"{sorted(GRIPPER_CONTROLLER_NAME_MAP)}"
            )

        gripper_spawner = Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                GRIPPER_CONTROLLER_NAME_MAP[cfg.gripper],
                "-c", "/controller_manager",
                "--controller-manager-timeout", "10",
                "--param-file", ctrl.gripper_yaml,
            ],
            output="screen",
        )
        actions.append(
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=arm_broadcaster_spawner,
                    on_exit=[gripper_spawner],
                )
            )
        )

    return arm_broadcaster_spawner



def append_gazebo_actions(cfg, actions):
    """
    Appends the Gazebo server launch and the robot spawn entity.

    Returns world_sdf_path (None when Gazebo is disabled) so downstream
    consumers -- gazebo_world_to_moveit.py in particular -- get the resolved
    file rather than re-deriving it.
    """
    if not cfg.gazebo_enabled:
        return None

    world_sdf_path = os.path.join(
        get_package_share_directory("fairino_gazebo_config"),
        "worlds",
        f"{cfg.world_name}.sdf",
    )
    if not os.path.exists(world_sdf_path):
        raise RuntimeError(f"world file not found: {world_sdf_path}")

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py"
            )
        ),
        launch_arguments={"gz_args": f"{world_sdf_path} -r"}.items(),
    )
    actions.append(TimerAction(period=2.0, actions=[gz_sim]))

    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-topic", "robot_description"],
        output="screen",
    )
    actions.append(TimerAction(period=4.0, actions=[spawn_entity]))

    return world_sdf_path


def append_moveit_actions(cfg, actions, world_sdf_path):
    """
    Appends move_group, RViz, and the Gazebo world -> planning scene bridge.

    world_sdf_path comes from append_gazebo_actions and is None when Gazebo is
    disabled, in which case the world bridge is skipped entirely -- there is no
    world to import into the planning scene.
    """
    if not cfg.moveit_enabled:
        return

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
                "use_sim_time": str(False),
                "robot_model": cfg.robot_model,
                "robot_mount": cfg.mount,
                "control_system": cfg.control_system,
                "moveit_pkg": cfg.moveit_pkg,
                "gripper": cfg.gripper,
                "env_config": cfg.env_config,
            }.items(),
        )
    )
    # add the planning scene relay script, which publish updates to foxglove
    actions.append(
        ExecuteProcess(
            cmd=["python3", PLANNING_SCENE_RELAY_PATH],
            name="planning_scene_relay",
            output="screen",
            respawn=True,
            respawn_delay=2.0,
        )
    )
    
    actions.append(
        Node(
            package="foxglove_bridge",
            executable="foxglove_bridge",
            name="foxglove_bridge",
            output="screen",
            respawn=True,
            respawn_delay=2.0,
            parameters=[{
                "port": 8765,
                "address": "0.0.0.0",
                "use_sim_time": False,
                "asset_uri_allowlist": [
                    r"^package://(?:[-\w%]+/)*[-\w%]+\.(?:dae|DAE|fbx|glb|gltf|jpeg|jpg"
                    r"|mtl|obj|png|stl|STL|tif|tiff|urdf|webp|xacro)$"
                ],
            }],
        )
    )
    if cfg.rviz_enabled:
        kinematics_yaml = load_yaml(cfg.moveit_pkg, "config/kinematics.yaml")
        actions.append(
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=[
                    "-d",
                    os.path.join(
                        get_package_share_directory(cfg.moveit_pkg), "config", "moveit.rviz"
                    ),
                ],
                parameters=[
                    {"use_sim_time": False},
                    {"robot_description_kinematics": kinematics_yaml},
                ],
                output="screen",
            )
        )

    if world_sdf_path is not None:
        actions.append(
            Node(
                package="fairino_gazebo_config",
                executable="gazebo_world_to_moveit.py",
                arguments=[world_sdf_path],
                parameters=[{"use_sim_time": False}],
                output="screen",
            )
        )