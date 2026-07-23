from moveit_configs_utils import MoveItConfigsBuilder
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
import yaml


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None


MOVEIT_PKG_MAP = {
    "fairino3":  "fairino3_v6_moveit2_config",
    "fairino5":  "fairino5_v6_moveit2_config",
    "fairino10": "fairino10_v6_moveit2_config",
    "fairino16": "fairino16_v6_moveit2_config",
    "fairino20": "fairino20_v6_moveit2_config",
    "fairino30": "fairino30_v6_moveit2_config",
}


def launch_setup(context, *args, **kwargs):
    robot_model_str = LaunchConfiguration('robot_model').perform(context)
    robot_mount = LaunchConfiguration('robot_mount').perform(context)
    control_system = LaunchConfiguration('control_system').perform(context)
    gripper = LaunchConfiguration('gripper').perform(context)
    use_sim_time_str = LaunchConfiguration('use_sim_time').perform(context)
    use_sim_time = use_sim_time_str.strip().lower() == 'true'
    env_config = LaunchConfiguration('env_config').perform(context)
    gripper_hardware_plugin = LaunchConfiguration('gripper_hardware_plugin').perform(context)
    gripper_hardware_connected = LaunchConfiguration('gripper_hardware_connected').perform(context)

    moveit_pkg = MOVEIT_PKG_MAP.get(robot_model_str, f"{robot_model_str}_v6_moveit2_config")

    description_pkg_share = get_package_share_directory('fairino_description')

    moveit_config = (
        MoveItConfigsBuilder(f"{robot_model_str}_v6_robot", package_name=moveit_pkg)
        .robot_description(
            file_path=os.path.join(description_pkg_share, 'robots', 'test_fairino.urdf.xacro'),
            mappings={
                "robot_model": robot_model_str,
                "robot_mount": robot_mount,
                "control_system": control_system,
                "gripper": gripper,
                "gripper_hardware_connected": gripper_hardware_connected, 
                "gripper_hardware_plugin": gripper_hardware_plugin,
            }
        )
        .robot_description_semantic(
            file_path=f"config/{robot_model_str}_v6_robot.srdf.xacro",
            mappings={
                "mount": robot_mount,
                "gripper": gripper,
            }
        )
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    kinematics_yaml = load_yaml(moveit_pkg, 'config/kinematics.yaml')

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "publish_robot_description": False,
        "publish_robot_description_semantic": True,
    }

    move_group_parameters = [
        moveit_config.to_dict(),
        {"use_sim_time": use_sim_time},
        planning_scene_monitor_parameters,
    ]
    if kinematics_yaml:
        move_group_parameters.append({"robot_description_kinematics": kinematics_yaml})

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=move_group_parameters,
    )

    env_loader_node = Node(
        package="fairino_universe",
        executable="load_env_to_moveit.py",
        output="screen",
        parameters=[{"env_config_file": env_config}],
    )

    return [move_group_node, TimerAction(period=4.0, actions=[env_loader_node])]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument('use_sim_time', default_value='true',
                               description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument('control_system', default_value="moveit",
                               description="Control system to use"),
        DeclareLaunchArgument('robot_model', default_value="fairino3",
                               description="Robot model to use"),
        DeclareLaunchArgument('robot_mount', default_value="world",
                               description="Robot mount type"),
        DeclareLaunchArgument('gripper', default_value='none',
                               description='Gripper to attach (e.g., dh_ag95, none)'),
        DeclareLaunchArgument('env_config', default_value='env_config.yaml',
                               description='Filename of the environment objects YAML '
                                            'to load into the planning scene'),
        DeclareLaunchArgument('gripper_hardware_plugin', default_value='mock_components/GenericSystem'),
        DeclareLaunchArgument('gripper_hardware_connected', default_value='false'),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])