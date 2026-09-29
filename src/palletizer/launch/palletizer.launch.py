import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    xacro_args = {"gripper": "none"}
    moveit_config = (
        MoveItConfigsBuilder("fairino5_v6_robot", package_name="fairino5_v6_moveit2_config")
        .robot_description(mappings=xacro_args)
        .robot_description_semantic(mappings=xacro_args)
        .to_moveit_configs()
    )

    params = os.path.join(
        get_package_share_directory("palletizer"), "config", "palletizer.yaml")

    planner = Node(
        package="palletizer",
        executable="pallet_planner",
        output="screen",
        parameters=[params],
    )

    executor = Node(
        package="palletizer",
        executable="palletizer_node",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            params,
        ],
    )

    return LaunchDescription([planner, executor])
