"""Starts the four conveyor nodes. Run this after (or alongside) your move_group launch."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

# EDIT THESE: the same robot name and MoveIt config package your move_group launch uses.
ROBOT_NAME = 'fairino3_v6_robot'
MOVEIT_CONFIG_PKG = 'fairino3_v6_moveit2_config'


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('conveyor_sim'), 'config', 'conveyor.yaml')

    moveit_config = (
        MoveItConfigsBuilder(ROBOT_NAME, package_name=MOVEIT_CONFIG_PKG)
        .robot_description_semantic(file_path='config/fairino3_v6_robot.srdf.xacro')
        .to_moveit_configs()
    )

    def conveyor_node(executable, extra_params=None):
        return Node(
            package='conveyor_sim',
            executable=executable,
            output='screen',
            parameters=[params] + (extra_params or []),
        )

    return LaunchDescription([
        conveyor_node('box_spawner'),
        conveyor_node('box_advancer'),
        conveyor_node('conveyor_manager'),
        # The sensor loads the robot model, so it needs the URDF and SRDF.
        conveyor_node('proximity_sensor', [
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ]),
    ])
