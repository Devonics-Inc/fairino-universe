# Overview

# This launch file serves as the master orchestrator for a Fairino robotic arm setup. Its primary job is to parse your robot's configuration data
# and boot up multiple software components simultaneously based on your command-line arguments

## components 

# command - line input parser 

# This part is responsible for parsing argument from the cli before sending these commands to other tools such as 
#     #xacro processor
#     #robot state publisher 
#     #controller manager 


# robot state publisher 

# controller manager

import os
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription, LaunchContext
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    ExecuteProcess,
    TimerAction,
    LogInfo
)
from launch_ros.actions import Node
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

import tempfile
import yaml

from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
    PythonExpression
)
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import LaunchConfigurationEquals, LaunchConfigurationNotEquals, IfCondition
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, TextSubstitution, LaunchConfiguration
import xacro
import yaml
import sys
import os


world_file_path = os.path.join(
    get_package_share_directory('fairino_gazebo_config'),
    'worlds',
    'empty.sdf'
)


"""
THIS CREATES A DIGITAL FAIRINO, THAT MIRRORS THE ROBOT AT THE IP ADDRESS SET IN /rt_state_data

USE NORMAL CONTROL FOR YOUR ROBOT AND THE GAZEBO BOT WILL FOLLOW

"""

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path) as file:
            return yaml.safe_load(file)
    except OSError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    pkg_share = get_package_share_directory('fairino_description')
    rail_pkg_share = get_package_share_directory('rail_description')
    if('IGN_GAZEBO_RESOURCE_PATH' in os.environ):
        gazebo_resource_path = os.environ['IGN_GAZEBO_RESOURCE_PATH'] + ':' + pkg_share + ':' + rail_pkg_share
    else:
        gazebo_resource_path = pkg_share + ':' + rail_pkg_share

    ####################
    # launch arguments #
    ####################
    world = LaunchConfiguration('world')
    world_arg = DeclareLaunchArgument(
        'world',
        default_value="empty",
        description="Name of world file to spawn robot into"
    )
    # Declare robot model
    robot_model = LaunchConfiguration('robot_model')
    robot_model_arg = DeclareLaunchArgument(
        'robot_model',
        default_value="fairino5",
        description="Name of robot model to spawn (ie. Fairino3)",
        choices=[
            'fairino3',
            'fairino5',
            'fairino10',
            'fairino16',
            'fairino20',
            'fairino30',
        ]
    )
    # Declare robot mount
    mount = LaunchConfiguration('mount')
    mount_arg = DeclareLaunchArgument(
        'mount',
        default_value="world",
        description="Name of object to mount robot to (ie. world or rail_carriage)"
    )
# === FIX 1: DECLARE GRIPPER ARGUMENT BEFORE PASSING IT TO THE XACRO ===
    gripper = LaunchConfiguration('gripper')
    gripper_arg = DeclareLaunchArgument(
        'gripper',
        default_value="none",  # This allows the command line to omit it safely
        description="Name of the gripper to attach (e.g., dh_ag95, none)"
    )

    moveit = LaunchConfiguration('moveit')
    moveit_arg = DeclareLaunchArgument(
        'moveit',
        default_value="false",
        description="Set to true to use moveit controller and obscicle porting from gazebo"
    )
    
    useGazebo = LaunchConfiguration('gazebo')
    gazebo_arg = DeclareLaunchArgument(
        'gazebo',
        default_value="false",
        description="Set to true to use gazebo controller and spawn the passed world"
    )    

    useHardware = LaunchConfiguration('hardware')
    hardware_arg = DeclareLaunchArgument(
        'hardware',
        default_value="false",
        description="Set to true to use load hardware controller"
    )    


    moveit_pkg_map = {
        "fairino3":  "fairino3_v6_moveit2_config",
        "fairino5":  "fairino5_v6_moveit2_config",
        "fairino10": "fairino10_v6_moveit2_config",
        "fairino16": "fairino16_v6_moveit2_config",
        "fairino20": "fairino20_v6_moveit2_config",
        "fairino30": "fairino30_v6_moveit2_config",
    }

    control_system_str = "moveit"  # default
    robot_model_str = "fairino5"  # default
    moveit_pkg = "fairino5_v6_moveit2_config"
    gripper_str = "none" # <-- Add default string tracker
    for arg in sys.argv:
        if arg.startswith("robot_model:="):
            robot_model_str = arg.split(":=")[1]
            moveit_pkg = moveit_pkg_map.get(robot_model_str, f"{robot_model_str}_v6_moveit2_config")
            print("\n\n\n\nUSING " , robot_model_str, "\n\n\n")
        elif arg.startswith("gripper:="):
            gripper_str = arg.split(":=")[1]

        elif arg.startswith("hardware:="):
            control_system_str = arg.split(":=")[1]
            if("TRUE" in control_system_str.upper()):
                control_system_str = "hardware"
               
        elif arg.startswith("gazebo:="):
            if(control_system_str != "hardware"):
                control_system_str = arg.split(":=")[1]
                if("TRUE" in control_system_str.upper()):
                    control_system_str = "gazebo"
        
            else:
                print("\n\n[INFO] Both 'gazebo' and 'hardware' passed into launch file; using hardware")
    print("\n\n\n\nUSING CONTROL SYSTEM: ", control_system_str, "\n\n\n")



# 1. Start with ParameterValue(
    robot_description = ParameterValue(
        Command([
            FindExecutable(name='xacro'),
            ' ',
            PathJoinSubstitution([
                FindPackageShare('fairino_description'),
                'robots',
                "test_fairino.urdf.xacro"
            ]),
            ' ',
            "robot_model:=", robot_model_str,
            ' ',
            "robot_mount:=", LaunchConfiguration('mount'),
            ' ',
            "control_system:=", control_system_str,
            ' ',
            "gripper:=", LaunchConfiguration('gripper'),  # Corrected syntax and formatting
        ]),  # Closed the Command bracket cleanly
        value_type=str  # Assigned value_type to the outer ParameterValue container
    )

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        respawn=True,
        output="screen",
        parameters=[{"robot_description": robot_description}, {"use_sim_time":False}],
    )

    ##########################################################
    # ------------------------
    # Gazebo
    # ------------------------
    # Create an instance of Gazebo
    

    # Load controllers into controller manager
    controllers_yaml = load_yaml(moveit_pkg, "config/ros2_controllers.yaml")
    controllers_yaml_path = os.path.join(
        get_package_share_directory(moveit_pkg),
        "config",
        "ros2_controllers.yaml"
    )

    gripper_yaml_path = os.path.join(
        get_package_share_directory(moveit_pkg),
        "config",
        "gripper_controllers.yaml"
    )
    controller_manager_parameters = [
        {'use_sim_time': False},
        controllers_yaml_path,
    ]

    # Only load gripper controller params if a gripper was actually requested,
    # and only if the file actually exists for this robot package.
    if gripper_str != "none" and os.path.exists(gripper_yaml_path):
        controller_manager_parameters.append(gripper_yaml_path)
    else:
        print(f"\n[INFO] Skipping gripper_controllers.yaml (gripper='{gripper_str}')\n")

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=controller_manager_parameters,
        remappings=[
            ("/controller_manager/robot_description", "/robot_description")
        ],
        output='screen',
    )

    # Spawn the joint_state_broadcaster for the gazebo robot
    joint_state_broadcaster = TimerAction(
        period=1.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "joint_state_broadcaster",
                    "-c", "/controller_manager",
                    "--param-file", controllers_yaml_path,  # ← add this
                ],
                output="screen",
            )
        ]
    )


    # -------------------- MOVEIT 2 CONTROLLER --------------------

    # Move Group parameters for moveit control - NOW WITH KINEMATICS CONFIG
    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("fairino_gazebo_config"),
                'launch',
                'move_group.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': str(control_system_str == "gazebo"),
            'robot_model': robot_model_str,
            'robot_mount': LaunchConfiguration('mount'),
            'control_system': control_system_str,
            'moveit_pkg': moveit_pkg,
            'gripper': LaunchConfiguration('gripper'),
        
        }.items(),
        condition=IfCondition(LaunchConfiguration('moveit'))
    )

    static_tfs = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare(PythonExpression([
                    "'", LaunchConfiguration('robot_model'), "_v6_moveit2_config'"
                ])),
                'launch',
                'static_virtual_joint_tfs.launch.py'
            ])
        ]),
        condition=LaunchConfigurationNotEquals('gazebo', 'true')
    )

    kinematics_yaml = load_yaml(moveit_pkg, "config/kinematics.yaml")
    rviz = Node(
            package="rviz2",
            executable="rviz2",
            arguments=['-d', PathJoinSubstitution([
                FindPackageShare(PythonExpression([
                    "'", LaunchConfiguration('robot_model'), "_v6_moveit2_config'"
                ])),
                'config',
                'moveit.rviz'
            ])],
            condition=IfCondition(LaunchConfiguration('moveit')),
            parameters=[
                {"use_sim_time": False},
                {"robot_description_kinematics": kinematics_yaml} # <--- This fixes the 3D point!
            ]
        )


    # Grab controllers to load
    fairino_controller_name = [PythonExpression([
        "'", LaunchConfiguration('robot_model'), "_controller'"
    ])]
    
    mount_controller = [PythonExpression([
        "'", LaunchConfiguration('mount'), "_controller'"
    ])]

    gripper_controller_name = [PythonExpression([
        "'", LaunchConfiguration('gripper'), "_controller'"
    ])]

    
##  configure Moveit Controllers  (High level)
##  this is responsible for generating the movegroup such as (arm / gripper) 
##  input (fairino_controller_name) / controllers_yaml_path 


    fairino_controller = TimerAction(
        period=1.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    fairino_controller_name,
                    "-c", "/controller_manager",
                    "--controller-manager-timeout", "10",
                    "--param-file", controllers_yaml_path,  # ← add this
                ],
                output="screen",
            ),
        ],
    )

    rail_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[mount_controller,
                   "-c", "/controller_manager",
                   "-t", "joint_trajectory_controller/JointTrajectoryController"
        ],
        condition=IfCondition(PythonExpression([
            "'",
            LaunchConfiguration('mount'),
            "' != 'world'",
        ])),

        output="screen",
    )

    # # === NEW: Pass gripper controller into controller spawner ===
    # gripper_controller = TimerAction(
    #     period=1.5,  # Staggered slightly after the robot controller
    #     actions=[
    #         Node(
    #             package="controller_manager",
    #             executable="spawner",
    #             arguments=[
    #                 gripper_controller_name,
    #                 "-c", "/controller_manager",
    #                 "--controller-manager-timeout", "10",
    #                 "--param-file", controllers_yaml_path,
    #             ],
    #             # Only run this if gripper argument is passed and NOT equal to 'none'
    #             condition=IfCondition(PythonExpression([
    #                 "'", LaunchConfiguration('gripper'), "' != 'none'"
    #             ])),
    #             output="screen",
    #         ),
    #     ],
    # )

    # Spawn gazebo
    world_string = PythonExpression([
        "'src/fairino_gazebo_config/worlds/", LaunchConfiguration('world'), ".sdf'"
    ])
    gazebo = TimerAction(
        period=2.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
                ]),
                launch_arguments={
                    'gz_args': [world_string, ' -r']

                }.items(),
                condition=LaunchConfigurationEquals('gazebo', 'true')
            )
        ]
    )

    # Create robot in robot from robot_description
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=['-topic', 'robot_description'], # To pass spawn location args: ['-x', '0.0', '-y','0.0',  '-z','0.0',  '-R','0.0',  '-P', '0.0', '-Y','0.0'
        condition=LaunchConfigurationEquals('gazebo', 'true'),
    )

    # World file -> MoveIt collision parser
    moveit_obs_gen = Node(
        package="fairino_gazebo_config",
        executable="gazebo_world_to_moveit.py",
        arguments=[world_string],
        condition=IfCondition(LaunchConfiguration('moveit')),
        parameters=[{"use_sim_time": False}]
    )




    def launch_dynamic_gripper_controller(context, *args, **kwargs):
        gripper_name = LaunchConfiguration('gripper').perform(context)
        
        gripper_joint_mapping = {
            'dh_ag95': ['dh_ag95_left_outer_knuckle_joint'], # Ensure this matches your actual URDF name
            'robotiq_2f_85': ['robotiq_2f_85_finger_joint'],
            'schunk_emh': ['schunk_emh_grip_joint'],
        }
        
        if gripper_name == 'none' or gripper_name not in gripper_joint_mapping:
            return []

        active_joints = gripper_joint_mapping[gripper_name]
        
        # Generate a temporary parameters configuration file dynamically
        # This guarantees the backend parser reads it strictly as a YAML array sequence
        param_dict = {
            "gripper_controller": {
                "ros__parameters": {
                    "joints": active_joints
                }
            }
        }
        
        tmp_param_file = tempfile.NamedTemporaryFile(mode='w', delete=False, suffix='.yaml')
        yaml.dump(param_dict, tmp_param_file)
        tmp_param_file.close()
        
        gripper_name = LaunchConfiguration('gripper').perform(context)
        
        if gripper_name == 'none':
            return []
            
        # Dynamically map straight to your clean, static standalone gripper config file
        gripper_yaml_file = os.path.join(
            get_package_share_directory(moveit_pkg),
            "config",
            "gripper_controllers.yaml"
        )
        
        gripper_controller_spawner = Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "gripper_controller", 
                "--param-file", gripper_yaml_file,
                "-c", "/controller_manager"
            ],
            output="screen"
        )
        
        return [gripper_controller_spawner]

            
    return LaunchDescription([
        SetEnvironmentVariable(name='IGN_GAZEBO_RESOURCE_PATH', value=gazebo_resource_path),
        world_arg,
        mount_arg,
        robot_model_arg,
        gazebo_arg,
        gripper_arg,
        moveit_arg,
        hardware_arg,
        rsp,
        spawn_robot,
        # static_tfs,
        OpaqueFunction(function=launch_dynamic_gripper_controller),
        controller_manager,
        joint_state_broadcaster,
        fairino_controller,
        rail_controller,
        gazebo,
        rviz,
        move_group,
        moveit_obs_gen,

    ])
