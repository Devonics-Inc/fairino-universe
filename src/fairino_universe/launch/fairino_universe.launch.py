"""
Fairino robotic arm bring-up launch file.

Parses launch arguments and starts:
  - robot_state_publisher (from an xacro-generated URDF)
  - ros2_control controller_manager + spawners (arm, mount rail, gripper)
  - MoveIt 2 move_group + RViz (optional, --moveit:=true)

  - Ignition Gazebo (optional, --gazebo:=true)

Digital-twin mode: run with hardware:=true and moveit:=true to mirror a
physical Fairino arm in RViz/Gazebo while driving it normally.
"""

import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from fr_universe_launch_utils import *
from fr_universe_launch_actions import * 
from utils_helpers import _truthy,_flatten,_load_config_file


def launch_setup(context, *args, **kwargs):


    # Fetch the resolved launch configuraiton from the context object
    cfg = resolve_config(context)

    # configure and build a custom interface supporting the correct ip address 
    build_configured_interface(cfg.robot_ip_address)


    actions = append_log_actions(cfg)




    # the detault start up delay is 0, however if we aim to run a clean start we set it to 1.5 seconds giving 
    # operating system enough time to klean the processes 
    kill_stale_processes(cfg)


    remaining_actions = []



    # process the robot_description to xml using xacro module
    append_robot_description_actions(cfg, remaining_actions, False)

    controller_manager_parameters = resolve_controller_parameters(cfg)
    append_controller_actions(cfg, actions, controller_manager_parameters)
    

    world_sdf_path = append_gazebo_actions(cfg,remaining_actions)
    append_moveit_actions(cfg, actions, world_sdf_path)



    startup_delay = 1.5 if cfg.clean_start_enabled else 0.0

    if startup_delay > 0.0:
        actions.append(TimerAction(period=startup_delay, actions=remaining_actions))
    else:
        actions.extend(remaining_actions)

    return actions





def generate_launch_description():

    """
    Entry point for the ros2 launch command -- the launch system looks up this
    exact function name after importing the file.

    Runs at parse time, before any launch argument has a value. Its job is to
    supply the default parameter values, read from launch_params.yaml, and to declare each
    argument name. Declaring a name is what later allows the LaunchService to
    override that default with a CLI value. This function cannot see what the user typed.

    Anything that needs a resolved argument value lives in launch_setup(),
    reached via the OpaqueFunction below.

    Returns a LaunchDescription -- an ordered list of actions handed to the
    LaunchService, which visits each in turn.
    """

    # fetch yaml default values from launch_params.yaml as a flat dictionary 
    yaml_launch_params = load_launch_params()   

    # store a string of the packages path combined | used later to set enviroment varaibles
    # before launching gazebo, as gazebo couldn't automatically resolve package::// in xacro 
    pakcages_path = build_resource_path(["fairino_description", "rail_description", "gripper_descriptions"])



    # Build one DeclareLaunchArgument action per configurable option.
    # Nothing is resolved here -- these are inert objects. Later, when the
    # LaunchService visits each one, it writes that name into the context,
    # using the CLI value if the user passed one and yaml_launch_params default otherwise.
    declared_arguments = declare_launch_arguments(yaml_launch_params)


    '''

        A ``LaunchDescription`` is essentially a collection of launch actions,
    which may include:

        ├── Launch arguments
        ├── Environment variables
        ├── Nodes
        ├── Included launch files
        └── Other actions

    '''

    return LaunchDescription(
        [
            # Gazebo is not a ROS program and can't resolve package:// on its
            # own -- it needs the description packages' share dirs on its
            # search path. Set first, so every process spawned below inherits it.
            SetEnvironmentVariable(name="IGN_GAZEBO_RESOURCE_PATH", value=pakcages_path),


            # One DeclareLaunchArgument action for each
            #  configurable option (eg : robot_model, gripper). 
            # the DeclareLaunchArgument action/function allows the launch system 
            # to recognize the argument name and writes its name into the context: 
            # the CLI value if the user passed one, otherwise the default YAML parameters
            *declared_arguments,
            # Function that carries out list of actions 
            OpaqueFunction(function=launch_setup),
        ]
    )