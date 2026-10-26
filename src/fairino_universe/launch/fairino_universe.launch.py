"""
What this file do
Fairino robotic arm bring-up launch file.

Parses launch arguments and starts:
  - robot_state_publisher (from an xacro-generated URDF)
  - ros2_control controller_manager + spawners (arm, mount rail, gripper)
  - MoveIt 2 move_group + RViz (optional, --moveit:=true)
  - Ignition Gazebo (optional, --gazebo:=true)

Digital-twin mode: run with hardware:=true and moveit:=true to mirror a
physical Fairino arm in RViz/Gazebo while driving it normally.
"""


"""
How it do it ? 

After I run `ros2 launch ...`, the ROS 2 launch system loads the launch file as import and looks for `generate_launch_description` 

`generate_launch_description()`.  returns a `LaunchDescription` object containing different launch actions 

that define how the system should be initialized.



One of these actions is `DeclareLaunchArgument`, which, when executed by the launch system,

declares an argument and establishes its default/YAML configuration value in the `LaunchContext`, 

Once the context contains the required configuration values, they can be retrieved when needed by 

creating a `LaunchConfiguration` substitution object and evaluating it using `perform(context)`. 

This resolves the substitution against the context and returns the corresponding value,

which can then be used by other launch actions to configure or execute operations.
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


"""
launch system creates and manages LaunchContext object and it uses the launchActions 
to generate this LaunchContext

"""
def launch_setup(context, *args, **kwargs):


    # Fetch the resolved launch configuraiton from the context object
    cfg = resolve_config(context)

    # configure and build a custom interface supporting the correct ip address 
    build_configured_interface(cfg.robot_ip_address,cfg.rebuild_plugin_enabled)


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
    append_visualization_actions(cfg,actions)



    startup_delay = 1.5 if cfg.clean_start_enabled else 0.0

    if startup_delay > 0.0:
        actions.append(TimerAction(period=startup_delay, actions=remaining_actions))
    else:
        actions.extend(remaining_actions)

    return actions



# descripe work for launch system to do
def generate_launch_description():
    
    """
    This function is responsible for generating the startup plan, represented by a
    LaunchDescription object.

    The startup plan contains a sequence of launch actions that define how the
    system should be initialized. These actions are constructed based on the
    configuration provided by the YAML file.

    The function reads and parses the YAML configuration, then uses the extracted
    values to construct the required launch action objects. It also declares launch
    arguments, making configuration values available through the LaunchContext
    during launch execution.

    Returns:
        LaunchDescription: A description containing the initial launch actions,
        including the declared launch arguments.
    """
    
    # fetch yaml default values from launch_params.yaml as a flat dictionary using YAML library
    yaml_launch_params = load_launch_params()   

    
    # turn each YAML parameter to a LAUNCH Actions 
    declared_arguments = declare_launch_arguments(yaml_launch_params)

    

    # store a string of the packages path combined | used later to set enviroment varaibles
    # before launching gazebo, as gazebo couldn't automatically resolve package::// in xacro 
    pakcages_path = build_resource_path(["fairino_description", "rail_description", "gripper_descriptions"])






    return LaunchDescription(
        [
            # Gazebo is not a ROS program and can't resolve package:// on its own it needs the description packages' share dirs on its
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