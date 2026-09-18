# fairino_universe/launch_utils/paths.py
import os
from ament_index_python.packages import get_package_share_directory
import yaml
import subprocess
import re
import sys
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess, LogInfo
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from launch.substitutions import LaunchConfiguration
from FrLaunchConfig import FrLaunchConfig

from utils_helpers import _truthy,_load_config_file,_is_string_true



def build_resource_path(package_names, existing=None):
    """
    Builds a colon-separated IGN_GAZEBO_RESOURCE_PATH containing the share
    directories of the given packages.

    Any pre-existing value is kept at the front so entries already on the
    path (Gazebo's own models, an overlay workspace) win on name collisions.
    Pass `existing` explicitly in tests; defaults to the real environment.
    """
    if existing is None:
        existing = os.environ.get("IGN_GAZEBO_RESOURCE_PATH")

    shares = [get_package_share_directory(p) for p in package_names]
    entries = ([existing] if existing else []) + shares
    return ":".join(entries)



# Workspace root -- must be defined before anything below that references it.
WORKSPACE_ROOT = os.getcwd()

FAIRINO_HARDWARE_PKG_VERSION = "fairino_hardware_v3_9_5"


FAIRINO_HARDWARE_IP_HEADER = os.path.join(
    WORKSPACE_ROOT, "src", FAIRINO_HARDWARE_PKG_VERSION,
    "include", "fairino_hardware", "data_type_def.h"
)
FAIRINO_HARDWARE_INTERFACE_HEADER = os.path.join(
    WORKSPACE_ROOT, "src", FAIRINO_HARDWARE_PKG_VERSION,
    "include", "fairino_hardware", "fairino_hardware_interface.hpp"
)

REBUILD_PACKAGES = ["fairino_hardware_v3_9_5"]


def prepare_fairino_sdk(robot_ip_address):
    """
    Writes the controller IP into fairino_hardware's headers and rebuilds it.

    The vendor SDK hardcodes the IP as a #define, so changing it means editing
    C++ source and recompiling. Blocks until colcon finishes.
    """
    patch_define_macro(FAIRINO_HARDWARE_IP_HEADER, "CONTROLLER_IP", robot_ip_address)
    patch_define_macro(
        FAIRINO_HARDWARE_INTERFACE_HEADER, "CONTROLLER_IP_ADDRESS", robot_ip_address
    )
    rebuild_packages(WORKSPACE_ROOT, REBUILD_PACKAGES)


def append_log_actions(cfg):
    """
    Append log actions to the actions list to log the current configuration
    that user have chosen 
    """
    actions = [
        LogInfo(msg=f"Robot model: {cfg.robot_model} | control system: {cfg.control_system} | "
                    f"gripper: {cfg.gripper} | moveit_pkg: {cfg.moveit_pkg} ")
    ]

    # append message that decalres whether this is a clean start or not 
    if cfg.clean_start_enabled:
        actions.append(
                LogInfo(msg="[INFO] simulation_clean_start=true: killing leftover Gazebo/ROS 2 processes")
            )
    
    return actions


def kill_stale_processes(cfg):
    """
    Kills leftover Gazebo / ros2_control / MoveIt processes from a previous run.

    A simulation that didn't shut down cleanly leaves processes holding the
    /controller_manager node name, stale models, and duplicate controllers,
    which then interfere with the new launch.

    The '[g]z sim' bracket trick stops pkill matching the shell running this
    command -- without it the shell's own command line contains the pattern and
    it kills itself mid-list. The trailing 'true' keeps the exit code 0 when
    nothing matched.

    Returns a list of actions so callers can splat it unconditionally.
    """
    if not cfg.clean_start_enabled:
        return []

    kill_cmd = (
        "pkill -9 -f '[g]z sim' ; "
        "pkill -9 -f '[i]gn gazebo' ; "
        "pkill -9 -f '[g]zserver' ; "
        "pkill -9 -f '[g]zclient' ; "
        "pkill -9 -f '[r]os2_control_node' ; "
        "pkill -9 -f '[r]obot_state_publisher' ; "
        "pkill -9 -f '[m]ove_group' ; "
        "true"
    )
    return [ExecuteProcess(cmd=["bash", "-c", kill_cmd], output="screen")]

def load_launch_params():
    """
    Loads launch_params.yaml and merges in the rail-geometry configuration 
    via 'rail_geometric_config' parameter, returning one flat dict of defaults.
    """

    config_dir = os.path.join(
        get_package_share_directory("fairino_universe"), "config"
    )

    defaults = _load_config_file(os.path.join(config_dir, "launch_params.yaml"))

    rail_filename = str(defaults.get("rail_geometric_config", "rail_default.yaml"))
    rail = _load_config_file(os.path.join(config_dir, rail_filename))

    return {**defaults, **rail}



def patch_define_macro(header_path, macro_name, value):
    """
    Overwrites a #define <macro_name> "..." line in a header file with the
    given value. Generic across any header/macro pair -- used for both
    data_type_def.h's CONTROLLER_IP and fairino_hardware_interface.hpp's
    CONTROLLER_IP_ADDRESS, driven by the same robot_ip_address argument.

    Matches the line:
        #define <macro_name> "<anything>"
    and replaces only the quoted string -- macro name, spacing, and any
    trailing comment on the line are left untouched.

    Raises RuntimeError if the file doesn't exist or the macro isn't found,
    so a typo'd macro name or moved header fails loudly instead of silently
    building against a stale value.
    """
    if not os.path.isfile(header_path):
        raise RuntimeError(f"[MACRO PATCH] Header not found: {header_path}")

    with open(header_path, "r") as f:
        content = f.read()

    pattern = rf'(#define\s+{re.escape(macro_name)}\s+")[^"]*(")'
    new_content, count = re.subn(pattern, rf'\g<1>{value}\g<2>', content)

    if count == 0:
        raise RuntimeError(
            f"[MACRO PATCH] {macro_name} #define not found in {header_path}. "
            "File format may have changed."
        )

    with open(header_path, "w") as f:
        f.write(new_content)

    print(f"[MACRO PATCH] {header_path}: {macro_name} set to \"{value}\"")


def update_controller_ip(header_path, ip_address):
    """
    Overwrites the CONTROLLER_IP #define in fairino_hardware's
    data_type_def.h with the given IP, before the package is rebuilt.
    Matches the line:
        #define CONTROLLER_IP "192.168.58.2"
    regardless of the current IP value, and replaces only the quoted
    string -- the rest of the line (macro name, spacing, comments) is
    left untouched.
    Raises RuntimeError if the file doesn't exist or the macro isn't
    found, so a typo'd IP or moved header fails loudly instead of
    silently building against a stale address.
    """
    if not os.path.isfile(header_path):
        raise RuntimeError(f"[IP PATCH] Header not found: {header_path}")
    with open(header_path, "r") as f:
        content = f.read()
    pattern = r'(#define\s+CONTROLLER_IP\s+")[^"]*(")'
    new_content, count = re.subn(pattern, rf'\g<1>{ip_address}\g<2>', content)
    if count == 0:
        raise RuntimeError(
            f"[IP PATCH] CONTROLLER_IP #define not found in {header_path}. "
            "File format may have changed."
        )
    with open(header_path, "w") as f:
        f.write(new_content)
    print(f"[IP PATCH] {header_path}: CONTROLLER_IP set to \"{ip_address}\"")


def rebuild_packages(workspace_dir, packages, symlink_install=True):
    cmd = ["colcon", "build", "--packages-select", *packages]
    if symlink_install:
        cmd.append("--symlink-install")
    print(f"[BUILD] Running: {' '.join(cmd)}  (cwd={workspace_dir})")
    result = subprocess.run(cmd, cwd=workspace_dir, capture_output=True, text=True)
    print(result.stdout)
    print(result.stderr)

    if result.returncode != 0:
        raise RuntimeError(f"[BUILD] colcon build failed for {packages}")

    if "ignoring unknown package" in result.stdout or "ignoring unknown package" in result.stderr:
        raise RuntimeError(
            f"[BUILD] colcon did not recognize package(s) {packages} in "
            f"--packages-select. Check the <name> tag in that package's "
            f"package.xml -- it may not match the folder name."
        )

    print(f"[BUILD] Rebuilt successfully: {', '.join(packages)}")



def load_rail_config(filename):
    """
    Loads mount/rail_length/rail_width from a rail-config YAML file.
    Returns {} if missing so hardcoded fallbacks still apply.
    """
    config_path = os.path.join(
        get_package_share_directory("fairino_universe"),
        "config",
        filename,
    )
    try:
        with open(config_path) as f:
            return yaml.safe_load(f) or {}
    except OSError:
        print(f"[INFO] No rail config found at {config_path}, using built-in defaults")
        return {}



# loads yaml file from a package's share directory, returns None if the file doesn't exist
def load_yaml(package_name, relative_path):
    """
    Loads a YAML file from a ROS package's share directory.

    Args:
        package_name (str): Name of the ROS package.
        relative_path (str): Relative path to the YAML file within the package's share directory.

    Returns:
        dict | list | None: Parsed YAML content if the file is successfully loaded,
        otherwise None if the file cannot be found or opened.
    """
    
    absolute_path = os.path.join(get_package_share_directory(package_name), relative_path)
    try:
        with open(absolute_path) as f:
            return yaml.safe_load(f)
    except OSError:
        return None






def declare_launch_arguments(launch_params):
        # Maps a robot_model argument to its corresponding MoveIt 2 config package.
        # Centralised here so every part of the launch file agrees on the package name
        MOVEIT_PKG_MAP = {
            "fairino3": "fairino3_v6_moveit2_config",
            "fairino5": "fairino5_v6_moveit2_config",
            "fairino10": "fairino10_v6_moveit2_config",
            "fairino16": "fairino16_v6_moveit2_config",
            "fairino20": "fairino20_v6_moveit2_config",
            "fairino30": "fairino30_v6_moveit2_config",
        }

        # each DeclareLaunchArgument command is mapping the parameter name
        # name in the context
        # use the CLI value if there is one for this name, 
        # otherwise my default comes from YAML confiugration 

        declared_arguments = [
        DeclareLaunchArgument(
            "robot_model",
            default_value=str(launch_params.get("robot_model", "fairino5")),
            description="Name of robot model to spawn (e.g. fairino5)",
            choices=list(MOVEIT_PKG_MAP.keys()),
        ),
        DeclareLaunchArgument(
            "gripper",
            default_value=str(launch_params.get("gripper", "none")),
            description="Gripper to attach (e.g. dh_ag95, none)",
        ),
        DeclareLaunchArgument(
            "moveit",
            default_value=str(launch_params.get("moveit", "false")),
            description="Set to true to launch the MoveIt 2 move_group + RViz",
        ),
        DeclareLaunchArgument(
            "gazebo_simulated_hardware",
            default_value=str(launch_params.get("gazebo_simulated_hardware", "false")),
            description="Set to true to launch Ignition Gazebo with the given world",
        ),
        DeclareLaunchArgument(
            "robot_hardware_connected",
            default_value=str(launch_params.get("robot_hardware_connected", "false")),
            description="Set to true to use the real hardware controller",
        ),
        DeclareLaunchArgument(
            "simulation_clean_start",
            default_value=str(launch_params.get("simulation_clean_start", "false")),
            description="Set to true to kill any leftover Gazebo/ros2_control processes "
                         "from a previous run before launching",
        ),
        DeclareLaunchArgument(
            "rail_width",
            default_value=str(launch_params.get("rail_width", "0.2")),
            description="Width of the mount rail (meters)",
        ),
        DeclareLaunchArgument(
            "rail_length",
            default_value=str(launch_params.get("rail_length", "0.2")),
            description="Length of the mount rail (meters)",
        ),
        DeclareLaunchArgument(
            "env_config",
            default_value=str(launch_params.get("env_config", "env_config.yaml")),
            description="Filename (in fairino_universe/config/) of the environment objects "
                         "to load into the MoveIt planning scene. Leave unset/empty to skip "
                         "loading any environment.",
        ),
        DeclareLaunchArgument(
            "mount",
            default_value=str(launch_params.get("mount", "world")),
            description="Object to mount the robot to (e.g. world or rail_carriage)",
        ),
        DeclareLaunchArgument(
            "rviz_enabled",
            default_value=str(launch_params.get("rviz_enabled", "true")),
            description="Set to true to launch RViz alongside MoveIt",
        ),
        DeclareLaunchArgument(
            "gripper_hardware_connected",
            default_value=str(launch_params.get("gripper_hardware_connected", "false")),
            description="Set to true to use the real gripper hardware interface instead of mock",
        ),
        DeclareLaunchArgument(
            "rail_controller",
            default_value=str(launch_params.get("rail_controller", "base_config.yaml")),
            description="Filename (in fairino_universe/config/) of the rail/mount controller yaml",
        ),
        DeclareLaunchArgument(
            "gripper_controller",
            default_value=str(launch_params.get("gripper_controller", "gripper_config.yaml")),
            description="Filename (in fairino_universe/config/) of the gripper controller yaml",
        ),
        DeclareLaunchArgument(
            "gripper_hardware_plugin",
            default_value=str(launch_params.get("gripper_hardware_plugin", "mock_components/GenericSystem")),
            description="ros2_control hardware plugin name to use for the gripper "
                         "when gripper_hardware_connected is true",
        ),
        DeclareLaunchArgument(
            "listen_only_mode",
            default_value=str(launch_params.get("listen_only_mode", "true")),
            description="If true, ros2_control interfaces are read-only (no command_interface) "
                        "for the arm joints — mirrors a real/simulated arm without driving it.",
        ),
        DeclareLaunchArgument(
            "robot_ip_address",
            default_value=str(launch_params.get("robot_ip_address", "192.168.58.2")),
            description="IP address of the physical Fairino arm controller "
                        "(used when robot_hardware_connected is true).",
        ),
        ]

        return declared_arguments
    

def build_configured_interface(robot_ip_address):
    """
    Writes the controller IP into fairino_hardware's headers and rebuilds it.

    The vendor SDK hardcodes the controller address as a #define in two
    headers, so pointing at a different robot means editing C++ source and
    recompiling. Blocks until colcon finishes.
    """
    patch_define_macro(FAIRINO_HARDWARE_IP_HEADER, "CONTROLLER_IP", robot_ip_address)
    patch_define_macro(
        FAIRINO_HARDWARE_INTERFACE_HEADER, "CONTROLLER_IP_ADDRESS", robot_ip_address
    )
    rebuild_packages(WORKSPACE_ROOT, REBUILD_PACKAGES)


def resolve_config(context) -> FrLaunchConfig:
    """
    Resolves every declared launch argument against the LaunchContext.

    Each LaunchConfiguration is a deferred lookup; .perform(context) returns
    the value the DeclareLaunchArgument actions wrote into the context -- the
    CLI value if the user passed one, otherwise the YAML/literal default.

    Raises if an argument was never declared, which is the usual symptom of
    deleting a DeclareLaunchArgument but leaving its use in place.
    """
    def s(name):
        return LaunchConfiguration(name).perform(context)

    return FrLaunchConfig(
        robot_model=s("robot_model"),
        mount=s("mount"),
        gripper=s("gripper"),
        rail_length=s("rail_length"),
        rail_width=s("rail_width"),
        robot_ip_address=s("robot_ip_address"),
        env_config=s("env_config"),
        base_controller_filename=s("rail_controller"),
        gripper_controller_filename=s("gripper_controller"),
        gripper_hardware_plugin=s("gripper_hardware_plugin"),
        gripper_hardware_connected=s("gripper_hardware_connected"),
        listen_only_mode=s("listen_only_mode"),
        moveit_enabled=_truthy(s("moveit")),
        gazebo_enabled=_truthy(s("gazebo_simulated_hardware")),
        hardware_enabled=_truthy(s("robot_hardware_connected")),
        rviz_enabled=_truthy(s("rviz_enabled")),
        clean_start_enabled=_truthy(s("simulation_clean_start")),
    )