from dataclasses import dataclass
from ament_index_python import get_package_share_directory
import os
from utils_helpers import _truthy
GRIPPER_CONTROLLER_NAME_MAP = {"dh_ag3": "gripper_controller2"}
# ---------------------------------------------------------------------------
# Resolved launch configuration
# ---------------------------------------------------------------------------
MOVEIT_PKG_MAP = {
    "fairino3": "fairino3_v6_moveit2_config",
    "fairino5": "fairino5_v6_moveit2_config",
    "fairino10": "fairino10_v6_moveit2_config",
    "fairino16": "fairino16_v6_moveit2_config",
    "fairino20": "fairino20_v6_moveit2_config",
    "fairino30": "fairino30_v6_moveit2_config",
}


@dataclass(frozen=True)
class FrLaunchConfig:
    """
    Every launch argument, resolved to a plain value.

    Built once at the top of launch_setup() and passed to each action builder,
    so the ~17 .perform() calls happen in exactly one place. Frozen: nothing
    downstream can accidentally reassign a field.

    Fields that stay `str` are the ones handed to xacro, which expects strings
    in its `mappings` dict -- converting them to bool here would break it.
    """

    robot_model: str
    mount: str
    gripper: str
    rail_length: str
    rail_width: str
    robot_ip_address: str
    env_config: str
    base_controller_filename: str
    gripper_controller_filename: str
    gripper_hardware_plugin: str
    gripper_hardware_connected: str   
    listen_only_mode: str            
    moveit_enabled: bool
    gazebo_enabled: bool
    hardware_enabled: bool
    rviz_enabled: bool
    clean_start_enabled: bool

    # -----------------------------------------------------------------------
    # Derived: execution mode
    # -----------------------------------------------------------------------

    @property
    def control_system(self) -> str:
        """
        Which ros2_control setup the xacro should load.

        Priority is hardware > gazebo > mock: if both hardware and gazebo are
        requested, the real robot wins. 'moveit' here means mock hardware
        (mock_components/GenericSystem) and is unrelated to whether move_group
        is launched -- see `moveit_enabled` for that.
        """
        if self.hardware_enabled:
            return "hardware"
        return "gazebo" if self.gazebo_enabled else "moveit"

    @property
    def listen_only_mode_enabled(self) -> bool:
        """Boolean view of listen_only_mode, which is stored as a str for xacro."""
        return _truthy(self.listen_only_mode)

    @property
    def run_sdk_forwarder(self) -> bool:
        """
        Whether ros2_forwarder.py should run.

        Only in hardware + listen-only mode: there the xacro loads
        mock_components/GenericSystem and drops the arm's command_interface, so
        nothing else is talking to the real robot and this script (via the
        vendor SDK) is the only connection. It must never run with
        listen_only_mode=false, where FairinoHardwareInterface owns the
        connection and the two would race to command the arm.
        """
        return self.hardware_enabled and self.listen_only_mode_enabled

    @property
    def standalone_controller_manager(self) -> bool:
        """
        Whether to start our own ros2_control_node.

        False in gazebo mode, where the gz_ros2_control plugin declared in the
        xacro runs a controller_manager inside the Gazebo process. Starting a
        second one duplicates the /controller_manager node name and makes
        controller activation flaky.
        """
        return self.control_system != "gazebo"

    @property
    def is_rail_mounted(self) -> bool:
        """True when the robot sits on a moving mount rather than the world frame."""
        return self.mount != "world"

    @property
    def has_gripper(self) -> bool:
        return self.gripper != "none"

    # -----------------------------------------------------------------------
    # Derived: packages and paths
    # -----------------------------------------------------------------------

    @property
    def moveit_pkg(self) -> str:
        """MoveIt 2 config package for this robot model."""
        return MOVEIT_PKG_MAP.get(
            self.robot_model, f"{self.robot_model}_v6_moveit2_config"
        )

    @property
    def arm_controller_name(self) -> str:
        """ros2_control controller name for the arm, e.g. 'fairino5_controller'."""
        return f"{self.robot_model}_controller"

    @property
    def rail_controller_name(self) -> str:
        """ros2_control controller name for the mount rail."""
        return f"{self.mount}_controller"

    @property
    def gripper_controller_name(self) -> str:
        """
        ros2_control controller name for the gripper.

        Most grippers use the default group; dh_ag3 actuates two joints and
        needs its own.
        """
        return GRIPPER_CONTROLLER_NAME_MAP.get(self.gripper, "gripper_controller")

    @property
    def config_dir(self) -> str:
        """fairino_universe/config/ in the install tree."""
        return os.path.join(
            get_package_share_directory("fairino_universe"), "config"
        )

    @property
    def controllers_yaml_path(self) -> str:
        """ros2_controllers.yaml from the model's MoveIt config package."""
        return os.path.join(
            get_package_share_directory(self.moveit_pkg),
            "config",
            "ros2_controllers.yaml",
        )

    @property
    def rail_yaml_path(self) -> str:
        return os.path.join(self.config_dir, self.base_controller_filename)

    @property
    def gripper_yaml_path(self) -> str:
        return os.path.join(self.config_dir, self.gripper_controller_filename)

    @property
    def xacro_path(self) -> str:
        """Top-level xacro for the robot description."""
        return os.path.join(
            get_package_share_directory("fairino_description"),
            "robots",
            "test_fairino.urdf.xacro",
        )

    @property
    def xacro_mappings(self) -> dict:
        """Arguments handed to xacro.process_file() for the robot description."""
        return {
            "robot_model": self.robot_model,
            "robot_mount": self.mount,
            "control_system": self.control_system,
            "gripper": self.gripper,
            "rail_length": self.rail_length,
            "rail_width": self.rail_width,
            "gripper_hardware_connected": self.gripper_hardware_connected,
            "gripper_hardware_plugin": self.gripper_hardware_plugin,
            "listen_only_mode": self.listen_only_mode,
            "robot_ip_address": self.robot_ip_address,
        }

    def summary(self) -> str:
        """One-line description for the startup LogInfo."""
        return (
            f"Robot model: {self.robot_model} | control system: {self.control_system} | "
            f"gripper: {self.gripper} | mount: {self.mount} | "
            f"moveit_pkg: {self.moveit_pkg}"
        )

