# FAIRINO Universe

FAIRINO Universe is an open-source simulation framework designed to accelerate automation projects built around FAIRINO robotic arms.

Built on top of ROS 2, MoveIt 2, and Gazebo, the framework provides a ready-to-use simulation environment, allowing engineers to focus on developing their application instead of spending time configuring robot, gripper, and environment URDFs from scratch.

It works by providing the user with a set of YAML configuration files. These files are parsed and used to automatically configure the simulation environment, enabling support for a wide range of automation applications.

![Demo](docs/videos/FR5EX.webp)

## Features

- **Modular Configuration**  
  Configure robots, grippers, linear rails, controllers, and simulation environments through reusable YAML files.

- **Custom Workcell Generation**  
  Build complete robotic workcells by combining predefined mounts, CAD models, and primitive objects.

- **Configurable End Effectors**  
  Easily integrate and configure different grippers and hardware interfaces.

- **Linear Rail Support**  
  Configure external linear axes, including geometry and controller parameters.

- **Listen-Only Mode**  
  Connect to an existing robot or simulator for visualization and monitoring without sending motion commands.

- **Custom CAD Assets**  
  Import STL models to create realistic simulation environments and industrial workcells.



## Installation

Clone the repository:

```bash
git clone https://github.com/Devonics-Inc/fairino-universe
cd fairino_universe
```

Build the workspace:

```bash
colcon build --symlink-install
```

Source the workspace:

```bash
source install/setup.bash
```

## Running FAIRINO Universe

Launch the simulation:

```bash
ros2 launch fairino_universe fairino_universe.launch.py
```

## Configuration

FAIRINO Universe is configured using a collection of YAML files that describe the robot, peripherals, controllers, and simulation environment.

The main configuration file is:

```text
config/launch_params.yaml
```

This file acts as the master configuration for the framework. It references additional configuration files that define the robot, linear rail, gripper, controllers, and environment. By modifying these files, different automation cells can be created without changing the launch code.

Example:

```yaml
robot:
  robot_model: fairino5
  robot_hardware_connected: "true"
  robot_ip_address: "192.168.58.2"

simulation:
  gazebo_simulated_hardware: "false"
  simulation_clean_start: "true"
  moveit: "true"
  rviz_enabled: "true"
  listen_only_mode: "true"

rail:
  rail_controller: test_rail_controllers2.yaml
  base_hardware_connected: "false"
  rail_geometric_config: rail_test.yaml

gripper:
  gripper: dh_ag3
  gripper_controller: gripper_controllers.yaml
  gripper_hardware_connected: "false"
  gripper_hardware_plugin: "fairino_hardware/GripperHardwareInterface_NOT_IMPLEMENTED"
```

### Robot

| Parameter | Description |
|-----------|-------------|
| `robot_model` | Selects the FAIRINO robot model to load (for example `fairino5`, `fairino10`, or `fairino20`). |
| `robot_hardware_connected` | Indicates whether a physical robot is connected. Set to `false` when running only in simulation. |
| `robot_ip_address` | IP address of the robot controller. This parameter is ignored when no hardware is connected. |

### Simulation

| Parameter | Description |
|-----------|-------------|
| `gazebo_simulated_hardware` | Uses Gazebo's simulated hardware interfaces instead of connecting to physical hardware. |
| `simulation_clean_start` | Terminates any existing Gazebo instances before launching a new simulation. |
| `moveit` | Launches the MoveIt 2 motion planning stack. |
| `rviz_enabled` | Starts RViz with the configured visualization. |
| `listen_only_mode` | Connects to an existing robot or simulator without sending motion commands. This mode is useful for visualization, monitoring, and debugging. A robot or simulator must already be running. |

### Rail

| Parameter | Description |
|-----------|-------------|
| `rail_controller` | Controller configuration file for the linear rail. |
| `base_hardware_connected` | Indicates whether a physical linear rail is connected. |
| `rail_geometric_config` | Geometry configuration describing the rail dimensions, mounting location, and kinematic properties. |

### Gripper

| Parameter | Description |
|-----------|-------------|
| `gripper` | Selects the gripper model to load. Supported values include `dh_ag95`, `dh_ag145`, `dh_pgc140`, and `dh_ag3`. |
| `gripper_controller` | Controller configuration for the selected gripper. |
| `gripper_hardware_connected` | Indicates whether a physical gripper is connected. |
| `gripper_hardware_plugin` | ROS 2 hardware interface plugin used to communicate with the gripper hardware. |

## Environment Configuration

The simulation environment is described using a dedicated YAML file referenced by the `environment.env_config` parameter in `launch_params.yaml`.

This file defines the robot mounting configuration, optional linear rail dimensions, and all objects that should appear in the simulation.

Example:

```yaml
# Robot mounting configuration
# Available options:
#   test_rail
#   sync_table
#   sync_table2
mount: sync_table2

# Linear rail dimensions (only used when a rail is present)
rail_length: 2.0
rail_width: 0.1

primitives:
  box:
    modular_table_leg:
      pose: [0.0, -1.0, 0.0, 0, 0, 0]
      size: [0.05, 0.05, 0.75]

  cylinder:
    pillar_1:
      pose: [-0.5, -0.7, 0.0, 0, 0, 0]
      radius: 0.1
      length: 0.5

stl:
  tower:
    pose: [0.5, 0.0, 0.0, 0, 0, 0]
    mesh: "package://fairino_universe/meshes/Tower.STL"
    scale: [0.001, 0.001, 0.001]

  table:
    pose: [0.5, 0.0, 0.0, 0, 0, 0]
    mesh: "package://fairino_universe/meshes/Table_assembly.STL"
    scale: [0.001, 0.001, 0.001]

  milling_machine:
    pose: [1.2, -0.5, 0.0, 0, 0, 0]
    mesh: "package://fairino_universe/meshes/Milling_machine.STL"
    scale: [0.001, 0.001, 0.001]
```

### Mount Configuration

The `mount` parameter selects the predefined robot workcell to load.

| Parameter | Description |
|-----------|-------------|
| `mount` | Selects the robot mounting configuration. Supported values include `test_rail`, `sync_table`, and `sync_table2`. |

### Rail Dimensions

These parameters are only used when the selected mounting configuration includes a linear rail.

| Parameter | Description |
|-----------|-------------|
| `rail_length` | Length of the linear rail in meters. |
| `rail_width` | Width of the linear rail in meters. |

### Primitive Objects

Primitive objects can be added directly to the simulation without requiring CAD models.

Currently supported primitive types include:

- `box`
- `cylinder`

Each primitive has a unique name and defines its geometry and pose.

#### Box

```yaml
box:
  table_leg:
    pose: [0.0, -1.0, 0.0, 0, 0, 0]
    size: [0.05, 0.05, 0.75]
```

| Parameter | Description |
|-----------|-------------|
| `pose` | Object pose as `[x, y, z, roll, pitch, yaw]` in meters and radians. |
| `size` | Box dimensions `[x, y, z]` in meters. |

#### Cylinder

```yaml
cylinder:
  pillar:
    pose: [-0.5, -0.7, 0.0, 0, 0, 0]
    radius: 0.1
    length: 0.5
```

| Parameter | Description |
|-----------|-------------|
| `pose` | Object pose as `[x, y, z, roll, pitch, yaw]`. |
| `radius` | Cylinder radius in meters. |
| `length` | Cylinder height in meters. |

### STL Meshes

Custom CAD models can be added using STL files.

```yaml
stl:
  milling_machine:
    pose: [1.2, -0.5, 0.0, 0, 0, 0]
    mesh: "package://fairino_universe/meshes/Milling_machine.STL"
    scale: [0.001, 0.001, 0.001]
```

| Parameter | Description |
|-----------|-------------|
| `pose` | Mesh pose as `[x, y, z, roll, pitch, yaw]`. |
| `mesh` | Path to the STL file using a ROS package URI. |
| `scale` | Scaling factor applied independently to the X, Y, and Z axes. |

## Coordinate System

All object poses are specified relative to the world coordinate frame.

```text
pose: [x, y, z, roll, pitch, yaw]
```

where:

- `x`, `y`, and `z` are specified in meters.
- `roll`, `pitch`, and `yaw` are specified in radians.

Using this environment configuration, complete robotic workcells—including tables, fixtures, machines, tooling, and other equipment—can be assembled simply by editing YAML files, eliminating the need to modify source code or URDF files.

## Configuration Philosophy

FAIRINO Universe follows a modular configuration approach. Instead of hardcoding the simulation setup, the framework builds the robot cell by combining multiple configuration files. This makes it easy to switch between different robots, grippers, linear rails, controllers, and environments simply by changing the YAML configuration files, without modifying any source code.