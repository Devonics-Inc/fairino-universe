# Fairino Universe Launch File

Single entry point for bringing up a Fairino robotic arm in simulation, visualization, or as a live "digital twin" of a physical robot.

`fairino_universe.launch.py` is responsible for:

- Generating the robot's URDF from xacro based on the selected model, mount, control system, and gripper
- Starting `robot_state_publisher`
- Starting `ros2_control_node` and spawning the appropriate controllers (arm, mount rail, gripper)
- Optionally starting Ignition Gazebo and spawning the robot into a world
- Optionally starting MoveIt 2 (`move_group`) and RViz

It supports three mutually-aware **control system** modes — `moveit`, `gazebo`, and `hardware` — selected via launch arguments, plus an optional pluggable **gripper**.


```mermaid
sequenceDiagram
    sequenceDiagram
    autonumber
    actor User
    participant LF as fairino_universe.launch.py
    participant Xacro as xacro.process_file
    participant RSP as robot_state_publisher
    participant CM as ros2_control_node
    participant SP1 as spawner_joint_state_broadcaster
    participant SP2 as spawner_arm_controller
    participant SP3 as spawner_mount_controller
    participant SP4 as spawner_gripper_controller
    participant GZ as Ignition_Gazebo
    participant MG as move_group_launch
    participant RV as RViz2
    participant W2M as gazebo_world_to_moveit_py

    User->>LF: ros2 launch with robot_model, mount, gripper, moveit, gazebo, hardware, world

    Note over LF: generate_launch_description
    LF->>LF: Set IGN_GAZEBO_RESOURCE_PATH
    LF->>LF: Declare launch arguments
    LF->>LF: OpaqueFunction calls launch_setup

    Note over LF: launch_setup - resolve all LaunchConfiguration values to strings

    rect rgb(245,245,245)
    Note over LF: Resolve control_system
    alt hardware is true
        LF->>LF: control_system equals hardware
        opt gazebo is also true
            LF->>LF: log warning both requested using hardware
        end
    else gazebo is true and hardware is false
        LF->>LF: control_system equals gazebo
    else neither flag set
        LF->>LF: control_system equals moveit default
    end
    end

    LF->>LF: resolve moveit_pkg from MOVEIT_PKG_MAP
    LF->>LF: LogInfo robot_model control_system gripper moveit_pkg

    rect rgb(245,245,245)
    Note over LF,Xacro: Robot description generation
    LF->>Xacro: process_file test_fairino urdf xacro with mappings
    Xacro-->>LF: return robot_description_xml
    end

    LF->>RSP: launch Node robot_state_publisher
    RSP-->>RSP: parse URDF publish robot_description and tf

    rect rgb(245,245,245)
    Note over LF,CM: Controller manager bring-up
    LF->>LF: build controller_manager_parameters list
    alt gripper not none and gripper yaml exists
        LF->>LF: append gripper_controllers yaml to params
    else
        LF->>LF: log info skipping gripper controllers yaml
    end
    LF->>CM: launch Node ros2_control_node with parameters
    CM-->>CM: load hardware interface
    CM-->>CM: configure and activate hardware
    end

    rect rgb(245,245,245)
    Note over LF,SP2: Spawners after timer delay of one second
    LF->>SP1: spawn joint_state_broadcaster
    SP1->>CM: load configure activate
    CM-->>SP1: activated
    SP1-->>LF: process finished cleanly

    LF->>SP2: spawn arm controller with timeout ten seconds
    SP2->>CM: load configure activate
    CM-->>SP2: activated
    SP2-->>LF: process finished cleanly
    end

    opt mount is not world
        LF->>SP3: spawn mount controller as joint trajectory controller
        SP3->>CM: load configure activate
        CM-->>SP3: activated
    end

    rect rgb(245,245,245)
    Note over LF,SP4: Gripper controller spawn is conditional
    alt gripper equals none
        LF->>LF: skip gripper controller entirely
    else gripper not recognized
        LF->>LF: log warning unknown gripper skipping spawn
    else gripper yaml missing
        LF->>LF: log warning yaml not found skipping spawn
    else all checks pass
        LF->>LF: resolve active_controller_name from map
        LF->>SP4: spawn active_controller_name with gripper yaml
        SP4->>CM: load configure activate
        CM-->>SP4: activated
        SP4-->>LF: process finished cleanly
    end
    end

    rect rgb(245,245,245)
    Note over LF,GZ: Gazebo runs only if gazebo flag is true
    opt gazebo is true
        LF->>LF: resolve world_sdf_path
        LF->>GZ: timer two seconds then include gz_sim launch
        GZ-->>GZ: start simulation load world
        LF->>GZ: spawn ros_gz_sim create node
        GZ-->>GZ: spawn robot model from robot_description topic
    end
    end

    rect rgb(245,245,245)
    Note over LF,W2M: MoveIt runs only if moveit flag is true
    opt moveit is true
        LF->>MG: include move_group launch with robot_model mount control_system moveit_pkg gripper
        MG-->>MG: load SRDF moveit_controllers yaml kinematics yaml
        MG-->>MG: start move_group action and service servers

        LF->>RV: spawn rviz2 node with moveit rviz config
        RV-->>RV: connect to move_group load planning groups

        LF->>W2M: spawn gazebo_world_to_moveit node with world path
        W2M->>W2M: parse world sdf
        alt world file resolves correctly
            W2M-->>MG: publish collision objects to planning scene
        else path unresolved known issue
            W2M--xW2M: FileNotFoundError process dies
        end
    end
    end


```
---

## Launch Arguments

| Argument | Default | Description |
|---|---|---|
| `world` | `empty` | Name of the world `.sdf` file (without extension) to spawn the robot into. Only used when `gazebo:=true`. |
| `robot_model` | `fairino5` | Which Fairino model to load. Must be one of the keys in `MOVEIT_PKG_MAP` (`fairino3`, `fairino5`, `fairino10`, `fairino16`, `fairino20`, `fairino30`). |
| `mount` | `world` | What the robot base is mounted to. `world` means fixed base; any other value (e.g. `rail_carriage`) spawns an additional mount controller. |
| `gripper` | `none` | Which gripper to attach. Must be a key in `GRIPPER_JOINT_MAP` to get a working controller spawn (see [Gripper System](#gripper-system) below); `none` disables gripper hardware entirely. |
| `moveit` | `false` | If `true`, launches `move_group`, RViz, and the Gazebo-world-to-MoveIt collision-object bridge. |
| `gazebo` | `false` | If `true`, launches Ignition Gazebo with the selected `world` and spawns the robot into it. |
| `hardware` | `false` | If `true`, selects the real hardware control system instead of the fake/sim one. Takes priority over `gazebo` if both are set. |

### Example invocations

```bash
# MoveIt-only, fake hardware, no gripper
ros2 launch fairino_universe fairino_universe.launch.py moveit:=true

# MoveIt + a specific gripper
ros2 launch fairino_universe fairino_universe.launch.py moveit:=true gripper:=dh_ag145

# Full digital twin: real hardware driving RViz/MoveIt in parallel
ros2 launch fairino_universe fairino_universe.launch.py hardware:=true moveit:=true

# Gazebo simulation with a custom world
ros2 launch fairino_universe fairino_universe.launch.py gazebo:=true world:=my_world moveit:=true
```

---

## Control System Resolution

The `control_system` value passed into the xacro and into `move_group.launch.py` is resolved from the three boolean args with this precedence:

```
hardware:=true                 → control_system = "hardware"   (always wins)
hardware:=false, gazebo:=true  → control_system = "gazebo"
both false                     → control_system = "moveit"      (fake/simulated joints, default)
```

If both `hardware:=true` and `gazebo:=true` are passed, hardware wins and a log line is printed to make that explicit — this lets you mirror a real robot's motion in Gazebo without Gazebo actually driving the joints.

---

## Robot Description Generation

Unlike a typical launch file that shells out to the `xacro` CLI via `Command([...])`, this file processes the xacro file **directly in Python** using the `xacro` module:

```python
robot_description_xml = xacro.process_file(
    xacro_path,
    mappings={
        "robot_model": robot_model,
        "robot_mount": mount,
        "control_system": control_system,
        "gripper": gripper,
    },
).toxml()
```

This is resolved inside `launch_setup`, an `OpaqueFunction`, which means all four values (`robot_model`, `mount`, `control_system`, `gripper`) are fully resolved strings by the time xacro runs — no `LaunchConfiguration` substitution objects are passed into xacro. This avoids substitution-related quoting/escaping issues that can occur when passing launch arguments through `Command()`.

The resulting URDF XML string is published as the `robot_description` parameter to `robot_state_publisher`, and separately re-derived by `ros2_control_node` via the `/controller_manager/robot_description` remap from `/robot_description`.

---

## Controller Bring-Up Sequence

1. **`ros2_control_node`** starts, loaded with:
   - `ros2_controllers.yaml` (arm + joint_state_broadcaster + mount rail controller definitions, keyed by `robot_model`'s MoveIt package)
   - `gripper_controllers.yaml`, **only if** `gripper != "none"` and the file exists for that MoveIt package (checked with `os.path.exists`)

2. **`joint_state_broadcaster`** is spawned after a 1-second `TimerAction` delay (a simple wait for `controller_manager` to be ready — see [Known Limitations](#known-limitations--things-to-watch)).

3. **Arm controller** (`{robot_model}_controller`, e.g. `fairino5_controller`) is spawned, also after a 1s delay, with a `--controller-manager-timeout 10` to tolerate slower startup.

4. **Mount/rail controller** (`{mount}_controller`) is spawned **only if `mount != "world"`** — e.g. passing `mount:=rail_carriage` spawns `rail_carriage_controller` as a `joint_trajectory_controller/JointTrajectoryController`.

5. **Gripper controller** is spawned only if all of the following hold:
   - `gripper != "none"`
   - `gripper` is a recognized key in `GRIPPER_JOINT_MAP`
   - `gripper_controllers.yaml` exists for the selected MoveIt package

   If any check fails, a `[WARN]` is printed and gripper controller spawning is skipped silently (the rest of the launch continues normally, but the gripper won't be actuatable).

---

## Gripper System

The gripper system is designed to support grippers with differing numbers of actuated joints and, when necessary, entirely different controller names.

### `GRIPPER_JOINT_MAP`

Maps a gripper name to the list of joint names it actuates. **Note:** despite the in-code comment calling this "kept for reference... not useful right now," it is actually load-bearing — it's what `launch_setup` checks to decide whether a requested gripper is "known" (`if gripper not in GRIPPER_JOINT_MAP`). The comment should probably be updated to reflect this; the joint list itself isn't consumed elsewhere in this file, but the *keys* are used for validation.

```python
GRIPPER_JOINT_MAP = {
    "dh_ag95": ["dh_ag95_left_outer_knuckle_joint"],
    "dh_ag145": ["dh_ag145_gripper_finger1_joint"],
    "robotiq_2f_85": ["robotiq_2f_85_finger_joint"],
    "dh_pgc140": ["dh_pgc140_gripper_finger1_joint"],
    "dh_ag3": ["gripper_joint_X", "gripper_joint_X2"],
}
```

### `GRIPPER_CONTROLLER_NAME_MAP`

Most grippers share a single `ros2_control` controller named `gripper_controller`. Grippers needing a *different* controller (e.g. because they have more than one independently-actuated joint and are configured under a separate controller block in `gripper_controllers.yaml`) are listed here:

```python
GRIPPER_CONTROLLER_NAME_MAP = {"dh_ag3": "gripper_controller2"}
```

Any gripper not listed here falls back to `"gripper_controller"` via `.get(gripper, "gripper_controller")`.

### Adding a new gripper — checklist

1. Add the gripper's xacro macro and hook it into the top-level robot xacro's gripper-selection logic.
2. Add the actuated joint name(s) to `GRIPPER_JOINT_MAP`.
3. If the gripper needs a dedicated `ros2_control` controller (distinct joint count/type from the default), add its controller name to `GRIPPER_CONTROLLER_NAME_MAP` and define that controller block in `gripper_controllers.yaml`.
4. Register the controller's **type** under `controller_manager: ros__parameters:` in `ros2_controllers.yaml` (the controller manager needs this to instantiate the controller; `gripper_controllers.yaml` only carries the controller's own params).
5. Add a matching entry to the **MoveIt-side** controller config (`moveit_controllers.yaml` in the relevant `*_v6_moveit2_config` package) under `moveit_simple_controller_manager`, including the controller name in `controller_names` — this is a separate file from `gripper_controllers.yaml` and must be kept in sync manually; nothing in this launch file updates it automatically.
6. If using MoveIt planning groups for the gripper, ensure the SRDF's gripper group lists the new joint(s).

---

## Gazebo Integration (optional)

When `gazebo:=true`:

- The `IGN_GAZEBO_RESOURCE_PATH` environment variable is set (in `generate_launch_description`, before `launch_setup` runs) to include `fairino_description` and `rail_description` package share dirs, prepended to any existing value.
- A 2-second `TimerAction` launches `ros_gz_sim`'s `gz_sim.launch.py` with the resolved world file path and `-r` (run immediately).
- A `ros_gz_sim create` node spawns the robot into the running Gazebo instance from the `/robot_description` topic.

---

## MoveIt 2 Integration (optional)

When `moveit:=true`:

- Includes `move_group.launch.py` from `fairino_gazebo_config`, passing through `robot_model`, `mount`, `control_system`, `moveit_pkg`, and `gripper`.
- Starts RViz with the MoveIt config's `moveit.rviz` display config and injects `robot_description_kinematics` from `kinematics.yaml` as a parameter (needed for the interactive 3D end-effector marker to work).
- Starts `gazebo_world_to_moveit.py`, which parses the world SDF and publishes matching collision objects into the MoveIt planning scene. **Known issue:** this node currently fails when `gazebo:=false`/no real Gazebo install path is available for the selected world — see below.

---

