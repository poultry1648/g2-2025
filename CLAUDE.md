# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

A ROS 2 **Jazzy** colcon workspace (Python 3.12, Gazebo Sim via `ros_gz`) for an FRC robot ("g2") playing the Reefscape field. All packages live in `src/`; `build/`, `install/`, `log/` are colcon outputs and are git-ignored. Run commands from the workspace root (`/home/poultry/g2-2026`), not from `src/`.

## Commands

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install                      # build everything
colcon build --symlink-install --packages-select motor_node
source install/setup.bash                           # after every build that adds packages/entry points

colcon test --packages-select motor_node && colcon test-result --verbose
python3 -m pytest src/motor_node/test/test_motor_model.py   # single test file, fast, no build needed
python3 -m pytest src/motor_node/test/test_flake8.py        # lint one package (flake8 / pep257 / copyright tests)
```

- `motor_msgs` is the only `ament_cmake` package (message generation); everything else is `ament_python`. Rebuild `motor_msgs` and re-source after changing any `.msg`.
- Adding a new console script, launch file, or config/mesh directory requires editing that package's `setup.py` (`entry_points` / `data_files`) and rebuilding.
- The VS Code "ros2" terminal profile (`.vscode/ros2.bashrc`) auto-sources ROS and `install/setup.bash`.

### Running

```bash
ros2 launch bringup gazebo.launch.py        # Gazebo + reefscape world + motor_node/bridge, spawns g2 after 5 s
ros2 run joy joy_node                       # gamepad
ros2 run application teleop                 # /joy -> /g2/swerve/target (Twist)
ros2 run application swerve                 # Twist -> per-module MotorCommands
ros2 launch description display.launch.py   # RViz view of the URDF
ros2 launch urdf_tuner tune.launch.py       # Tk slider GUI + RViz; saves edits back to src/description/urdf/g2.urdf
ros2 launch kitbot_bringup bringup.launch.py  # separate kitbot (diff-drive) robot, full sim stack
```

Headless Gazebo: `ros2 launch bringup gazebo.launch.py gz_args:="-s -r <world_file>"`.

## Architecture

### Layering (g2 robot)

```
teleop (joy) ──Twist /g2/swerve/target──> swerve ──MotorCommand──> motor_node ──> Gazebo or TalonFX
                                                  <──MotorState───
```

- **`application`** — robot logic. Talks to motors *only* via `motor_msgs` topics, never to Gazebo directly, so the same code runs in sim and on the real robot. Swerve kinematics geometry is in `application/constants.py`.
- **`motor_node`** — the hardware abstraction. One node owns every motor, configured by `config/motors.yaml` (motor presets → `defaults` → per-motor overrides; gains, gear ratio, CAN id, inversion). Topics are derived in `config.py`:
  - `/g2/motors/<name>/command` (`MotorCommand`), `/g2/motors/<name>/state` (`MotorState`)
  - `/g2/joint_effort/<joint>` (sim only, Float64 torque)
- **Transports** (`transport.py` ABC): `transport: sim` uses `GazeboTransport`, which *emulates TalonFX firmware* — a DC motor model (`motor_model.py`) + PID runs in motor_node and publishes joint torque; `transport: talonfx` uses `TalonFXTransport` (Phoenix 6, imported lazily so sim doesn't need it). Select with the YAML or the `transport:=` launch arg.
- **Units**: `MotorCommand`/`MotorState` mirror Phoenix 6 — position in **rotations**, velocity in **rotations/s**, mechanism (joint) side after gear ratio. Gazebo joint states are radians and are converted in `GazeboTransport.update`. Control modes: DUTY_CYCLE, VOLTAGE, TORQUE_CURRENT, VELOCITY, POSITION.

### Sim wiring

- `motor_node/config/g2_bridge.yaml` bridges `/clock`, `/world/reefscape/model/g2/joint_state` → `/joint_states`, and each `/g2/joint_effort/<joint>` → Gazebo `cmd_force`. The URDF (`description/urdf/g2.urdf`) has a matching `ApplyJointForce` plugin per actuated joint.
- **Adding a motor** requires changes in three places kept in sync by hand: an entry in `motors.yaml`, a bridge entry in `g2_bridge.yaml`, and an `ApplyJointForce` plugin in the URDF. The model must be spawned as `g2` in world `reefscape` for the topic names to match.
- `bringup/launch/gazebo.launch.py` sets `GZ_SIM_RESOURCE_PATH` so `package://description/...` meshes and `gazebo/models/*` resolve.

### Other packages

- `gazebo` — Reefscape field world (`models/reefscape.world`) and field models.
- `description` — g2 URDF and meshes (`visual/*.glb`, `collision/*.stl`).
- `kitbot_*` — an independent, older diff-drive kitbot stack (its own description/bringup/application, bridged with `parameter_bridge` CLI args rather than motor_node).
- `bringup/g2_talker`, `g2_listener` — demo nodes only.
