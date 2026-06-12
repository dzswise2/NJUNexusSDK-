# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Nexus SDK is a ROS2 Humble workspace for robot master-slave teleoperation, simulation, and data collection. A Nexus-Arm (master) controls slave arms (Y1, AR5, Piper) via DDS. The system runs on Ubuntu 22.04 with Python 3.10+.

## Build Commands

```bash
# Source ROS2 environment (required before any build/run)
source /opt/ros/humble/setup.bash

# Build entire workspace
colcon build --symlink-install

# Build a single package
colcon build --symlink-install --packages-select <package_name>

# Build a package and its dependencies
colcon build --symlink-install --packages-up-to <package_name>

# Source workspace after build
source install/setup.bash

# infra_msg must be built first if other packages fail to find it
colcon build --symlink-install --packages-select infra_msg
source install/setup.bash
colcon build --symlink-install
```

## Running

```bash
# Simulation (no hardware needed)
ros2 launch nexus_manage nexus_arm_v15_right_to_y1_sim_system.launch.py

# Real hardware (V15 Right → AR5-08L suction cup)
ros2 launch nexus_manage nexus_arm_v15_right_to_ar5_08l_suction_cup_real_system.launch.py

# Cross-subnet deployment (master/slave on different networks)
ros2 launch nexus_manage nexus_arm_v15_right_to_ar5_08l_suction_cup_real_system_cross_subnet.launch.py role:=master
ros2 launch nexus_manage nexus_arm_v15_right_to_ar5_08l_suction_cup_real_system_cross_subnet.launch.py role:=slave robot_id:=ar5_01

# Kill all ROS2 processes
./scripts/kill_ros_processes.sh
```

## Architecture

The workspace is a colcon multi-package workspace. All packages live as git submodules under `src/`. The `.gitmodules` file only tracks 6 of 10 packages; the others (mujoco_sim, gripper_keyboard, data_collector, collector_test_tool) are tracked differently.

### Package Dependency Flow (teleoperation)

```
teleop_adapter (hardware I/O) → human_data (kinematics solver) → nexus_manage (state machine + routing) → robot_controller (arm control)
                                                                                                           ↕
                                                                                                      mujoco_sim (simulation)
```

### Key Packages

- **infra_msg** — ROS2 message/service definitions. Must build first; all other packages depend on it.
- **nexus_manage** — System orchestrator. Contains launch files that compose the full system, a state machine engine, and cross-subnet DDS configuration. The `launch/` directory has the top-level system launch files. `config/slave_registry.yaml` maps slave IPs for multi-slave deployments.
- **robot_controller** — C++ arm control node. Uses Pinocchio for dynamics, OSQP for QP solving. Config files under `config/` set `is_simulation: true/false` to toggle motor inertia compensation.
- **human_data** — C++ kinematics solver node. Reads keyboard/pedal input via libevdev from `/dev/input/event*`.
- **teleop_adapter** — Hardware communication layer (serial ports, vendor SDKs).
- **mujoco_sim** — Python MuJoCo simulation with ROS2 bridge. Robot URDF/MJCF models in `robot_description/`.
- **gripper_keyboard** — Reads keyboard keys (Q/W/E/R/T) via libevdev for gripper and teleop control.
- **data_collector** — Records teleoperation episodes to `~/datasets/`.
- **vision_data_hub** — Multi-camera capture + WebRTC streaming (depends on RealSense, GStreamer, FFmpeg).

### Cross-Subnet DDS (CycloneDDS)

Launch files in `nexus_manage/launch/` ending in `_cross_subnet.launch.py` generate CycloneDDS XML at runtime with explicit peer lists and port assignments. The `slave_registry.py` module auto-detects role by matching local IPs against `slave_registry.yaml`. Key config constants (`_MASTER_EXTERNAL_IP`, `_SLAVE_EXTERNAL_IP`, `_PORT_BASE`) are at the top of each cross-subnet launch file.

## Release Scripts

- `scripts/make_release.sh` — Builds closed-source customer release (prebuilt .so + headers, no .cpp). Configured via `RELEASE_PACKAGES`, `PKG_LIBRARIES`, etc. at the top.
- `scripts/publish_release.sh` — Publishes release to `jd_release` branch via git worktree.
- `scripts/ci_release.sh` — Dual-architecture (x86_64 + aarch64) release using Docker containers.
- `scripts/build_docker_release.sh` — Builds Docker environment image with all deps pre-installed.
- `scripts/check_public_headers.sh` — CI gate checking for leaked vendor SDK includes or excessive private members.

## Submodule Workflow

```bash
# Initialize all submodules
git submodule update --init --recursive

# Switch submodules to a specific branch (not all submodules have all branches)
git submodule foreach 'if git branch -r | grep -q "origin/<branch>"; then git checkout <branch> && git pull origin <branch>; fi'
```

## Language & Conventions

- C++ packages use CMake via `ament_cmake`. Node executables are named `*_node` (e.g., `arm_control_node`, `human_data_solver_node`, `teleop_manager_node`).
- Python packages (mujoco_sim, data_collector) use `ament_python` with `setup.py`.
- Launch files are Python-based (`.launch.py`), located in each package's `launch/` directory. System-level launch files that compose multiple packages are in `nexus_manage/launch/`.
- Configuration is via YAML files in each package's `config/` directory, loaded as ROS2 parameters.
- The project language is primarily Chinese (comments, docs, commit messages).
