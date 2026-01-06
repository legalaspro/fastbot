# fastbot_slam

ROS 2 navigation stack for Fastbot — mapping, localization, and path planning with Nav2.

## Table of Contents

- [Workflow Overview](#workflow-overview)
- [Step 1: Create a Map](#step-1-create-a-map)
- [Step 2: Navigate (Recommended Launch)](#step-2-navigate-recommended-launch)
- [Individual Launch Files](#individual-launch-files)
  - [cartographer.launch.py](#️-cartographerlaunchpy)
  - [localization.launch.py](#-localizationlaunchpy)
  - [pathplanner.launch.py](#-pathplannerlaunchpy)
  - [map_server.launch.py](#️-map_serverlaunchpy)
- [Simulation Workarounds](#simulation-workarounds)
- [Configuration Files](#configuration-files)
- [Dependencies](#dependencies)
- [License](#license)

---

## Workflow Overview

Navigation requires a pre-built map. Follow these steps:

```
1. CREATE MAP        →  2. NAVIGATE
   cartographer.launch.py     navigation.launch.py
```

**Step 1:** Build a map using Cartographer SLAM
**Step 2:** Use the saved map for autonomous navigation

---

## Step 1: Create a Map

Before navigation, you need to create a map of your environment using Cartographer:

```bash
ros2 launch fastbot_slam cartogprapher.launch.py
```

Drive the robot around to cover the area. When finished, save the map:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map
```

Copy the generated `.yaml` and `.pgm` files to the `config/` folder.

---

## Step 2: Navigate (Recommended Launch)

Once you have a map, use the all-in-one navigation launch:

```bash
ros2 launch fastbot_slam navigation.launch.py map_file:=my_map.yaml
```

This single launch file starts the complete navigation stack:

- **Timestamp filter** — fixes non-monotonic timestamps from simulation
- **Map server** — loads your saved map
- **AMCL** — particle filter localization
- **Nav2 stack** — planner, controller, behaviors, behavior tree navigator
- **Auto-localization** — triggers global localization and spins the robot to help the particle filter converge quickly
- **RViz** — visualization (starts after Nav2 is ready)

### Parameters

| Parameter      | Default              | Description                           |
| -------------- | -------------------- | ------------------------------------- |
| `map_file`     | `studio_edit.yaml`   | Map YAML filename in `config/` folder |
| `use_sim_time` | `True`               | Use simulation clock (Gazebo)         |
| `scan_input`   | `/fastbot_1/scan`    | Input laser scan topic                |
| `scan_output`  | `/scan`              | Republished scan topic (monotonic)    |
| `odom_input`   | `/fastbot_1/odom`    | Input odometry topic                  |
| `odom_output`  | `/odom`              | Republished odom topic (monotonic)    |
| `cmd_vel_out`  | `/fastbot_1/cmd_vel` | Velocity command output topic         |

### Example with custom topics

```bash
ros2 launch fastbot_slam navigation.launch.py \
  map_file:=warehouse.yaml \
  scan_input:=/robot/scan \
  odom_input:=/robot/odom \
  cmd_vel_out:=/robot/cmd_vel \
  use_sim_time:=False
```

---

## Individual Launch Files

For debugging or modular setups, you can run components separately:

### 🗺️ cartographer.launch.py

SLAM mapping with Cartographer.

```bash
ros2 launch fastbot_slam cartogprapher.launch.py
```

| Parameter      | Default | Description          |
| -------------- | ------- | -------------------- |
| `use_sim_time` | `True`  | Use simulation clock |

---

### 📍 localization.launch.py

Localization only (map server + AMCL), no path planning.

```bash
ros2 launch fastbot_slam localization.launch.py map_file:=studio.yaml
```

| Parameter      | Default       | Description           |
| -------------- | ------------- | --------------------- |
| `map_file`     | `studio.yaml` | Map YAML in `config/` |
| `use_sim_time` | `True`        | Use simulation clock  |
| `rviz`         | `True`        | Start RViz            |

---

### 🧭 pathplanner.launch.py

Localization + Nav2 path planning (modular approach).

```bash
ros2 launch fastbot_slam pathplanner.launch.py map_file:=studio.yaml
```

| Parameter      | Default       | Description           |
| -------------- | ------------- | --------------------- |
| `map_file`     | `studio.yaml` | Map YAML in `config/` |
| `use_sim_time` | `True`        | Use simulation clock  |

---

### 🗂️ map_server.launch.py

View a saved map in RViz (no localization or navigation).

```bash
ros2 launch fastbot_slam map_server.launch.py map_file:=studio.yaml
```

| Parameter  | Default       | Description           |
| ---------- | ------------- | --------------------- |
| `map_file` | `studio.yaml` | Map YAML in `config/` |

---

## Simulation Workarounds

### Timestamp Filter

Some simulators produce non-monotonic timestamps that break AMCL and Cartographer. The `timestamp_filter_node.py` republishes `/scan` and `/odom` with corrected monotonic timestamps by nudging repeated timestamps forward by 1 nanosecond.

### Auto-Localization

On startup, the robot's position is unknown. The `trigger_global_localization.py` script:

1. Waits for AMCL to be active
2. Calls `/reinitialize_global_localization` to spread particles across the map
3. Spins the robot slowly (configurable duration/speed) to let the particle filter converge

This happens automatically with `navigation.launch.py` — no manual "2D Pose Estimate" needed.

---

## Configuration Files

Located in `config/`:

| File                                             | Purpose                                 |
| ------------------------------------------------ | --------------------------------------- |
| `amcl_config.yaml`                               | AMCL localization parameters            |
| `cartographer.lua`                               | Cartographer SLAM configuration         |
| `controller.yaml`                                | Nav2 DWB controller settings            |
| `planner.yaml`                                   | Nav2 global planner (NavFn/Smac)        |
| `behavior.yaml`                                  | Recovery behaviors (spin, backup, wait) |
| `bt_navigator.yaml`                              | Behavior tree navigator config          |
| `navigate_to_pose_w_replanning_and_recovery.xml` | Custom behavior tree                    |
| `studio.yaml` / `studio.pgm`                     | Example map                             |

---

## Dependencies

- ROS 2 Humble (or later)
- cartographer_ros
- nav2_amcl
- nav2_map_server
- nav2_planner
- nav2_controller
- nav2_behaviors
- nav2_bt_navigator
- nav2_lifecycle_manager

## License

MIT — see LICENSE in repository root.
