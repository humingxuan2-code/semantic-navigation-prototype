# Semantic Navigation Prototype

A closed-loop mobile robot navigation prototype built in Gazebo Fortress with C++ and Gazebo Transport.

The project implements odometry-based motion control for a differential-drive mobile robot. It progresses from basic velocity commands to closed-loop distance control, yaw control, point-to-point navigation, trajectory logging, and multi-case benchmark evaluation.

## Project Highlights

- Differential-drive mobile robot simulation in Gazebo Fortress
- C++17 control programs using Gazebo Transport
- Real-time odometry feedback for position `(x, y)` and heading `yaw`
- Closed-loop distance control
- Closed-loop 90-degree left and right turns
- Relative point-to-point navigation
- CSV trajectory logging and Python visualization
- Six-case continuous relative-navigation benchmark

## Technical Stack

| Component | Configuration |
|---|---|
| Host OS | Windows |
| Linux environment | WSL2 |
| Ubuntu | 22.04.5 LTS |
| Robotics middleware | ROS 2 Humble |
| Simulator | Gazebo Fortress / Ignition Gazebo |
| Main language | C++17 |
| Visualization | Python + Matplotlib |
| Robot model | Differential-drive wheeled robot |

## Core Capabilities

### 1. Closed-Loop Distance Control

The robot reads odometry feedback and adjusts its forward speed based on the remaining distance to the goal.

Example result:

    Target distance: 0.500 m
    Actual distance: 0.498 m
    Absolute error: 0.002 m

### 2. Closed-Loop Yaw Control

The controller converts the odometry quaternion into yaw and continuously adjusts angular velocity according to the remaining heading error.

Validated 90-degree turning results:

    Left turn:
    Actual turn: 90.40 degrees
    Final angle error: 0.40 degrees

    Right turn:
    Actual turn: -89.44 degrees
    Final angle error: 0.56 degrees

### 3. Closed-Loop Point-to-Point Navigation

The robot continuously performs:

    compute target heading
    -> rotate in place until aligned
    -> move forward toward the target
    -> re-check odometry and correct heading
    -> stop inside the goal tolerance

A relative target can be specified as:

    forward distance (m)
    left offset (m)

Example command:

    ~/semantic_nav_ws/tools/drive_blue_goto rel 0.40 0.30

Example result:

    Initial target distance: 0.500 m
    Final target error: 0.002 m

## Trajectory Logging and Visualization

During navigation, the system records the following data into CSV:

- time
- world x and y position
- yaw angle
- target position
- distance to goal
- heading error
- linear velocity
- angular velocity
- current control mode

A Python script visualizes the actual trajectory, start point, target point, and final point.

### Single Navigation Trajectory

![Single navigation trajectory](outputs/exp009_rel_040_030/trajectory.png)

## Multi-Case Navigation Benchmark

Six continuous relative-navigation tasks were executed without resetting the robot between cases.

| Case | Relative target | Final error |
|---|---:|---:|
| case01_straight_030 | forward 0.30 m | 4.97 mm |
| case02_left_030_020 | forward 0.30 m, left 0.20 m | 8.45 mm |
| case03_right_030_020 | forward 0.30 m, right 0.20 m | 8.88 mm |
| case04_straight_050 | forward 0.50 m | 5.29 mm |
| case05_left_040_030 | forward 0.40 m, left 0.30 m | 3.79 mm |
| case06_right_040_030 | forward 0.40 m, right 0.30 m | 5.02 mm |

Benchmark summary:

    Average final error: approximately 6.07 mm
    Maximum final error: approximately 8.88 mm
    Goal tolerance: 25 mm

### Final Goal Error Comparison

![Final goal error comparison](outputs/benchmark_rel_v1/summary/final_error_comparison.png)

### Six Continuous Navigation Trajectories

![Six trajectory runs](outputs/benchmark_rel_v1/summary/trajectory_overlay.png)

## Repository Structure

    semantic_nav_ws/
    ├── README.md
    ├── docs/
    │   └── PROJECT_LOG.md
    ├── tools/
    │   ├── drive_blue.cpp
    │   ├── drive_blue_distance.cpp
    │   ├── drive_blue_distance_slow.cpp
    │   ├── drive_blue_yaw.cpp
    │   ├── drive_blue_goto.cpp
    │   ├── drive_blue_goto_trace.cpp
    │   ├── plot_trajectory.py
    │   ├── run_navigation_benchmark.sh
    │   └── summarize_benchmark.py
    └── outputs/
        ├── exp009_rel_040_030/
        └── benchmark_rel_v1/
            ├── manifest.csv
            └── summary/
                ├── benchmark_summary.csv
                ├── final_error_comparison.png
                └── trajectory_overlay.png

## Build

The control programs use Gazebo Transport and Ignition message libraries.

Example build command:

    g++ -std=c++17 -O2 \
      ~/semantic_nav_ws/tools/drive_blue_goto.cpp \
      -o ~/semantic_nav_ws/tools/drive_blue_goto \
      $(pkg-config --cflags --libs ignition-transport11 ignition-msgs8)

## Run

Start the Gazebo differential-drive demo first:

    source /opt/ros/humble/setup.bash
    export LIBGL_ALWAYS_SOFTWARE=1
    ros2 launch ros_gz_sim_demos diff_drive.launch.py rviz:=false

Examples:

    ~/semantic_nav_ws/tools/drive_blue_yaw left90
    ~/semantic_nav_ws/tools/drive_blue_yaw right90

    ~/semantic_nav_ws/tools/drive_blue_goto rel 0.40 0.30

    ~/semantic_nav_ws/tools/drive_blue_goto_trace rel 0.40 0.30
    python3 ~/semantic_nav_ws/tools/plot_trajectory.py

    ~/semantic_nav_ws/tools/run_navigation_benchmark.sh
    python3 ~/semantic_nav_ws/tools/summarize_benchmark.py

## Current Limitations and Next Steps

The current benchmark is a continuous relative-navigation test, meaning that each case starts from the previous case's final pose. Future work includes:

- independent repeated trials from fixed initial poses
- waypoint-sequence navigation
- obstacle maps and collision-aware planning
- integration with ROS 2 navigation tools
- visual perception and semantic navigation
- simulation-to-real transfer to wheeled robot platforms
