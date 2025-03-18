# ROS 2 Dual Arm Teleoperation

> **Note:** The demonstration GIFs below are large files and may take a moment to load.


![Dual Arm Demo 1](readme_resources/01.gif)
![Dual Arm Demo 2](readme_resources/02.gif)

This repository contains a ROS 2 package for vision-based teleoperation of a dual robot arm system. The system uses a camera to track hand movements via MediaPipe and maps them to control commands for two robot arms with grippers.

## Project Overview

This system allows:
- Tracking up to two hands simultaneously using a camera
- Mapping hand positions to robot arm movements
- Using hand gestures to control gripper open/close actions
- Visualization via RViz

## Repository Structure

```
teleop_arm_control/
├── config/                      # Configuration files
│   └── simple_config.rviz       # RViz configuration for visualization
├── launch/                      # Launch files
│   └── basic_teleop_launch.launch.py # Main launch file for teleoperation
├── package.xml                  # Package manifest
├── resource/                    # Package resources
│   └── teleop_arm_control       # Package marker
├── setup.cfg                    # Package setup configuration
├── setup.py                     # Package setup script
├── teleop_arm_control/          # Python source code
│   ├── __init__.py              # Package initialization 
│   ├── joint_publisher_node.py  # Node to publish joint states for the arms
│   └── teleoperation_node.py    # Node for camera-based hand tracking
├── urdf/                        # Robot description files
│   └── robot_arm.urdf.xacro     # URDF for the dual arm robot system
└── README.md                    # This file
```

## System Architecture

The system consists of three main components:

1. **Teleoperation Node (`teleoperation_node.py`)**
   - Processes camera frames using MediaPipe for hand detection
   - Tracks up to two hands simultaneously
   - Maps hand positions to target poses for the robot arms
   - Detects hand gestures for gripper control (open/closed)
   - Publishes target poses and gripper commands

2. **Joint Publisher Node (`joint_publisher_node.py`)**
   - Receives target poses from the teleoperation node
   - Performs simplified inverse kinematics for the robot arms
   - Controls the arms' joint positions
   - Manages gripper actions based on commands

3. **Robot State Publisher**
   - Loads the robot model from URDF
   - Publishes the robot's state for visualization

## Robot Configuration

The robot consists of two identical arms, each with:
- 6 revolute joints for arm movement
- 2 prismatic joints for gripper control
- Total of 16 controllable joints across both arms

## Prerequisites

- ROS 2 (tested on Humble and Rolling)
- Python 3
- OpenCV
- MediaPipe
- cv_bridge
- transforms3d

## Installation

1. Clone this repository into your ROS 2 workspace's `src` directory:
   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/yourusername/teleop_arm_control.git
   ```

2. Install dependencies:
   ```bash
   pip install mediapipe opencv-python transforms3d
   ```

3. Build the package:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select teleop_arm_control
   ```

## Usage

1. Source your ROS 2 workspace:
   ```bash
   source ~/ros2_ws/install/setup.bash
   ```

2. Launch the teleoperation system:
   ```bash
   ros2 launch teleop_arm_control basic_teleop_launch.launch.py
   ```

3. Use your hands in front of the camera to control the robot arms:
   - Move your hands to control arm positions
   - Hand orientation controls the arm's end-effector orientation
   - Open/close your hand to open/close the gripper

## Hand Control Mapping

- **Hand Position**: Controls the arm's end-effector position
- **Hand Orientation**: Controls the arm's end-effector orientation (full 6-DOF control)
- **Hand Open/Closed**: Controls the gripper (open/closed)

## License

This project is licensed under the Apache License 2.0 - see the LICENSE file for details.