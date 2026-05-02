# Two-Wheeled Self-Balancing Robot (ROS2)

## Overview

This project is a **ROS2-based two-wheeled self-balancing robot** simulated in Gazebo using `ros_gz` integration.

The robot uses:
- IMU sensor for orientation feedback
- Differential drive wheel control via `ros2_control`
- A velocity-based controller for stabilization
- Gazebo Sim physics for dynamics

The main goal is to achieve **real-time self-balancing using feedback control (PID).**

---

## System Architecture

```
IMU (Gazebo)
   |
ros_gz_bridge
   |
ROS2 IMU Topic (/imu/data)
   |
State Estimator (pitch, pitch_rate)
   |
Balance Controller (PID)
   |
wheel_controller (ros2_control)
   |
Gazebo Simulation
```

---

## Software Stack

| Component | Technology |
|----------|------------|
| OS | Ubuntu 22.04 |
| ROS Version | ROS2 Humble |
| Simulator | Gazebo Sim (ros_gz) |
| Control | ros2_control |
| Estimation | IMU EKF |
| Bridge | ros_gz_bridge |
| Robot Description | URDF + Gazebo xacro |

---

## Controllers

This project uses:
- `joint_state_broadcaster`
- `forward_command_controller` (wheel velocity control)

---

## Topics

### Key ROS2 Topics
- `/imu/data` - IMU sensor data
- `/joint_states` - wheel joint feedback
- `/wheel_controller/commands` - velocity commands
- `/tf` - robot transforms

---

## Build Instructions

### 1. Build workspace
```bash
colcon build
```

### 2. Source environment
```bash
source install/setup.bash
```

---

## Launch Simulation

### Start full simulation
```bash
ros2 launch robot_bringup bringup.launch.py
```

This will start:
- Gazebo Sim
- Robot spawn
- IMU + wheel controllers
- ROS2 bridge

---

## Control Flow

**balancing controller node**:
- Reads `/imu/data`
- Computes pitch angle
- Outputs wheel velocity commands

---

## Project Type

- Mobile Robotics
- ROS2 Control + Gazebo Simulation
