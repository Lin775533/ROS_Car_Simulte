# 🤖 ROS Motion Planning & Control

<div align="center">

![ROS Noetic](https://img.shields.io/badge/ROS-Noetic-blue?style=for-the-badge&logo=ros)
![Gazebo](https://img.shields.io/badge/Gazebo-11-orange?style=for-the-badge)
![Python](https://img.shields.io/badge/Python-3.8+-green?style=for-the-badge&logo=python)
![TurtleBot3](https://img.shields.io/badge/TurtleBot3-Waffle_Pi-red?style=for-the-badge)

</div>

## 📋 Overview

This repository contains implementation of motion planning and control algorithms for a TurtleBot3 robot using ROS and Gazebo simulation environment. The project focuses on:

- PID controller implementation for precise robot navigation
- Multiple control modes for robot movement
- Trajectory planning and execution
- Real-time pose monitoring and feedback

## 🗂️ Repository Structure

```
Lab3_YuChun_Lin_ws/
├── build/                  # Build directory
├── devel/                  # Development space
├── src/                    # Source code
│   ├── pid_controller/     # Main ROS package
│   │   ├── src/
│   │   │   ├── pid_controller.py     # PID implementation
│   │   │   └── motion_planner.py     # Reference pose generator
│   │   ├── CMakeLists.txt
│   │   └── package.xml
└── catkin_workspace        # Workspace configuration
```

## ⚙️ Implementation Details

### PID Controller (`pid_controller.py`)

The PID controller implements two separate control loops:

1. **Angular Velocity Controller**:
   - Controls the robot's orientation
   - Tunes robot turning to face target direction
   - Parameters: Kp_angular, Ki_angular, Kd_angular

2. **Linear Velocity Controller**:
   - Controls the robot's forward movement
   - Manages acceleration and deceleration
   - Parameters: Kp_linear, Ki_linear, Kd_linear

### Motion Planner (`motion_planner.py`)

The motion planner:
- Takes user input for desired position (x, y, θ)
- Publishes reference poses
- Monitors robot progress
- Supports two operation modes:
  - **Mode 0**: Sequential control (rotate → move → rotate)
  - **Mode 1**: Simultaneous position and orientation control

## 🚀 Control Modes

### Mode 0: Sequential Control

<div align="center">
<img src="screenshots/mode0.png" alt="Mode 0 Trajectory" width="600"/>
</div>

Control sequence:
1. First rotation to face target
2. Linear movement to target position
3. Final rotation to achieve target orientation

### Mode 1: Simultaneous Control

<div align="center">
<img src="screenshots/mode1.png" alt="Mode 1 Trajectory" width="600"/>
</div>

- Simultaneously controls both linear and angular velocities
- Robot moves in curved trajectories
- More efficient but potentially more complex control

## 💻 How to Run

1. **Start ROS master**:
   ```bash
   roscore
   ```

2. **Launch Gazebo simulation environment**:
   ```bash
   roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch
   ```

3. **Run PID controller node**:
   ```bash
   rosrun pid_controller pid_controller.py
   ```

4. **Run motion planner node**:
   ```bash
   rosrun pid_controller motion_planner.py
   ```

5. **Enter target coordinates when prompted**:
   ```
   Enter x goal: 2.0
   Enter y goal: 3.0
   Enter theta goal (radians): 1.57
   Enter mode (0 or 1): 1
   ```

## 📊 Performance Metrics

The system achieves high precision in target acquisition:

| Metric | Requirement | Achieved |
|--------|-------------|----------|
| Position Error | ≤ 0.1 meters | ✅ 0.05 meters |
| Orientation Error | ≤ 0.1 radians | ✅ 0.07 radians |

## 🔍 PID Tuning

For optimal performance, the PID parameters were tuned using the following approach:

1. Set Ki and Kd to zero
2. Increase Kp until system oscillates
3. Add Kd for dampening
4. Fine-tune Ki to eliminate steady-state error

Final parameters:

```python
# Angular Controller
Kp_angular = 1.5
Ki_angular = 0.01
Kd_angular = 0.5

# Linear Controller
Kp_linear = 0.8
Ki_linear = 0.01
Kd_linear = 0.2
```

## 📌 Author

**Yu-Chun Lin**  
University of California, Irvine  
Master of Embedded & Cyber-Physical Systems
