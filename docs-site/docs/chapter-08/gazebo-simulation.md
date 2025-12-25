---
sidebar_position: 2
title: "Gazebo Simulation"
description: "Using Gazebo simulator for robot development with ROS2 integration"
---

# Gazebo Simulation

## Introduction

Gazebo is the most widely used open-source robot simulator, offering tight integration with ROS/ROS2 and a comprehensive feature set for robot development.

## Gazebo Versions

### Gazebo Classic (Gazebo 11)
- Mature and stable
- Extensive plugin library
- ROS1 integration

### Gazebo Sim (Ignition)
- Modern architecture
- Better performance
- ROS2 native support
- Actively developed

## Installation

### With ROS2 Humble
```bash
sudo apt install ros-humble-gazebo-ros-pkgs
```

### Standalone Gazebo Sim
```bash
sudo apt install gz-harmonic
```

## Core Concepts

### World Files
Define the simulation environment in SDF format.

```xml
<?xml version="1.0"?>
<sdf version="1.8">
  <world name="my_world">
    <physics type="ode">
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <light type="directional" name="sun">
      <pose>0 0 10 0 0 0</pose>
    </light>
    <model name="ground_plane">
      <!-- Ground plane definition -->
    </model>
  </world>
</sdf>
```

### Robot Models (URDF/SDF)
- URDF: Universal Robot Description Format
- SDF: Simulation Description Format
- Define links, joints, sensors

### Plugins
Extend Gazebo functionality:
- Sensor plugins (camera, LiDAR)
- Model plugins (controllers)
- World plugins (physics modifications)

## ROS2 Integration

### Gazebo-ROS Bridge
```xml
<plugin filename="libgazebo_ros_diff_drive.so" name="diff_drive">
  <ros>
    <namespace>/robot</namespace>
    <remapping>cmd_vel:=cmd_vel</remapping>
    <remapping>odom:=odom</remapping>
  </ros>
  <wheel_separation>0.5</wheel_separation>
  <wheel_diameter>0.1</wheel_diameter>
</plugin>
```

### Common Plugins

| Plugin | Purpose |
|--------|---------|
| gazebo_ros_camera | RGB camera sensor |
| gazebo_ros_depth_camera | Depth camera |
| gazebo_ros_ray_sensor | LiDAR simulation |
| gazebo_ros_imu_sensor | IMU data |
| gazebo_ros_diff_drive | Differential drive control |

## Launching Gazebo with ROS2

### Launch File Example
```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                'gazebo_ros', '/launch/gazebo.launch.py'
            ]),
        ),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'robot', '-file', 'robot.urdf'],
        ),
    ])
```

## Physics Settings

### Real-time Factor
- 1.0 = real-time
- `>1.0` = faster than real-time
- `<1.0` = slower than real-time

### Step Size
- Smaller = more accurate
- Larger = faster simulation
- Typical: 0.001s

## Sensors in Gazebo

### Camera
```xml
<sensor type="camera" name="camera">
  <update_rate>30.0</update_rate>
  <camera>
    <horizontal_fov>1.3962634</horizontal_fov>
    <image>
      <width>640</width>
      <height>480</height>
    </image>
  </camera>
</sensor>
```

### LiDAR
```xml
<sensor type="ray" name="lidar">
  <ray>
    <scan>
      <horizontal>
        <samples>360</samples>
        <resolution>1</resolution>
        <min_angle>-3.14159</min_angle>
        <max_angle>3.14159</max_angle>
      </horizontal>
    </scan>
    <range>
      <min>0.1</min>
      <max>10.0</max>
    </range>
  </ray>
</sensor>
```

## Summary

Gazebo provides a complete simulation environment for robot development with excellent ROS2 support.

---

*Next: [NVIDIA Isaac Sim](./isaac-sim.md)*
