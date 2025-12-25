---
sidebar_position: 1
---

# Robotics Fundamentals

This chapter introduces the core concepts and principles underlying all robotic systems. Understanding these fundamentals is essential for building intelligent physical AI systems.

## What is a Robot?

A robot is a programmable machine capable of:
- **Sensing** its environment
- **Processing** information to make decisions
- **Acting** on the physical world

### Key Characteristics

| Characteristic | Description |
|----------------|-------------|
| Autonomy | Ability to operate without human intervention |
| Programmability | Behavior can be modified through software |
| Sensing | Ability to perceive environment |
| Actuation | Ability to affect environment |

## Robot Classification

### By Application Domain

```
Robots
├── Industrial Robots
│   ├── Manufacturing arms
│   ├── Welding robots
│   └── Assembly systems
├── Service Robots
│   ├── Domestic (vacuum, lawn)
│   ├── Professional (medical, logistics)
│   └── Social (companions, assistants)
├── Mobile Robots
│   ├── Ground (wheeled, legged, tracked)
│   ├── Aerial (drones, UAVs)
│   └── Underwater (AUVs, ROVs)
└── Humanoid Robots
    ├── Full humanoid (bipedal)
    ├── Upper-body (torso + arms)
    └── Head-only (social interaction)
```

### By Degrees of Freedom (DoF)

**Degrees of Freedom** represent the number of independent parameters that define a robot's configuration:

- **1-DoF**: Simple linear actuator
- **3-DoF**: Basic positioning (XYZ)
- **6-DoF**: Full pose control (position + orientation)
- **7+ DoF**: Redundant manipulators (extra flexibility)

## Robot Components

### 1. Mechanical Structure

The physical body consisting of:
- **Links**: Rigid body segments
- **Joints**: Connections allowing relative motion
- **End-effector**: The tool or gripper at the robot's "hand"

### 2. Actuators

Devices that produce motion:

| Type | Pros | Cons | Use Case |
|------|------|------|----------|
| DC Motors | Simple, cheap | Limited torque | Mobile robots |
| Servo Motors | Precise, integrated | Cost | Manipulators |
| Stepper Motors | Open-loop positioning | Low speed | 3D printers |
| Hydraulic | High force | Complex, messy | Heavy industry |
| Pneumatic | Fast, light | Noisy, imprecise | Grippers |

### 3. Sensors

Devices that gather information:

**Proprioceptive** (internal state):
- Encoders (position)
- IMU (orientation, acceleration)
- Current sensors (torque estimation)

**Exteroceptive** (environment):
- Cameras (visual information)
- LIDAR (distance measurement)
- Force/torque sensors (contact)

### 4. Control System

The "brain" that coordinates:
- **Low-level**: Motor control loops
- **Mid-level**: Motion execution
- **High-level**: Task planning

## Coordinate Systems and Transformations

### Reference Frames

Every robot uses multiple coordinate frames:

```
World Frame (W)
     │
     └── Robot Base Frame (B)
              │
              ├── Joint 1 Frame
              │        │
              │        └── Joint 2 Frame
              │                  │
              │                  └── ... 
              │
              └── End-Effector Frame (E)
```

### Homogeneous Transformations

Position and orientation are combined using 4×4 matrices:

$$
T = \begin{bmatrix}
R_{3 \times 3} & p_{3 \times 1} \\
0_{1 \times 3} & 1
\end{bmatrix}
$$

Where:
- $R$: Rotation matrix
- $p$: Translation vector

### Forward and Inverse Kinematics

**Forward Kinematics**: Given joint angles, find end-effector pose
$$
T_{end} = T_1 \cdot T_2 \cdot T_3 \cdot ... \cdot T_n
$$

**Inverse Kinematics**: Given desired pose, find joint angles
- Often multiple solutions exist
- May have no solution (unreachable)
- Computational complexity varies

## The Control Loop

```
┌─────────────────────────────────────────────────────────┐
│                                                         │
│  Reference    +  ┌────────┐  Control  ┌────────┐       │
│  Input  ──────○──│Controller│────────▶│ Robot  │───┐   │
│               -  └────────┘  Signal   └────────┘   │   │
│               │                                     │   │
│               │      ┌────────────┐                │   │
│               └──────│   Sensor   │◀───────────────┘   │
│                      │  Feedback  │                     │
│                      └────────────┘                     │
│                                                         │
└─────────────────────────────────────────────────────────┘
```

### PID Control

The most common control strategy:

$$
u(t) = K_p e(t) + K_i \int e(t)dt + K_d \frac{de(t)}{dt}
$$

Where:
- $K_p$: Proportional gain
- $K_i$: Integral gain
- $K_d$: Derivative gain
- $e(t)$: Error at time $t$

## Key Takeaways

1. **Robots combine** sensing, processing, and actuation
2. **Degrees of freedom** determine the robot's workspace and capability
3. **Coordinate transformations** are fundamental for robot mathematics
4. **Control systems** enable precise, stable motion
5. **Understanding fundamentals** enables building complex systems

---

*Next: [Sensors and Actuators](./sensors-and-actuators.md) →*
