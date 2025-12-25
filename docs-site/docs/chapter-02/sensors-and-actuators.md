---
sidebar_position: 2
---

# Sensors and Actuators

Sensors and actuators form the interface between a robot's computational brain and the physical world. This section provides a detailed look at the technologies that enable robots to perceive and act.

## Sensors: The Robot's Senses

### Proprioceptive Sensors

These sensors measure the robot's internal state.

#### Encoders

Measure rotational position and velocity:

| Type | Resolution | Absolute/Incremental | Cost |
|------|------------|---------------------|------|
| Optical | Very high (>10,000 CPR) | Both | High |
| Magnetic | Medium | Both | Medium |
| Potentiometer | Low | Absolute | Low |

**Applications**: Joint position feedback, wheel odometry

#### Inertial Measurement Units (IMU)

Combine multiple sensors:
- **Accelerometer**: Linear acceleration (3-axis)
- **Gyroscope**: Angular velocity (3-axis)
- **Magnetometer**: Magnetic field (3-axis)

```
IMU Data Fusion → Orientation Estimate
     ↑
Complementary Filter / Kalman Filter
```

#### Current Sensors

Measure motor current to estimate torque:
$$
\tau = K_t \cdot i
$$
Where $K_t$ is the torque constant and $i$ is current.

### Exteroceptive Sensors

These sensors perceive the external environment.

#### Cameras

**RGB Cameras**
- Standard visual information
- Used for object recognition, visual servoing
- Frame rates: 30-120 fps typical

**Depth Cameras**
- Intel RealSense, Microsoft Azure Kinect
- Technologies: Structured light, Time-of-Flight
- Range: 0.3m - 10m typical

**Event Cameras**
- Asynchronous, high dynamic range
- Microsecond temporal resolution
- Emerging technology for high-speed robotics

#### LIDAR (Light Detection and Ranging)

Measures distance using laser light:

```
Types:
├── 2D LIDAR (single plane scan)
│   └── Good for navigation, obstacle avoidance
└── 3D LIDAR (volumetric scanning)
    └── Full environment mapping
```

| Specification | 2D LIDAR | 3D LIDAR |
|--------------|----------|----------|
| Range | 10-30m | 100-200m |
| Points/sec | 10K-40K | 300K-2M |
| Cost | $100-$500 | $1K-$75K |

#### Force/Torque Sensors

Measure interaction forces:
- 6-axis F/T sensors (ATI, OnRobot)
- Used for:
  - Contact detection
  - Force control
  - Compliance
  - Safe human interaction

#### Tactile Sensors

Sense touch and pressure:
- Resistive, capacitive, piezoelectric
- Emerging: Vision-based tactile (GelSight)
- Applications: Grasping, manipulation

#### Ultrasonic Sensors

Sound-based distance measurement:
- Low cost
- Works in dust/fog
- Lower resolution than LIDAR/cameras

## Actuators: The Robot's Muscles

### Electric Motors

#### DC Motors

Simple, widely used:
- Easy to control
- Good for low-cost applications
- Require gear reduction for torque

#### Brushless DC Motors (BLDC)

More efficient and durable:
- No brush wear
- Higher power density
- Used in drones, EVs, robotics

#### Servo Motors

Integrated motor + encoder + driver:
- Precise position control
- Built-in feedback loop
- Common in industrial robots

#### Stepper Motors

Discrete angular steps:
- No feedback needed for positioning
- Lower speeds
- Common in 3D printers, CNC

### Transmission Systems

#### Gear Reduction

Increases torque at the expense of speed:
$$
\tau_{out} = N \cdot \tau_{in}
$$

$$
\omega_{out} = \frac{\omega_{in}}{N}
$$

Where $N$ is the gear ratio.

#### Harmonic Drives

- Zero backlash
- High reduction ratios (30:1 - 320:1)
- Compact
- Used in robot joints, space applications

#### Belt/Chain Drives

- Allows spatial separation of motor and joint
- Lower stiffness than gears
- Quieter operation

### Advanced Actuators

#### Series Elastic Actuators (SEA)

Include a compliant element in series:
- Safer human interaction
- Force sensing via spring deflection
- Shock absorption

```
Motor → Gearbox → Spring → Output
                    ↑
            Deflection sensor
```

#### Quasi-Direct Drive

High-torque motors with minimal gearing:
- High backdrivability
- Fast force control
- MIT Cheetah, Ghost Robotics

#### Hydraulic Actuators

Fluid-powered for high force:
- Excellent power-to-weight ratio
- Boston Dynamics Atlas
- Complex fluid systems required

## Sensor Fusion

Combining multiple sensors for better perception:

```
┌─────────┐   ┌─────────┐   ┌─────────┐
│ Camera  │   │  IMU    │   │  LIDAR  │
└────┬────┘   └────┬────┘   └────┬────┘
     │             │             │
     └─────────────┴─────────────┘
                   │
          ┌────────┴────────┐
          │  Sensor Fusion  │
          │  (Kalman Filter)│
          └────────┬────────┘
                   │
          ┌────────┴────────┐
          │  Fused State    │
          │  Estimate       │
          └─────────────────┘
```

### Extended Kalman Filter (EKF)

Standard approach for nonlinear state estimation:
- Predict step: Use motion model
- Update step: Incorporate sensor measurements
- Handles uncertainty in measurements

## Practical Considerations

### Selection Criteria

| Factor | Sensors | Actuators |
|--------|---------|-----------|
| Accuracy | Resolution, noise | Positioning repeatability |
| Speed | Update rate | Response time |
| Range | Measurement range | Motion range |
| Cost | Unit cost, maintenance | Motor + driver cost |
| Integration | Interface (I2C, SPI, USB) | Power requirements |

### Common Pitfalls

1. **Sensor noise**: Always filter raw measurements
2. **Calibration drift**: Regular recalibration needed
3. **EMI interference**: Shield cables, proper grounding
4. **Thermal effects**: Temperature compensation

## Key Takeaways

1. **Proprioceptive sensors** measure internal state (encoders, IMU)
2. **Exteroceptive sensors** perceive environment (cameras, LIDAR)
3. **Electric motors** are dominant but hydraulics offer high force
4. **Transmission systems** trade speed for torque
5. **Sensor fusion** combines multiple sensors for robust perception

---

*Next: [Robot Kinematics](./robot-kinematics.md) →*
