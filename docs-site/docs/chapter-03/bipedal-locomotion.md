# Bipedal Locomotion

Bipedal locomotion—walking on two legs—is one of the defining challenges in humanoid robotics. This section explores the principles, methods, and technologies that enable robots to walk like humans.

## Understanding Human Walking

### The Gait Cycle

Human walking consists of a repeating **gait cycle**:

```
┌─────────────────────────────────────────────────────────────────┐
│                      GAIT CYCLE (100%)                          │
├─────────────────────────────────┬───────────────────────────────┤
│      STANCE PHASE (60%)         │      SWING PHASE (40%)        │
├─────────────────────────────────┼───────────────────────────────┤
│ • Heel Strike                   │ • Toe Off                     │
│ • Foot Flat                     │ • Mid-Swing                   │
│ • Mid-Stance                    │ • Terminal Swing              │
│ • Heel Off                      │                               │
└─────────────────────────────────┴───────────────────────────────┘
```

### Key Biomechanical Concepts

| Concept | Description |
|---------|-------------|
| **Center of Mass (CoM)** | Point where body mass is concentrated |
| **Center of Pressure (CoP)** | Point where ground reaction force acts |
| **Zero Moment Point (ZMP)** | Point where horizontal moments are zero |
| **Support Polygon** | Area enclosed by feet in contact with ground |

## Control Strategies for Bipedal Walking

### 1. Zero Moment Point (ZMP) Control

The most widely used approach for humanoid walking:

**Principle**: Keep the ZMP within the support polygon to prevent falling.

**Advantages**:
- Mathematically well-defined
- Works well on flat terrain
- Proven in ASIMO, HRP series

**Limitations**:
- Assumes flat ground contact
- Conservative walking patterns
- Cannot handle large disturbances

### 2. Capture Point Control

A more dynamic approach based on the concept of "capture points":

```python
# Simplified Capture Point Calculation
# CP = CoM_position + CoM_velocity / omega_0
# where omega_0 = sqrt(g / h)

import numpy as np

def calculate_capture_point(com_pos, com_vel, height, g=9.81):
    """
    Calculate the capture point for balance recovery.
    
    Args:
        com_pos: Center of mass position [x, y]
        com_vel: Center of mass velocity [vx, vy]
        height: Height of center of mass
        g: Gravitational acceleration
    
    Returns:
        Capture point position [x, y]
    """
    omega_0 = np.sqrt(g / height)
    capture_point = com_pos + com_vel / omega_0
    return capture_point
```

### 3. Model Predictive Control (MPC)

Optimization-based approach that plans footsteps ahead:

- **Preview horizon**: Plans 1-2 seconds into future
- **Cost function**: Minimizes tracking error and energy
- **Constraints**: ZMP limits, joint limits, friction cones

### 4. Reinforcement Learning Approaches

Modern deep RL methods for locomotion:

- **Policy Gradient Methods**: PPO, SAC
- **Sim-to-Real Transfer**: Training in simulation
- **Reward Shaping**: Encouraging natural gaits

## Walking Pattern Generation

### Cart-Table Model

A simplified model treating the humanoid as an inverted pendulum on a cart:

```
      CoM (mass point)
         ●
        /|
       / |
      /  | height (h)
     /   |
────●────────────
   ZMP   Floor
```

The dynamics are described by:
```
ẍ_CoM = (g/h) * (x_CoM - x_ZMP)
```

### Preview Control

Algorithm for generating walking patterns:

1. **Define ZMP trajectory** based on footstep plan
2. **Use preview control** to calculate CoM trajectory
3. **Compute joint angles** via inverse kinematics
4. **Execute trajectory** with feedback control

## Dynamic Walking vs Static Walking

### Static Walking

- CoM always within support polygon
- Very slow (tortoise-like)
- Very stable
- Used in early humanoids

### Dynamic Walking

- CoM may leave support polygon
- Fast, natural-looking gait
- Requires active balance
- Used in modern humanoids

| Aspect | Static | Dynamic |
|--------|--------|---------|
| Speed | Slow | Fast |
| Stability | Inherent | Active |
| Energy | High | Lower |
| Natural | No | Yes |

## Walking on Various Terrains

### Stairs and Slopes

Additional considerations for non-flat terrain:

```python
class TerrainAdaptation:
    def adjust_for_stairs(self, step_height, step_depth):
        """
        Modify gait parameters for stair climbing.
        """
        # Increase hip and knee flexion
        # Adjust CoM trajectory
        # Modify swing foot trajectory
        pass
    
    def adjust_for_slope(self, slope_angle):
        """
        Modify gait for inclined surfaces.
        """
        # Adjust ankle angle
        # Shift CoM forward/backward
        # Modify step length
        pass
```

### Uneven Terrain

Strategies for rough terrain:
- **Terrain mapping** using vision
- **Foot placement optimization**
- **Compliance and force control**
- **Reactive stepping**

## Balance Recovery

When a humanoid is pushed or trips, it must recover balance:

### Recovery Strategies

1. **Ankle Strategy**: Small perturbations, feet stay in place
2. **Hip Strategy**: Larger perturbations, bending at waist
3. **Stepping Strategy**: Large perturbations, taking a step
4. **Reaching Strategy**: Using arms for balance

```
Perturbation Magnitude
│
│  ┌─────────────┐
│  │   Ankle     │ Small
│  │   Strategy  │
│  ├─────────────┤
│  │    Hip      │ Medium
│  │   Strategy  │
│  ├─────────────┤
│  │  Stepping   │ Large
│  │   Strategy  │
│  └─────────────┘
▼
```

## State-of-the-Art Examples

### Boston Dynamics Atlas

- Whole-body MPC
- Hydraulic actuators
- Real-time terrain perception
- Demonstrates running, jumping, parkour

### Agility Robotics Digit

- Electric actuators
- Spring-loaded legs for efficiency
- Designed for commercial deployment
- Focus on reliability and repeatability

### Tesla Optimus

- Electric actuators
- AI-driven control
- Designed for manufacturing environments
- Learning-based locomotion

## Summary

Bipedal locomotion remains one of the most challenging problems in robotics. Success requires:
- Accurate dynamic models
- Robust control algorithms
- High-performance actuators
- Real-time sensing and computation
- Integration of learning-based methods

The field continues to advance rapidly, with robots now capable of running, jumping, and navigating complex terrain.

## Key Equations

| Concept | Equation |
|---------|----------|
| Natural Frequency | $\omega_0 = \sqrt{g/h}$ |
| Capture Point | $x_{CP} = x_{CoM} + \dot{x}_{CoM}/\omega_0$ |
| Linear Inverted Pendulum | $\ddot{x}_{CoM} = \omega_0^2 (x_{CoM} - x_{ZMP})$ |
