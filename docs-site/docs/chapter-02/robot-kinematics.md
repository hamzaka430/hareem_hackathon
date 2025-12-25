---
sidebar_position: 3
---

# Robot Kinematics

Kinematics is the study of motion without considering the forces that cause it. For robots, kinematics helps us understand the relationship between joint configurations and the robot's position in space.

## Forward Kinematics

Given joint positions/angles, determine the end-effector pose.

### Denavit-Hartenberg (DH) Convention

A systematic way to describe robot geometry using four parameters per joint:

| Parameter | Symbol | Description |
|-----------|--------|-------------|
| Link length | $a_i$ | Distance along $x_i$ from $z_{i-1}$ to $z_i$ |
| Link twist | $\alpha_i$ | Angle from $z_{i-1}$ to $z_i$ about $x_i$ |
| Link offset | $d_i$ | Distance along $z_{i-1}$ from $x_{i-1}$ to $x_i$ |
| Joint angle | $\theta_i$ | Angle from $x_{i-1}$ to $x_i$ about $z_{i-1}$ |

### DH Transformation Matrix

Each joint contributes a transformation:

$$
T_i = \begin{bmatrix}
\cos\theta_i & -\sin\theta_i\cos\alpha_i & \sin\theta_i\sin\alpha_i & a_i\cos\theta_i \\
\sin\theta_i & \cos\theta_i\cos\alpha_i & -\cos\theta_i\sin\alpha_i & a_i\sin\theta_i \\
0 & \sin\alpha_i & \cos\alpha_i & d_i \\
0 & 0 & 0 & 1
\end{bmatrix}
$$

### Forward Kinematics Chain

The end-effector pose is computed by chaining transformations:

$$
T_0^n = T_0^1 \cdot T_1^2 \cdot T_2^3 \cdot ... \cdot T_{n-1}^n
$$

### Example: 2-Link Planar Arm

```
    L1        L2
○────────○────────→ End-effector
   θ1       θ2
```

End-effector position:
$$
x = L_1\cos\theta_1 + L_2\cos(\theta_1 + \theta_2)
$$
$$
y = L_1\sin\theta_1 + L_2\sin(\theta_1 + \theta_2)
$$

## Inverse Kinematics

Given desired end-effector pose, find joint configurations.

### Challenges

1. **Multiple solutions**: Different joint configurations can achieve same pose
2. **No solution**: Desired pose may be unreachable
3. **Singularities**: Configurations where movement becomes restricted

### Analytical Solutions

For simple robots, closed-form solutions exist:

**2-Link Arm IK**:
$$
\theta_2 = \cos^{-1}\left(\frac{x^2 + y^2 - L_1^2 - L_2^2}{2L_1L_2}\right)
$$

$$
\theta_1 = \tan^{-1}\left(\frac{y}{x}\right) - \tan^{-1}\left(\frac{L_2\sin\theta_2}{L_1 + L_2\cos\theta_2}\right)
$$

### Numerical Solutions

For complex robots, iterative methods are used:

#### Jacobian-Based Methods

The Jacobian relates joint velocities to end-effector velocities:
$$
\dot{x} = J(q) \cdot \dot{q}
$$

**Jacobian Pseudoinverse**:
$$
\dot{q} = J^+ \cdot \dot{x}
$$

Where $J^+ = J^T(JJ^T)^{-1}$ is the pseudoinverse.

**Damped Least Squares** (handles singularities):
$$
\dot{q} = J^T(JJ^T + \lambda^2 I)^{-1} \cdot \dot{x}
$$

#### Optimization-Based Methods

Formulate IK as an optimization problem:
$$
\min_q ||f(q) - x_{desired}||^2 + \lambda ||q - q_0||^2
$$

Libraries: KDL, Pinocchio, Drake

## Velocity Kinematics

### The Jacobian Matrix

For an $n$-DoF robot with 6-dimensional task space:

$$
J = \begin{bmatrix}
\frac{\partial x}{\partial q_1} & \frac{\partial x}{\partial q_2} & \cdots & \frac{\partial x}{\partial q_n} \\
\frac{\partial y}{\partial q_1} & \frac{\partial y}{\partial q_2} & \cdots & \frac{\partial y}{\partial q_n} \\
\vdots & \vdots & \ddots & \vdots \\
\frac{\partial \omega_z}{\partial q_1} & \frac{\partial \omega_z}{\partial q_2} & \cdots & \frac{\partial \omega_z}{\partial q_n}
\end{bmatrix}
$$

### Singularities

When $\det(J) = 0$, the robot is in a singular configuration:
- Loss of one or more degrees of freedom
- End-effector cannot move in certain directions
- Inverse Jacobian undefined

**Types of singularities**:
- **Boundary**: At workspace limits
- **Internal**: Within workspace (elbow extended)

## Workspace Analysis

### Reachable Workspace

All points the end-effector can reach with any orientation:
```
Reachable = {x : ∃q such that f(q) = x}
```

### Dexterous Workspace

Points reachable with any orientation:
```
Dexterous ⊆ Reachable
```

### Factors Affecting Workspace

- Link lengths
- Joint limits
- Joint types (revolute vs. prismatic)
- Number of DOFs

## Practical Implementation

### Python Example with NumPy

```python
import numpy as np

def forward_kinematics_2link(theta1, theta2, L1=1.0, L2=1.0):
    """Forward kinematics for 2-link planar arm"""
    x = L1 * np.cos(theta1) + L2 * np.cos(theta1 + theta2)
    y = L1 * np.sin(theta1) + L2 * np.sin(theta1 + theta2)
    return x, y

def inverse_kinematics_2link(x, y, L1=1.0, L2=1.0):
    """Inverse kinematics for 2-link planar arm"""
    # Check if point is reachable
    d = np.sqrt(x**2 + y**2)
    if d > L1 + L2 or d < abs(L1 - L2):
        return None  # Unreachable
    
    # Calculate theta2 (elbow)
    cos_theta2 = (x**2 + y**2 - L1**2 - L2**2) / (2 * L1 * L2)
    theta2 = np.arccos(np.clip(cos_theta2, -1, 1))
    
    # Calculate theta1 (shoulder)
    k1 = L1 + L2 * np.cos(theta2)
    k2 = L2 * np.sin(theta2)
    theta1 = np.arctan2(y, x) - np.arctan2(k2, k1)
    
    return theta1, theta2
```

### Using Robotics Libraries

**ROS2 + MoveIt2**:
```python
from moveit_msgs.msg import RobotState
from moveit2_py import MoveIt2

# Initialize MoveIt2
moveit = MoveIt2(robot_description="robot")

# Compute IK
pose_goal = [0.5, 0.2, 0.3, 0, 0, 0, 1]  # x, y, z, qx, qy, qz, qw
joint_solution = moveit.compute_ik(pose_goal)
```

## Key Takeaways

1. **Forward kinematics** maps joint space to task space (always solvable)
2. **Inverse kinematics** maps task space to joint space (may have multiple or no solutions)
3. **The Jacobian** relates velocities between spaces and is key for control
4. **Singularities** must be detected and handled carefully
5. **Workspace** defines the robot's reachable region

---

*Next Chapter: [Humanoid Robotics](../chapter-03/humanoid-overview.md) →*
