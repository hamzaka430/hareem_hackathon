# Control Systems

Control systems enable robots to track desired trajectories, maintain stability, and respond to disturbances. This section covers the control theory essential for humanoid robotics.

## Fundamentals of Control

### Open-Loop vs Closed-Loop Control

```
┌─────────────────────────────────────────────────────────────────┐
│                    Open-Loop Control                             │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│   Reference ──▶ Controller ──▶ Plant ──▶ Output                │
│                                                                  │
│   No feedback - cannot correct for disturbances                 │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────┐
│                    Closed-Loop Control                           │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│   Reference ──▶ ⊕ ──▶ Controller ──▶ Plant ──▶ Output          │
│                ▲                              │                  │
│                │                              │                  │
│                └────── Sensor ◀───────────────┘                  │
│                      (Feedback)                                  │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

### Control System Terminology

| Term | Symbol | Description |
|------|--------|-------------|
| **Reference** | $r(t)$ | Desired setpoint |
| **Output** | $y(t)$ | Measured system state |
| **Error** | $e(t) = r(t) - y(t)$ | Difference from reference |
| **Control Input** | $u(t)$ | Command to actuator |
| **Disturbance** | $d(t)$ | External interference |

## PID Control

The most widely used control algorithm:

### PID Equation

$$u(t) = K_p e(t) + K_i \int_0^t e(\tau) d\tau + K_d \frac{de(t)}{dt}$$

Where:
- $K_p$: Proportional gain (reacts to current error)
- $K_i$: Integral gain (eliminates steady-state error)
- $K_d$: Derivative gain (predicts future error)

### Digital PID Implementation

```python
class PIDController:
    """
    Discrete PID controller implementation.
    """
    
    def __init__(
        self,
        kp: float,
        ki: float,
        kd: float,
        dt: float,
        output_limits: tuple = (None, None)
    ):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.output_min, self.output_max = output_limits
        
        # State
        self.integral = 0.0
        self.prev_error = 0.0
    
    def compute(self, setpoint: float, measurement: float) -> float:
        """
        Compute control output.
        """
        error = setpoint - measurement
        
        # Proportional term
        p_term = self.kp * error
        
        # Integral term with anti-windup
        self.integral += error * self.dt
        i_term = self.ki * self.integral
        
        # Derivative term (on error)
        derivative = (error - self.prev_error) / self.dt
        d_term = self.kd * derivative
        
        self.prev_error = error
        
        # Total output
        output = p_term + i_term + d_term
        
        # Apply output limits
        if self.output_max is not None:
            output = min(output, self.output_max)
        if self.output_min is not None:
            output = max(output, self.output_min)
        
        return output
    
    def reset(self):
        """Reset controller state."""
        self.integral = 0.0
        self.prev_error = 0.0
```

### PID Tuning

Common tuning methods:

| Method | Description |
|--------|-------------|
| **Manual** | Iterative adjustment |
| **Ziegler-Nichols** | Based on critical gain |
| **Cohen-Coon** | Based on step response |
| **Auto-tuning** | Automated identification |

```python
def ziegler_nichols_tuning(ku: float, tu: float) -> dict:
    """
    Ziegler-Nichols tuning from ultimate gain and period.
    
    Args:
        ku: Ultimate gain (gain at sustained oscillation)
        tu: Ultimate period (oscillation period)
    
    Returns:
        PID gains
    """
    return {
        'P':   {'kp': 0.5 * ku, 'ki': 0, 'kd': 0},
        'PI':  {'kp': 0.45 * ku, 'ki': 0.54 * ku / tu, 'kd': 0},
        'PID': {'kp': 0.6 * ku, 'ki': 1.2 * ku / tu, 'kd': 0.075 * ku * tu}
    }
```

## State-Space Control

### State-Space Representation

Linear systems in state-space form:

$$\dot{x} = Ax + Bu$$
$$y = Cx + Du$$

Where:
- $x$: State vector
- $u$: Input vector
- $y$: Output vector
- $A$: System matrix
- $B$: Input matrix
- $C$: Output matrix
- $D$: Feedthrough matrix

### Full State Feedback

```
┌─────────────────────────────────────────────────────────────────┐
│                   Full State Feedback                            │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│   r ──▶ ⊕ ────▶ Plant ────▶ x ────▶ y                          │
│         ▲                   │                                    │
│         │       ┌───┐       │                                    │
│         └───────│-K │◀──────┘                                    │
│                 └───┘                                            │
│                                                                  │
│   u = -Kx                                                        │
│   Closed-loop: ẋ = (A - BK)x                                    │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

### Pole Placement

Design K to place closed-loop poles:

```python
import numpy as np
from scipy import signal

def pole_placement(A, B, desired_poles):
    """
    Compute feedback gain for desired pole locations.
    
    Args:
        A: System matrix (n x n)
        B: Input matrix (n x m)
        desired_poles: List of n desired eigenvalues
    
    Returns:
        K: Feedback gain matrix
    """
    # Use Ackermann's formula or place algorithm
    K = signal.place_poles(A, B, desired_poles).gain_matrix
    return K

# Example: Double integrator
A = np.array([[0, 1], [0, 0]])
B = np.array([[0], [1]])
desired_poles = [-2, -3]  # Stable, real poles

K = pole_placement(A, B, desired_poles)
```

### Linear Quadratic Regulator (LQR)

Optimal state feedback minimizing:

$$J = \int_0^\infty (x^T Q x + u^T R u) dt$$

```python
from scipy.linalg import solve_continuous_are

def lqr(A, B, Q, R):
    """
    Solve continuous-time LQR problem.
    
    Args:
        A, B: System matrices
        Q: State cost matrix (penalizes state deviation)
        R: Input cost matrix (penalizes control effort)
    
    Returns:
        K: Optimal feedback gain
        P: Solution to Riccati equation
    """
    # Solve Algebraic Riccati Equation
    P = solve_continuous_are(A, B, Q, R)
    
    # Compute optimal gain
    K = np.linalg.inv(R) @ B.T @ P
    
    return K, P

# Example usage
Q = np.diag([10, 1])  # Penalize position more than velocity
R = np.array([[1]])    # Control effort cost

K, P = lqr(A, B, Q, R)
```

## Model Predictive Control (MPC)

MPC optimizes over a future horizon:

### MPC Formulation

At each timestep, solve:

$$
\min_{u_{0:N-1}} \sum_{k=0}^{N-1} \ell(x_k, u_k) + \ell_N(x_N)
$$

Subject to:
- $x_{k+1} = f(x_k, u_k)$ (dynamics)
- $x_k \in \mathcal{X}$ (state constraints)
- $u_k \in \mathcal{U}$ (input constraints)
- $x_0 = x_{current}$ (initial state)

### MPC Implementation

```python
import cvxpy as cp
import numpy as np

class LinearMPC:
    """
    Linear Model Predictive Controller.
    """
    
    def __init__(
        self,
        A: np.ndarray,
        B: np.ndarray,
        Q: np.ndarray,
        R: np.ndarray,
        N: int,
        x_min: np.ndarray = None,
        x_max: np.ndarray = None,
        u_min: np.ndarray = None,
        u_max: np.ndarray = None
    ):
        self.A = A
        self.B = B
        self.Q = Q
        self.R = R
        self.N = N  # Horizon
        
        self.nx = A.shape[0]
        self.nu = B.shape[1]
        
        # Constraints
        self.x_min = x_min if x_min is not None else -np.inf * np.ones(self.nx)
        self.x_max = x_max if x_max is not None else np.inf * np.ones(self.nx)
        self.u_min = u_min if u_min is not None else -np.inf * np.ones(self.nu)
        self.u_max = u_max if u_max is not None else np.inf * np.ones(self.nu)
    
    def solve(self, x0: np.ndarray, x_ref: np.ndarray = None) -> np.ndarray:
        """
        Solve MPC problem and return optimal control.
        """
        if x_ref is None:
            x_ref = np.zeros(self.nx)
        
        # Decision variables
        x = cp.Variable((self.N + 1, self.nx))
        u = cp.Variable((self.N, self.nu))
        
        # Cost
        cost = 0
        constraints = [x[0] == x0]
        
        for k in range(self.N):
            # Stage cost
            cost += cp.quad_form(x[k] - x_ref, self.Q)
            cost += cp.quad_form(u[k], self.R)
            
            # Dynamics constraint
            constraints.append(x[k+1] == self.A @ x[k] + self.B @ u[k])
            
            # State constraints
            constraints.append(x[k] >= self.x_min)
            constraints.append(x[k] <= self.x_max)
            
            # Input constraints
            constraints.append(u[k] >= self.u_min)
            constraints.append(u[k] <= self.u_max)
        
        # Terminal cost
        cost += cp.quad_form(x[self.N] - x_ref, self.Q)
        
        # Solve
        problem = cp.Problem(cp.Minimize(cost), constraints)
        problem.solve()
        
        # Return first control input
        return u.value[0]
```

### MPC for Humanoid Robots

```
┌─────────────────────────────────────────────────────────────────┐
│              MPC for Humanoid Walking                            │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  Optimize:                                                       │
│  - CoM trajectory                                                │
│  - Footstep locations (optional)                                │
│                                                                  │
│  Constraints:                                                    │
│  - ZMP within support polygon                                    │
│  - Joint limits                                                  │
│  - Torque limits                                                 │
│  - Friction cones                                                │
│                                                                  │
│  Result: Smooth, stable walking motion                          │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

## Impedance Control

For safe interaction with environments and humans:

### Impedance Model

Target behavior:

$$M_d \ddot{x} + D_d \dot{x} + K_d x = F_{ext}$$

Where:
- $M_d$: Desired inertia
- $D_d$: Desired damping
- $K_d$: Desired stiffness
- $F_{ext}$: External force

```python
class ImpedanceController:
    """
    Cartesian impedance controller.
    """
    
    def __init__(
        self,
        M_d: np.ndarray,  # Desired inertia
        D_d: np.ndarray,  # Desired damping
        K_d: np.ndarray   # Desired stiffness
    ):
        self.M_d = M_d
        self.D_d = D_d
        self.K_d = K_d
    
    def compute_force(
        self,
        x: np.ndarray,      # Current position
        x_dot: np.ndarray,  # Current velocity
        x_d: np.ndarray,    # Desired position
        x_dot_d: np.ndarray,# Desired velocity
        x_ddot_d: np.ndarray # Desired acceleration
    ) -> np.ndarray:
        """
        Compute commanded force for impedance behavior.
        """
        # Position and velocity errors
        e = x_d - x
        e_dot = x_dot_d - x_dot
        
        # Impedance control law
        F = (self.K_d @ e + 
             self.D_d @ e_dot + 
             self.M_d @ x_ddot_d)
        
        return F
```

### Admittance Control

Dual of impedance control—takes force input, outputs motion:

$$x = \frac{F_{ext}}{M_d s^2 + D_d s + K_d}$$

## Whole-Body Control

For humanoid robots with many DOF:

### Task Hierarchy

```
┌─────────────────────────────────────────────────────────────────┐
│              Whole-Body Control Task Stack                       │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  Priority 1 (Highest): Balance / Don't Fall                     │
│  ┌─────────────────────────────────────────────────────────┐    │
│  │  Keep ZMP in support polygon                             │    │
│  └─────────────────────────────────────────────────────────┘    │
│                           ↓                                      │
│  Priority 2: Contact Constraints                                │
│  ┌─────────────────────────────────────────────────────────┐    │
│  │  Feet stay in contact with ground                        │    │
│  └─────────────────────────────────────────────────────────┘    │
│                           ↓                                      │
│  Priority 3: Task Objective                                     │
│  ┌─────────────────────────────────────────────────────────┐    │
│  │  Hand reaches target position                            │    │
│  └─────────────────────────────────────────────────────────┘    │
│                           ↓                                      │
│  Priority 4: Posture                                            │
│  ┌─────────────────────────────────────────────────────────┐    │
│  │  Maintain comfortable joint configuration                │    │
│  └─────────────────────────────────────────────────────────┘    │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

### Quadratic Programming Formulation

```python
def whole_body_controller(
    robot,
    task_objectives,
    constraints
):
    """
    Solve whole-body control as QP.
    
    minimize:   Σ w_i ||J_i q̈ + J̇_i q̇ - ẍ_i_desired||²
    subject to: Dynamics: M q̈ + h = S^T τ + J_c^T f
                Contact: f in friction cone
                Joint limits: q_min ≤ q ≤ q_max
                Torque limits: τ_min ≤ τ ≤ τ_max
    """
    # Decision variables: q̈, τ, f (contact forces)
    # Solve QP to find optimal accelerations and torques
    pass
```

## Summary

Control systems are essential for humanoid robots:

| Controller | Use Case | Complexity |
|------------|----------|------------|
| **PID** | Single joints, simple tasks | Low |
| **LQR** | Optimal regulation | Medium |
| **MPC** | Constrained, preview control | High |
| **Impedance** | Safe interaction | Medium |
| **Whole-Body** | Full humanoid control | Very High |

Modern humanoid control combines these approaches in hierarchical architectures.

## Key Equations

| Concept | Equation |
|---------|----------|
| PID | $u = K_p e + K_i \int e + K_d \dot{e}$ |
| State Feedback | $u = -Kx$ |
| LQR Cost | $J = \int (x^T Q x + u^T R u) dt$ |
| Impedance | $M_d \ddot{x} + D_d \dot{x} + K_d x = F_{ext}$ |
