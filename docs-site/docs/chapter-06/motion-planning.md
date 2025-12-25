# Motion Planning

Motion planning is the computational problem of finding a sequence of valid configurations that moves a robot from a start state to a goal state while avoiding obstacles. This is fundamental for humanoid robots navigating complex environments.

## What is Motion Planning?

### The Planning Problem

Given:
- **Start configuration**: Robot's initial state
- **Goal configuration**: Desired final state
- **Environment**: Obstacles and constraints
- **Robot model**: Kinematics and geometry

Find:
- **Valid path**: Collision-free trajectory from start to goal

```
┌─────────────────────────────────────────────────────────────────┐
│                    Motion Planning Problem                       │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│    Start ●                        ○ Goal                        │
│            \                     /                               │
│             \   ┌───────┐      /                                │
│              \  │       │     /                                 │
│               \ │Obstacle│   /                                  │
│                \└───────┘  /                                    │
│                 \        /                                      │
│                  ●──────●                                       │
│                    Path                                         │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

### Configuration Space (C-Space)

The **configuration space** represents all possible robot configurations:

| DOF | Configuration Space | Example |
|-----|---------------------|---------|
| 2D point | $\mathbb{R}^2$ | Mobile robot position |
| 2D robot with rotation | $\mathbb{R}^2 \times SO(2)$ | Mobile robot |
| 6-DOF arm | $\mathbb{R}^6$ or $T^6$ | Robotic manipulator |
| Humanoid | $\mathbb{R}^{30+}$ | Full-body motion |

**C-space decomposition**:
- **C-free**: Valid configurations (no collisions)
- **C-obstacle**: Invalid configurations (collision)

## Sampling-Based Planning

For high-dimensional spaces like humanoids, sampling-based methods are essential.

### Probabilistic Roadmap (PRM)

PRM builds a graph of valid configurations:

```python
import numpy as np

class PRM:
    """
    Probabilistic Roadmap planner.
    """
    
    def __init__(self, config_space, collision_checker):
        self.c_space = config_space
        self.collision = collision_checker
        self.vertices = []
        self.edges = []
    
    def build_roadmap(self, n_samples=1000, k_neighbors=10):
        """
        Build roadmap by sampling and connecting.
        """
        # Phase 1: Sample valid configurations
        while len(self.vertices) < n_samples:
            q = self.c_space.sample_random()
            if not self.collision.check(q):
                self.vertices.append(q)
        
        # Phase 2: Connect neighbors
        for v in self.vertices:
            neighbors = self.find_k_nearest(v, k_neighbors)
            for n in neighbors:
                if self.collision.check_path(v, n):
                    self.edges.append((v, n))
    
    def query(self, start, goal):
        """
        Find path from start to goal using roadmap.
        """
        # Connect start and goal to roadmap
        # Use graph search (A*, Dijkstra) to find path
        pass
```

**Advantages**:
- Good for multiple queries
- Captures environment structure

**Disadvantages**:
- Pre-computation required
- Memory for storing roadmap

### Rapidly-exploring Random Tree (RRT)

RRT grows a tree by random exploration:

```python
class RRT:
    """
    Rapidly-exploring Random Tree planner.
    """
    
    def __init__(self, config_space, collision_checker, step_size=0.1):
        self.c_space = config_space
        self.collision = collision_checker
        self.step_size = step_size
        self.tree = []
    
    def plan(self, start, goal, max_iterations=10000):
        """
        Plan path from start to goal.
        """
        self.tree = [{'config': start, 'parent': None}]
        
        for i in range(max_iterations):
            # Sample random configuration (with goal bias)
            if np.random.random() < 0.05:
                q_rand = goal
            else:
                q_rand = self.c_space.sample_random()
            
            # Find nearest node in tree
            nearest = self.find_nearest(q_rand)
            
            # Extend toward random sample
            q_new = self.extend(nearest['config'], q_rand)
            
            # Check collision
            if not self.collision.check_path(nearest['config'], q_new):
                self.tree.append({
                    'config': q_new,
                    'parent': nearest
                })
                
                # Check if we reached the goal
                if self.distance(q_new, goal) < self.step_size:
                    return self.extract_path(self.tree[-1])
        
        return None  # No path found
    
    def extend(self, q_from, q_to):
        """
        Extend from q_from toward q_to by step_size.
        """
        direction = q_to - q_from
        distance = np.linalg.norm(direction)
        if distance < self.step_size:
            return q_to
        return q_from + (direction / distance) * self.step_size
```

### RRT* (Optimal RRT)

RRT* improves on RRT by rewiring the tree for shorter paths:

```
┌─────────────────────────────────────────────────────────────────┐
│                      RRT vs RRT*                                 │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│     RRT (First solution)         RRT* (Optimized)               │
│                                                                  │
│     ●───●                        ●───●                          │
│          \                            \                          │
│           ●──●                         ●                         │
│               \                         \                        │
│                ●                         ●                       │
│                 \                       /                        │
│                  ●●○                   ○                         │
│                                       (shorter path)             │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

## Grid-Based Planning

For discrete environments or when completeness guarantees are needed.

### A* Algorithm

A* finds optimal paths on graphs:

```python
import heapq

def astar(start, goal, get_neighbors, heuristic):
    """
    A* search algorithm.
    
    Args:
        start: Start state
        goal: Goal state
        get_neighbors: Function returning neighbors and costs
        heuristic: Heuristic function h(state)
    
    Returns:
        Path from start to goal
    """
    open_set = [(0, start)]
    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}
    
    while open_set:
        _, current = heapq.heappop(open_set)
        
        if current == goal:
            return reconstruct_path(came_from, current)
        
        for neighbor, cost in get_neighbors(current):
            tentative_g = g_score[current] + cost
            
            if neighbor not in g_score or tentative_g < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score[neighbor] = tentative_g + heuristic(neighbor, goal)
                heapq.heappush(open_set, (f_score[neighbor], neighbor))
    
    return None  # No path found
```

### Common Heuristics

| Heuristic | Formula | Use Case |
|-----------|---------|----------|
| Euclidean | $\sqrt{\sum(x_i - g_i)^2}$ | Any direction movement |
| Manhattan | $\sum|x_i - g_i|$ | Grid with 4 directions |
| Chebyshev | $\max|x_i - g_i|$ | Grid with 8 directions |

## Trajectory Optimization

For smooth, dynamically feasible motions.

### Optimization Formulation

$$\min_{x_{0:T}, u_{0:T-1}} \sum_{t=0}^{T-1} c(x_t, u_t) + c_T(x_T)$$

Subject to:
- $x_{t+1} = f(x_t, u_t)$ (dynamics)
- $x_0 = x_{start}$ (initial condition)
- $x_T = x_{goal}$ (goal condition)
- $x_t \in \mathcal{X}_{free}$ (collision avoidance)
- $u_t \in \mathcal{U}$ (input limits)

### CHOMP (Covariant Hamiltonian Optimization)

```python
class CHOMP:
    """
    Covariant Hamiltonian Optimization for Motion Planning.
    """
    
    def __init__(self, robot, environment):
        self.robot = robot
        self.env = environment
    
    def optimize(self, initial_trajectory, iterations=100):
        """
        Optimize trajectory to minimize cost.
        """
        trajectory = initial_trajectory.copy()
        
        for i in range(iterations):
            # Compute smoothness cost gradient
            smooth_grad = self.smoothness_gradient(trajectory)
            
            # Compute obstacle cost gradient
            obstacle_grad = self.obstacle_gradient(trajectory)
            
            # Update trajectory
            trajectory -= self.learning_rate * (
                smooth_grad + self.obstacle_weight * obstacle_grad
            )
            
            # Project to satisfy constraints
            trajectory = self.project_constraints(trajectory)
        
        return trajectory
```

### TrajOpt (Trajectory Optimization)

Key features:
- Sequential convex optimization
- Collision costs via signed distance
- Joint limit handling

## Motion Planning for Humanoids

### Whole-Body Motion Planning

Humanoid planning must consider:
- Balance constraints
- Many degrees of freedom
- Contact dynamics

```
┌─────────────────────────────────────────────────────────────────┐
│              Humanoid Motion Planning Stack                      │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  ┌─────────────────────────────────────────────────────────┐    │
│  │  Task Planning: What to do (grasp object, walk there)   │    │
│  └────────────────────────┬────────────────────────────────┘    │
│                           │                                      │
│                           ▼                                      │
│  ┌─────────────────────────────────────────────────────────┐    │
│  │  Contact Planning: Where/when to make contacts          │    │
│  └────────────────────────┬────────────────────────────────┘    │
│                           │                                      │
│                           ▼                                      │
│  ┌─────────────────────────────────────────────────────────┐    │
│  │  Motion Planning: Joint trajectories                    │    │
│  └────────────────────────┬────────────────────────────────┘    │
│                           │                                      │
│                           ▼                                      │
│  ┌─────────────────────────────────────────────────────────┐    │
│  │  Trajectory Optimization: Smooth, feasible paths        │    │
│  └─────────────────────────────────────────────────────────┘    │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

### Footstep Planning

For bipedal locomotion:

```python
class FootstepPlanner:
    """
    Plan sequence of footsteps for walking.
    """
    
    def __init__(self, robot_params):
        self.step_length = robot_params['max_step_length']
        self.step_width = robot_params['step_width']
    
    def plan_footsteps(self, start_pose, goal_pose):
        """
        Generate footstep sequence.
        """
        footsteps = []
        current = start_pose
        foot = 'left'  # Start with left foot
        
        while self.distance(current, goal_pose) > self.step_length:
            # Compute next footstep
            next_step = self.compute_next_step(current, goal_pose, foot)
            footsteps.append(next_step)
            
            current = next_step
            foot = 'right' if foot == 'left' else 'left'
        
        return footsteps
```

## MoveIt 2

MoveIt is the standard motion planning framework for ROS2:

### Setup

```bash
# Install MoveIt 2
sudo apt install ros-humble-moveit

# Generate robot configuration
ros2 run moveit_setup_assistant moveit_setup_assistant
```

### Using MoveIt in Python

```python
from moveit_msgs.msg import MoveItErrorCodes
from moveit.planning import MoveItPy

def plan_and_execute():
    """
    Example of using MoveIt for motion planning.
    """
    # Initialize MoveIt
    moveit = MoveItPy(node_name="moveit_example")
    arm = moveit.get_planning_component("arm")
    
    # Set planning scene
    arm.set_start_state_to_current_state()
    
    # Set goal
    arm.set_goal_state(configuration_name="home")
    
    # Plan
    plan_result = arm.plan()
    
    if plan_result:
        # Execute
        arm.execute()
```

### Planning Pipeline

```
┌─────────────────────────────────────────────────────────────────┐
│                    MoveIt Planning Pipeline                      │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  Request ─▶ Planner ─▶ Path ─▶ Post-processing ─▶ Trajectory    │
│                                                                  │
│  Planners: OMPL (RRT, PRM, etc.), STOMP, CHOMP                  │
│  Post-processing: Time parameterization, smoothing              │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

## Summary

Motion planning is essential for autonomous humanoid robots:

| Method | Pros | Cons | Best For |
|--------|------|------|----------|
| **PRM** | Multi-query, global | Memory, preprocessing | Static environments |
| **RRT** | Single-query, fast | Suboptimal | High-DOF spaces |
| **A*** | Optimal, complete | Exponential in dimensions | Low-DOF, discrete |
| **Trajectory Opt** | Smooth, constraints | Local optima | Refinement |

Humanoid motion planning requires combining these methods with balance and contact constraints for whole-body motion.

## Key Concepts

- **Configuration Space**: Space of all robot states
- **Collision Checking**: Determining valid configurations
- **Sampling**: Exploring C-space efficiently
- **Optimization**: Refining paths for quality
