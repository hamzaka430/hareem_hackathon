# Path Planning Algorithms

This section provides a detailed exploration of path planning algorithms used in robotics, from classical graph search methods to modern sampling-based techniques.

## Graph-Based Algorithms

### Dijkstra's Algorithm

The foundation of many path planning methods:

```python
import heapq
from typing import Dict, List, Tuple, Optional

def dijkstra(
    graph: Dict[str, List[Tuple[str, float]]],
    start: str,
    goal: str
) -> Tuple[Optional[List[str]], float]:
    """
    Dijkstra's shortest path algorithm.
    
    Args:
        graph: Adjacency list {node: [(neighbor, cost), ...]}
        start: Start node
        goal: Goal node
    
    Returns:
        (path, cost) or (None, inf) if no path exists
    """
    # Priority queue: (cost, node)
    pq = [(0, start)]
    
    # Track best known cost to each node
    distances = {start: 0}
    
    # Track path
    previous = {}
    
    while pq:
        current_dist, current = heapq.heappop(pq)
        
        # Skip if we've found a better path
        if current_dist > distances.get(current, float('inf')):
            continue
        
        # Goal reached
        if current == goal:
            path = []
            node = goal
            while node in previous:
                path.append(node)
                node = previous[node]
            path.append(start)
            return path[::-1], current_dist
        
        # Explore neighbors
        for neighbor, edge_cost in graph.get(current, []):
            distance = current_dist + edge_cost
            
            if distance < distances.get(neighbor, float('inf')):
                distances[neighbor] = distance
                previous[neighbor] = current
                heapq.heappush(pq, (distance, neighbor))
    
    return None, float('inf')
```

**Complexity**: $O((V + E) \log V)$ with binary heap

### A* Algorithm

A* adds a heuristic to guide search toward the goal:

```python
def astar_search(
    start: Tuple[int, int],
    goal: Tuple[int, int],
    grid: List[List[int]],
    heuristic_func
) -> Optional[List[Tuple[int, int]]]:
    """
    A* search on a 2D grid.
    
    Args:
        start: Start position (row, col)
        goal: Goal position (row, col)
        grid: 2D grid where 0=free, 1=obstacle
        heuristic_func: Heuristic function h(pos, goal)
    
    Returns:
        Path as list of positions, or None
    """
    rows, cols = len(grid), len(grid[0])
    
    # f = g + h
    open_set = [(heuristic_func(start, goal), 0, start)]
    g_score = {start: 0}
    came_from = {}
    
    # 8-directional movement
    directions = [
        (-1, -1), (-1, 0), (-1, 1),
        (0, -1),          (0, 1),
        (1, -1),  (1, 0),  (1, 1)
    ]
    
    while open_set:
        f, g, current = heapq.heappop(open_set)
        
        if current == goal:
            # Reconstruct path
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            return path[::-1]
        
        for dr, dc in directions:
            neighbor = (current[0] + dr, current[1] + dc)
            
            # Check bounds
            if not (0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols):
                continue
            
            # Check obstacle
            if grid[neighbor[0]][neighbor[1]] == 1:
                continue
            
            # Cost: 1 for cardinal, sqrt(2) for diagonal
            move_cost = 1.414 if dr != 0 and dc != 0 else 1.0
            tentative_g = g_score[current] + move_cost
            
            if tentative_g < g_score.get(neighbor, float('inf')):
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score = tentative_g + heuristic_func(neighbor, goal)
                heapq.heappush(open_set, (f_score, tentative_g, neighbor))
    
    return None

# Common heuristics
def euclidean_heuristic(pos, goal):
    return ((pos[0] - goal[0])**2 + (pos[1] - goal[1])**2)**0.5

def manhattan_heuristic(pos, goal):
    return abs(pos[0] - goal[0]) + abs(pos[1] - goal[1])
```

### D* Lite

For replanning when the environment changes:

```
┌─────────────────────────────────────────────────────────────────┐
│                     D* Lite Algorithm                            │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  1. Initial plan from goal to start (reverse search)            │
│  2. Robot follows path                                          │
│  3. When obstacle detected:                                     │
│     - Update affected edges                                     │
│     - Recompute only affected nodes                             │
│     - Continue with updated path                                │
│                                                                  │
│  Advantage: Efficient replanning (don't restart from scratch)   │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

## Sampling-Based Algorithms

### Rapidly-exploring Random Trees (RRT)

Detailed implementation with visualization:

```python
import numpy as np
from typing import List, Optional, Tuple

class RRTNode:
    def __init__(self, position: np.ndarray, parent: Optional['RRTNode'] = None):
        self.position = position
        self.parent = parent

class RRT:
    def __init__(
        self,
        start: np.ndarray,
        goal: np.ndarray,
        bounds: Tuple[np.ndarray, np.ndarray],
        obstacle_checker,
        step_size: float = 0.5,
        goal_sample_rate: float = 0.05
    ):
        self.start = RRTNode(start)
        self.goal = goal
        self.bounds = bounds
        self.is_collision = obstacle_checker
        self.step_size = step_size
        self.goal_sample_rate = goal_sample_rate
        self.nodes: List[RRTNode] = [self.start]
    
    def plan(self, max_iter: int = 5000) -> Optional[List[np.ndarray]]:
        """
        Run RRT planning.
        """
        for _ in range(max_iter):
            # Sample random point
            q_rand = self._sample()
            
            # Find nearest node
            nearest = self._nearest(q_rand)
            
            # Steer toward sample
            q_new = self._steer(nearest.position, q_rand)
            
            # Check collision
            if not self.is_collision(nearest.position, q_new):
                new_node = RRTNode(q_new, nearest)
                self.nodes.append(new_node)
                
                # Check goal
                if np.linalg.norm(q_new - self.goal) < self.step_size:
                    return self._extract_path(new_node)
        
        return None
    
    def _sample(self) -> np.ndarray:
        """Sample random configuration with goal bias."""
        if np.random.random() < self.goal_sample_rate:
            return self.goal
        
        return np.random.uniform(self.bounds[0], self.bounds[1])
    
    def _nearest(self, point: np.ndarray) -> RRTNode:
        """Find nearest node to point."""
        distances = [np.linalg.norm(node.position - point) 
                     for node in self.nodes]
        return self.nodes[np.argmin(distances)]
    
    def _steer(self, from_pos: np.ndarray, to_pos: np.ndarray) -> np.ndarray:
        """Steer from one position toward another."""
        direction = to_pos - from_pos
        distance = np.linalg.norm(direction)
        
        if distance < self.step_size:
            return to_pos
        
        return from_pos + (direction / distance) * self.step_size
    
    def _extract_path(self, node: RRTNode) -> List[np.ndarray]:
        """Extract path from node to root."""
        path = []
        while node is not None:
            path.append(node.position)
            node = node.parent
        return path[::-1]
```

### RRT* (Asymptotically Optimal)

```python
class RRTStar(RRT):
    def __init__(self, *args, rewire_radius: float = 2.0, **kwargs):
        super().__init__(*args, **kwargs)
        self.rewire_radius = rewire_radius
        self.costs = {id(self.start): 0}
    
    def plan(self, max_iter: int = 5000) -> Optional[List[np.ndarray]]:
        """RRT* with rewiring."""
        for _ in range(max_iter):
            q_rand = self._sample()
            nearest = self._nearest(q_rand)
            q_new = self._steer(nearest.position, q_rand)
            
            if self.is_collision(nearest.position, q_new):
                continue
            
            # Find nearby nodes for rewiring
            near_nodes = self._near(q_new)
            
            # Choose best parent
            best_parent = nearest
            best_cost = self._cost(nearest) + self._dist(nearest.position, q_new)
            
            for node in near_nodes:
                cost = self._cost(node) + self._dist(node.position, q_new)
                if cost < best_cost and not self.is_collision(node.position, q_new):
                    best_parent = node
                    best_cost = cost
            
            # Add new node
            new_node = RRTNode(q_new, best_parent)
            self.nodes.append(new_node)
            self.costs[id(new_node)] = best_cost
            
            # Rewire nearby nodes
            for node in near_nodes:
                new_cost = best_cost + self._dist(q_new, node.position)
                if new_cost < self._cost(node):
                    if not self.is_collision(q_new, node.position):
                        node.parent = new_node
                        self.costs[id(node)] = new_cost
            
            # Check goal
            if np.linalg.norm(q_new - self.goal) < self.step_size:
                return self._extract_path(new_node)
        
        return None
    
    def _near(self, point: np.ndarray) -> List[RRTNode]:
        """Find all nodes within rewire radius."""
        return [
            node for node in self.nodes
            if np.linalg.norm(node.position - point) < self.rewire_radius
        ]
    
    def _cost(self, node: RRTNode) -> float:
        """Get cost to reach node."""
        return self.costs.get(id(node), float('inf'))
    
    def _dist(self, p1: np.ndarray, p2: np.ndarray) -> float:
        return np.linalg.norm(p1 - p2)
```

### Informed RRT*

Focuses sampling in an ellipsoidal region once a solution is found:

```
┌─────────────────────────────────────────────────────────────────┐
│                    Informed RRT* Sampling                        │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│         Start ●═══════════════════════════════● Goal            │
│               ╲                               ╱                 │
│                ╲         Ellipse            ╱                  │
│                 ╲       (focuses           ╱                   │
│                  ╲      sampling)         ╱                    │
│                   ╲                      ╱                     │
│                    ╲____________________╱                      │
│                                                                  │
│  After finding initial path, only sample within ellipse         │
│  defined by current best cost                                    │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

## Potential Field Methods

### Artificial Potential Fields

```python
class PotentialFieldPlanner:
    """
    Potential field navigation.
    """
    
    def __init__(
        self,
        attractive_gain: float = 1.0,
        repulsive_gain: float = 100.0,
        obstacle_distance: float = 2.0
    ):
        self.k_att = attractive_gain
        self.k_rep = repulsive_gain
        self.d_0 = obstacle_distance
    
    def attractive_force(
        self, position: np.ndarray, goal: np.ndarray
    ) -> np.ndarray:
        """
        Attractive force toward goal.
        F_att = -k_att * (position - goal)
        """
        return -self.k_att * (position - goal)
    
    def repulsive_force(
        self, position: np.ndarray, obstacles: List[np.ndarray]
    ) -> np.ndarray:
        """
        Repulsive force from obstacles.
        """
        force = np.zeros_like(position)
        
        for obs in obstacles:
            diff = position - obs
            dist = np.linalg.norm(diff)
            
            if dist < self.d_0:
                # Repulsive force magnitude
                magnitude = self.k_rep * (1/dist - 1/self.d_0) / (dist**2)
                # Direction away from obstacle
                direction = diff / dist
                force += magnitude * direction
        
        return force
    
    def plan(
        self,
        start: np.ndarray,
        goal: np.ndarray,
        obstacles: List[np.ndarray],
        step_size: float = 0.1,
        max_steps: int = 1000
    ) -> List[np.ndarray]:
        """
        Plan path using gradient descent on potential field.
        """
        path = [start.copy()]
        position = start.copy()
        
        for _ in range(max_steps):
            # Compute total force
            f_att = self.attractive_force(position, goal)
            f_rep = self.repulsive_force(position, obstacles)
            total_force = f_att + f_rep
            
            # Normalize and step
            if np.linalg.norm(total_force) > 0:
                direction = total_force / np.linalg.norm(total_force)
                position = position + step_size * direction
                path.append(position.copy())
            
            # Check goal reached
            if np.linalg.norm(position - goal) < step_size:
                path.append(goal)
                break
        
        return path
```

**Limitation**: Local minima can trap the robot.

## Algorithm Comparison

| Algorithm | Completeness | Optimality | Time | Space |
|-----------|--------------|------------|------|-------|
| **Dijkstra** | Complete | Optimal | $O((V+E)\log V)$ | $O(V)$ |
| **A*** | Complete | Optimal* | $O(b^d)$ | $O(b^d)$ |
| **RRT** | Probabilistically | No | - | $O(n)$ |
| **RRT*** | Probabilistically | Asymptotic | - | $O(n)$ |
| **PRM** | Probabilistically | Asymptotic | $O(n^2)$ | $O(n^2)$ |
| **Potential Field** | No | No | $O(n)$ | $O(1)$ |

*A* is optimal if heuristic is admissible

## Practical Considerations

### Choosing an Algorithm

```
┌─────────────────────────────────────────────────────────────────┐
│                  Algorithm Selection Guide                       │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  Low dimensional (2-3D), need optimal?                          │
│    → A* or Dijkstra                                             │
│                                                                  │
│  High dimensional (>6D), single query?                          │
│    → RRT or RRT*                                                │
│                                                                  │
│  High dimensional, multiple queries?                            │
│    → PRM                                                        │
│                                                                  │
│  Dynamic environment, need fast replanning?                     │
│    → D* Lite                                                    │
│                                                                  │
│  Need real-time reactive behavior?                              │
│    → Potential Fields + higher-level planner                    │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

### Path Smoothing

Raw paths often need smoothing:

```python
def smooth_path(
    path: List[np.ndarray],
    collision_checker,
    iterations: int = 100
) -> List[np.ndarray]:
    """
    Smooth path using shortcutting.
    """
    smoothed = path.copy()
    
    for _ in range(iterations):
        if len(smoothed) <= 2:
            break
        
        # Random two points
        i = np.random.randint(0, len(smoothed) - 2)
        j = np.random.randint(i + 2, len(smoothed))
        
        # Try direct connection
        if not collision_checker(smoothed[i], smoothed[j]):
            # Remove intermediate points
            smoothed = smoothed[:i+1] + smoothed[j:]
    
    return smoothed
```

## Summary

Path planning algorithms form the computational backbone of robot navigation:

- **Graph search** (A*, Dijkstra) for discrete, low-dimensional spaces
- **Sampling-based** (RRT, PRM) for high-dimensional configuration spaces
- **Optimization-based** methods for trajectory refinement
- **Potential fields** for reactive local navigation

The choice depends on dimensionality, optimality requirements, and computational constraints.
