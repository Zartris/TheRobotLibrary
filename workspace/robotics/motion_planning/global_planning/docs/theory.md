# Theory: Global Planning

---

## 1. Graph Search Concepts

All grid-based global planners operate on a graph built from the occupancy grid:
- **Nodes:** free cells in the grid
- **Edges:** adjacency connections (4-connected or 8-connected)
- **Edge cost:** distance between cell centres (1.0 for cardinal, $\sqrt{2}$ for diagonal)

### Priority queue search (general template)

```
frontier ← priority queue containing {start, cost=0}
came_from ← {}
cost_so_far ← {start: 0}

while frontier not empty:
    current ← frontier.pop()
    if current == goal: break
    for each neighbour of current:
        new_cost = cost_so_far[current] + edge_cost(current, neighbour)
        if neighbour not in cost_so_far or new_cost < cost_so_far[neighbour]:
            cost_so_far[neighbour] = new_cost
            priority = new_cost + heuristic(neighbour, goal)   ← 0 for Dijkstra
            frontier.push(neighbour, priority)
            came_from[neighbour] = current

path ← reconstruct(came_from, start, goal)
```

**Dijkstra:** `heuristic = 0` always — explores uniformly by cost  
**A\*:** `heuristic = h(n, goal)` — biases exploration toward the goal

---

## 2. A\* and Heuristic Admissibility

A heuristic $h(n)$ is **admissible** if it never overestimates the true cost to goal:
$$h(n) \le h^*(n) \quad \forall n$$

When $h$ is admissible, A\* is **optimal** (finds the cheapest path). For 2D grids:
- **Manhattan distance** (4-connected, unit cost): $h = |x_g - x| + |y_g - y|$
- **Euclidean distance** (8-connected, resolution-scaled): $h = \text{res}\sqrt{(x_g-x)^2+(y_g-y)^2}$
- **Octile distance** (8-connected, exact): $h = \text{res}(\max(\Delta x, \Delta y) + (\sqrt{2}-1)\min(\Delta x, \Delta y))$

A\* with the octile heuristic on an 8-connected grid is both optimal and explores minimally.

---

## 3. Sampling-Based Planning: RRT

RRT grows a tree from the start into $\mathcal{C}_{\text{free}}$ by randomly sampling:

1. $q_{\text{rand}} \sim \text{Uniform}(\mathcal{C})$ (occasionally sample $q_{\text{goal}}$)
2. $q_{\text{near}} = \arg\min_{q \in T} d(q, q_{\text{rand}})$ (nearest node in tree)
3. $q_{\text{new}} = \text{steer}(q_{\text{near}}, q_{\text{rand}}, \epsilon)$ (step of size $\epsilon$)
4. If the segment $q_{\text{near}} \to q_{\text{new}}$ is collision-free: add to tree

RRT is **probabilistically complete** — as sample count $\to \infty$, it finds a path if one exists.

### RRT* (asymptotically optimal)

After inserting $q_{\text{new}}$, RRT* additionally:
- **Chooses the best parent:** from all nodes within radius $r$ of $q_{\text{new}}$, pick the
  one that gives the lowest cost path from start
- **Rewires:** for each near node, check if routing through $q_{\text{new}}$ reduces its cost;
  if so, update its parent

As $n \to \infty$, RRT* converges to the **optimal** path.

---

## 4. Path Post-Processing

Raw planner output (grid or tree) produces a jerky path. Smooth it before handing to
the trajectory planner:
- **Path pruning (ray-casting):** remove intermediate waypoints that can be bypassed with
  a straight collision-free segment
- **Spline interpolation:** fit a smooth curve through the waypoints (done in `trajectory_planning/spline_fitting/`)

---

## Further Reading

- LaValle — *Planning Algorithms*, Chapters 2 and 5
- Hart, Nilsson, Raphael — *A Formal Basis for the Heuristic Determination of Minimum Cost Paths (A*)*, 1968
- Karaman & Frazzoli — *Sampling-based Algorithms for Optimal Motion Planning*, IJRR 2011
