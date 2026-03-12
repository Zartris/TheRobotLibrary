# global_planning

Geometric path planning through a static 2D map — finds a collision-free path from
start to goal without considering time, speed, or robot dynamics.

**Output:** a list of 2D poses / waypoints forming the path.

---

## Sub-modules

| Sub-module | Description |
|---|---|
| [`astar/`](astar/) | A\* — optimal grid-based path planner |
| [`dijkstra/`](dijkstra/) | Dijkstra — uniform-cost planner (A\* with zero heuristic) |
| [`rrt/`](rrt/) | RRT / RRT\* — sampling-based planners for cluttered or high-dimensional spaces |

---

## When to use each planner

- **Dijkstra** — learning purposes or when a zero heuristic makes analysis simpler
- **A\*** — standard choice for grid maps; optimal and fast in practice
- **RRT** — non-grid environments, narrow passages, or when the feasible space is complex
- **RRT\*** — when path quality matters and you can afford more samples

---

## See also

After finding a geometric path, use [`../trajectory_planning/`](../trajectory_planning/)
to add velocity, timing, and smoothness.

See [`docs/theory.md`](docs/theory.md) for the theoretical foundations.
