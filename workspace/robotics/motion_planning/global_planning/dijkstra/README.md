# dijkstra

Dijkstra's algorithm — uniform-cost optimal path planning on a grid.

Dijkstra is A\* with a zero heuristic. It explores all directions uniformly by cost,
making it useful for learning, for cases where no good heuristic exists, or for computing
the shortest path to *all* reachable cells (not just a single goal).

**Standalone module.** Depends only on `common`.

---

## Integration

```cmake
add_subdirectory(common)
add_subdirectory(dijkstra)
target_link_libraries(your_target PRIVATE dijkstra)
```

```cpp
#include <dijkstra/dijkstra.hpp>

dijkstra::Dijkstra planner{};
auto result = planner.plan(grid, start_cell, goal_cell);

// Or compute cost-to-go map from a single goal to all free cells:
auto cost_map = planner.computeCostMap(grid, goal_cell);
```

## Theory

See [`../docs/theory.md`](../docs/theory.md) — Dijkstra is the graph search template with `heuristic = 0`.
