# astar

A\* path planner — optimal, complete grid-based path finding.

**Standalone module.** Depends only on `common`. Copy this folder to any C++ project.

---

## Integration

```cmake
add_subdirectory(common)
add_subdirectory(astar)
target_link_libraries(your_target PRIVATE astar)
```

```cpp
#include <astar/astar.hpp>

astar::Config cfg{.connectivity = astar::Connectivity::Eight};
astar::AStar planner{cfg};

// grid is a 2D bool array (or occupancy_grid::OccupancyGrid passed as IGrid)
auto result = planner.plan(grid, start_cell, goal_cell);
if (result.found) {
    auto path = result.path;   // std::vector<common::Point2D> in world coords
}
```

---

## Theory

See [`docs/theory.md`](docs/theory.md) — also see the
[sub-area theory](../docs/theory.md#2-a-and-heuristic-admissibility) for heuristic details.
