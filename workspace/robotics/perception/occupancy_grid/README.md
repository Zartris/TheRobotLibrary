# occupancy_grid

2D probabilistic occupancy grid map updated incrementally from range sensor readings.

**Standalone module.** Depends only on `common`. Copy this folder to any C++ project.

---

## What's in this module

- `OccupancyGrid` — fixed-size 2D grid with log-odds update and probability query
- `OccupancyGridUpdater` — applies a lidar scan to a grid given the robot pose (uses ray tracing)
- Serialization helpers (save/load grid to/from binary)
- Inflation layer — expand occupied cells by robot radius for path planning

---

## Integration

```cmake
add_subdirectory(common)
add_subdirectory(occupancy_grid)
target_link_libraries(your_target PRIVATE occupancy_grid)
```

```cpp
#include <occupancy_grid/occupancy_grid.hpp>
#include <occupancy_grid/grid_updater.hpp>

// 200×200 cells, 5 cm resolution, world origin at (-5, -5)
occupancy_grid::OccupancyGrid grid{200, 200, 0.05, {-5.0, -5.0}};
occupancy_grid::GridUpdater updater{};

updater.update(grid, robot_pose, lidar_scan);

double prob = grid.probability(1.0, 2.5);   // world coords → cell probability
```

---

## Theory

See [`docs/theory.md`](docs/theory.md) for the log-odds update derivation and the
inverse sensor model.
