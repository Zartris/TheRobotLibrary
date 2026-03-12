# ray_casting

Synthetic range sensor simulation — cast rays through a known occupancy map to compute
what a real sensor would measure.

**Standalone module.** Depends only on `common`. Copy this folder to any C++ project.

---

## What's in this module

- `RayCaster` — casts a configurable lidar pattern through a grid map
- Bresenham grid traversal (fast integer arithmetic, no floating-point per step)
- DDA (Digital Differential Analyzer) alternative for sub-cell accuracy
- Configurable angular resolution, range limit, and noise injection

---

## Primary Use Cases

1. **Simulation** — generate synthetic sensor readings for the simulation backend
2. **Filter evaluation** — test state estimators against known ground truth
3. **Map validation** — compare expected vs. actual lidar for anomaly detection

---

## Integration

```cmake
add_subdirectory(common)
add_subdirectory(ray_casting)
target_link_libraries(your_target PRIVATE ray_casting)
```

```cpp
#include <ray_casting/ray_caster.hpp>

ray_casting::Config cfg{
    .num_beams     = 360,
    .angle_min     = -M_PI,
    .angle_max     =  M_PI,
    .max_range     = 8.0,
    .noise_std_dev = 0.02,   // metres, 0 = noiseless
};

ray_casting::RayCaster caster{cfg};
lidar_processing::Scan scan = caster.cast(grid, robot_pose);
```

---

## Theory

See [`docs/theory.md`](docs/theory.md) for ray traversal algorithms (Bresenham vs DDA)
and synthetic noise injection models.
