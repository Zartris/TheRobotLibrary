# pure_pursuit

Pure pursuit path-tracking controller for differential-drive mobile robots.

**Standalone module.** Depends only on `common`. Copy this folder to any C++ project.

---

## What's in this module

- `PurePursuit` — computes `(v, ω)` velocity commands to track a 2D path
- Adaptive lookahead distance (scales with speed)
- Handles path segments, closed loops, and goal tolerance

---

## Integration

```cmake
add_subdirectory(common)
add_subdirectory(pure_pursuit)
target_link_libraries(your_target PRIVATE pure_pursuit)
```

```cpp
#include <pure_pursuit/pure_pursuit.hpp>

pure_pursuit::PurePursuit tracker{/* lookahead_distance */ 0.5};
tracker.setPath(path);  // std::vector<common::Point2D>

// In your control loop:
common::Twist cmd = tracker.computeVelocity(robot_pose, forward_speed);
```

---

## Theory

See [`docs/theory.md`](docs/theory.md) for the geometric derivation, lookahead distance
effects, and adaptive speed scheduling.
