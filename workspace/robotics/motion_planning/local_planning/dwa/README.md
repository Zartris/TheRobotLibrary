# dwa

Dynamic Window Approach (DWA) — real-time local planner for differential-drive robots.

**Standalone module.** Depends only on `common`.

---

## Integration

```cmake
add_subdirectory(common)
add_subdirectory(dwa)
target_link_libraries(your_target PRIVATE dwa)
```

```cpp
#include <dwa/dwa.hpp>

dwa::RobotConstraints constraints{
    .max_speed        = 1.0,    // m/s
    .min_speed        = -0.2,
    .max_omega        = 1.5,    // rad/s
    .max_accel        = 2.0,    // m/s²
    .max_angular_accel= 3.0,
};
dwa::Config cfg{
    .sim_time    = 2.0,   // seconds to simulate each arc
    .dt          = 0.1,
    .v_samples   = 20,
    .omega_samples = 40,
    .weights = {.heading = 0.5, .clearance = 0.3, .velocity = 0.2},
};

dwa::DWA planner{constraints, cfg};

// In the control loop:
common::Twist cmd = planner.computeVelocity(
    robot_pose, current_velocity, goal_pose, current_obstacles);
```

---

## Theory

See [`docs/theory.md`](docs/theory.md) and [`../docs/theory.md`](../docs/theory.md).
