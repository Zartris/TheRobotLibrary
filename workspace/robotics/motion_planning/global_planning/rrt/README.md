# rrt

RRT and RRT\* — sampling-based path planners for continuous configuration spaces.

More suitable than grid search when: the environment is continuous and non-grid,
the robot has complex kinematic constraints, or narrow passages make grid search expensive.

**Standalone module.** Depends only on `common`.

---

## What's in this module

- `RRT` — basic RRT: probabilistically complete, fast, non-optimal
- `RRTStar` — RRT\*: asymptotically optimal, same interface
- `GoalBiasedRRT` — biases sampling toward goal 5–10% of the time (faster convergence)
- `KinodynamicRRT` — extends toward samples using the robot's motion model (respects dynamics)

---

## Integration

```cmake
add_subdirectory(common)
add_subdirectory(rrt)
target_link_libraries(your_target PRIVATE rrt)
```

```cpp
#include <rrt/rrt_star.hpp>

rrt::Config cfg{
    .max_iterations   = 5000,
    .step_size        = 0.3,    // metres
    .goal_bias        = 0.1,    // sample goal 10% of the time
    .goal_tolerance   = 0.2,
};

rrt::RRTStar planner{cfg};
auto result = planner.plan(collision_checker, start_pose, goal_pose, bounds);
```

## Theory

See [`docs/theory.md`](docs/theory.md) and [`../docs/theory.md`](../docs/theory.md).
