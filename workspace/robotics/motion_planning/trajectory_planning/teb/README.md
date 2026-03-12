# Timed Elastic Band (TEB)

Jointly optimizes **path shape and timing** to generate a smooth,
dynamically-feasible trajectory that avoids obstacles — without separating
spatial and temporal planning.

TEB is placed in `trajectory_planning/` because it produces a full $(\mathbf{x}(t), v(t))$
trajectory, not merely a geometric path. It can be used as:
- A trajectory smoother (refine a spline + velocity profile)
- A local planner that replans around dynamic obstacles at runtime

---

## Background

Originated from Rösmann et al. (2012, 2013, 2017). Widely deployed in ROS via
`teb_local_planner`. The C++ implementation here is a standalone version suitable
for autonomous ground vehicles and mobile robots.

---

## Interface

```cpp
#include "teb/teb_planner.hpp"

TebConfig cfg;
cfg.dt_ref       = 0.1;   // nominal time gap between poses
cfg.dt_hysteresis= 0.03;  // allowed deviation before pruning/inserting
cfg.max_vel_x    = 1.5;
cfg.max_accel_x  = 0.8;
cfg.min_obstacle_dist = 0.3;

TebPlanner planner(cfg);
planner.setObstacles(obstacles);    // std::vector<Obstacle>

TebTrajectory traj = planner.optimize(initial_plan, start_pose, goal_pose);
// initial_plan: spline path or previous trajectory
// Returns: traj.poses[], traj.time_diffs[] (dt between consecutive poses)
```

---

## File Layout

```
teb/
├── CMakeLists.txt
├── README.md
├── docs/
│   └── theory.md          ← elastic band concept, g2o formulation, hysteresis
├── include/
│   └── teb/
│       ├── teb_planner.hpp
│       ├── teb_config.hpp
│       ├── trajectory.hpp
│       └── obstacle.hpp
├── src/
│   ├── teb_planner.cpp
│   └── trajectory.cpp
└── tests/
    └── test_teb.cpp
```

---

## Dependencies

- `common/` — `Pose2D`, `Twist`
- `spline_fitting/` — optional (for initial plan generation)
- Eigen
- g2o (sparse graph optimizer) — or a custom Gauss-Newton implementation

---

## References

- Rösmann et al. — *Trajectory modification considering dynamic constraints of autonomous robots*, ROBOTIK 2012
- Rösmann et al. — *Efficient trajectory optimization using a sparse model*, ECC 2013
- Rösmann et al. — *Integrated online trajectory planning and optimization in distinctive topologies*, RAS 2017
