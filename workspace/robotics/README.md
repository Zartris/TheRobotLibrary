# Robotics Modules

This directory contains the self-contained C++ robotics library modules.

Each subdirectory is a standalone library — it has its own `CMakeLists.txt`, public headers,
source files, unit tests, and theory documentation. Any module can be copied to a different
project and used independently without any dependency on the rest of this repository.

---

## Modules

Each domain contains sub-modules. Copy only the sub-module(s) you need — they are
self-contained with their own `CMakeLists.txt` and `docs/theory.md`.

| Domain              | Sub-module                              | Description                                        |
|---------------------|-----------------------------------------|----------------------------------------------------|
| `common/`           | *(flat — no sub-modules)*               | `Pose2D`, `Twist`, `Transform2D`, math utils       |
| `control/`          | `pid/`                                  | Discrete PID with anti-windup                      |
|                     | `pure_pursuit/`                         | Geometric path tracking; adaptive lookahead        |
|                     | `mpc/`                                  | Receding-horizon QP for diff-drive                 |
| `perception/`       | `lidar_processing/`                     | Scan filtering, segmentation, line extraction      |
|                     | `occupancy_grid/`                       | Binary Bayes grid, log-odds, inflation             |
|                     | `ray_casting/`                          | Bresenham / DDA ray traversal                      |
|                     | `obstacle_detection/`                   | Clustering, tracking, data association             |
| `state_estimation/` | `ekf/`                                  | Extended Kalman Filter, diff-drive model           |
|                     | `particle_filter/`                      | MCL / AMCL                                         |
|                     | `ekf_slam/`                             | Augmented-state EKF-SLAM                           |
|                     | `lidar_slam/`                           | Scan matching, pose-graph SLAM                     |
| `motion_planning/`  | `global_planning/astar/`                | A\* with octile heuristic                          |
|                     | `global_planning/dijkstra/`             | Dijkstra / distance maps                           |
|                     | `global_planning/rrt/`                  | RRT and RRT\* for continuous C-space               |
|                     | `local_planning/dwa/`                   | Dynamic Window Approach                            |
|                     | `trajectory_planning/velocity_profiling/` | Trapezoidal / S-curve / curvature-aware speed    |
|                     | `trajectory_planning/spline_fitting/`   | Cubic spline, Catmull-Rom, B-spline                |
|                     | `trajectory_planning/teb/`              | Timed Elastic Band (joint path + timing opt.)      |
|                     | `trajectory_planning/time_optimal/`     | TOPP-RA, minimum-snap QP                           |
|                     | `multi_robot/orca/`                     | VO→RVO→ORCA reactive avoidance                     |
|                     | `multi_robot/priority_planning/`        | Priority-based sequential planning                 |
|                     | `multi_robot/cbs/`                      | Conflict-Based Search (optimal MAPF)               |
|                     | `multi_robot/dmpc/`                     | Distributed MPC for fleet trajectory planning      |
|                     | `multi_robot/mader/`                    | MADER: decentralized async (MIT ACL)               |

---

## Standard Module Layout

Every module follows the same structure:

```
<module>/
├── CMakeLists.txt          # Standalone build config; produces a static library
├── README.md               # Module overview and integration guide
├── include/
│   └── <module>/           # Public headers (add this path to your include dirs)
├── src/                    # Implementation (.cpp files)
├── tests/                  # Unit tests (Catch2 or GoogleTest)
└── docs/
    └── theory.md           # Theory, mathematics, and algorithm explanations
```

---

## Dependency Rules

- All modules may depend on `common`.
- No module may depend on another domain module (e.g. `control` must not include from `perception`).
- No module may depend on `simulation` or `frontends`.
- Cross-domain shared types belong in `common`.

```
common  ←  control
common  ←  perception
common  ←  state_estimation
common  ←  motion_planning
```

---

## Adding a New Module

See [`../architecture.md`](../architecture.md) — "Adding a New Robotics Module" section.
