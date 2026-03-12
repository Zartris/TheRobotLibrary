# Priority-Based Planning

The simplest **coordinated** multi-robot planner. Assign a priority ordering to robots;
each robot plans its trajectory treating all higher-priority robots' planned trajectories
as moving obstacles. Very fast, easy to implement, and a useful baseline.

---

## When to Use

| Good fit | Poor fit |
|---|---|
| Small to medium fleets (2–30) | All robots have equal priority (need fairness) |
| Sparse environments | Dense environments (frequent restarts) |
| Offline / pre-mission planning | Online with frequent priority changes |
| Learning / debugging multi-robot | Time-critical optimal solutions |

---

## Interface

```cpp
#include "priority_planning/priority_planner.hpp"

PriorityConfig cfg;
cfg.priority_mode = PriorityMode::FIXED_ORDER;  // or RANDOM, DISTANCE_TO_GOAL

PriorityPlanner planner(cfg, single_robot_planner);
// single_robot_planner: any planner satisfying SingleRobotPlanner interface

std::vector<Trajectory> trajectories = planner.plan(
    start_states,   // std::vector<RobotState>
    goal_poses,     // std::vector<Pose2D>
    occupancy_map
);
```

---

## File Layout

```
priority_planning/
├── CMakeLists.txt
├── README.md
├── docs/
│   └── theory.md          ← priority ordering, space-time obstacles, completeness analysis
├── include/
│   └── priority_planning/
│       ├── priority_planner.hpp
│       └── space_time_obstacle.hpp
├── src/
│   └── priority_planner.cpp
└── tests/
    └── test_priority_planning.cpp
```

---

## Dependencies

- `common/`
- `global_planning/astar/` or `global_planning/rrt/` — as the underlying single-robot planner
- `trajectory_planning/velocity_profiling/` — to convert paths to time-stamped trajectories
