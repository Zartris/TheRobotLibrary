# ORCA — Optimal Reciprocal Collision Avoidance

Reactive, fully decentralized collision avoidance for a fleet of mobile robots.
No communication protocol, no pre-planned paths required between agents — each robot
makes independent decisions based on its local observation of neighbors.

**Learning path:** Velocity Obstacles (VO) → Reciprocal Velocity Obstacles (RVO) → ORCA

---

## When to Use

| Good fit | Poor fit |
|---|---|
| Large crowds / swarms | Narrow corridors (deadlocks possible) |
| Unknown number of agents | Hard optimality requirements |
| Robots without reliable comms | Non-holonomic with large turning radius |
| Real-time at high $N$ | Complex dynamic obstacles |

For non-holonomic robots (diff-drive), use **NH-ORCA** (adds kinematic constraints to LP).

---

## Interface

```cpp
#include "orca/orca_planner.hpp"

OrcaConfig cfg;
cfg.time_horizon   = 5.0;   // seconds to look ahead
cfg.robot_radius   = 0.25;  // metres
cfg.max_speed      = 1.5;   // m/s

OrcaPlanner planner(cfg);

// Called every control cycle for one robot:
Velocity2D v_safe = planner.computeVelocity(
    my_state,           // RobotState: position + velocity
    goal_velocity,      // preferred velocity (toward goal)
    neighbor_states     // std::vector<RobotState> of all observed neighbors
);
```

---

## File Layout

```
orca/
├── CMakeLists.txt
├── README.md
├── docs/
│   └── theory.md          ← VO → RVO → ORCA math, LP derivation
├── include/
│   └── orca/
│       ├── orca_planner.hpp
│       ├── velocity_obstacle.hpp    ← VO and RVO geometry
│       └── orca_line.hpp            ← ORCA half-plane representation
├── src/
│   ├── orca_planner.cpp
│   └── velocity_obstacle.cpp
└── tests/
    └── test_orca.cpp
```

---

## Dependencies

- `common/` — `Pose2D`, geometry utilities

---

## References

- Fiorini & Shiller — *Motion Planning in Dynamic Environments using Velocity Obstacles*, IJRR 1998
- van den Berg et al. — *Reciprocal Velocity Obstacles for Real-Time Multi-Agent Navigation*, ICRA 2008 (RVO)
- van den Berg et al. — *Reciprocal n-Body Collision Avoidance*, ISRR 2011 (ORCA)
- [RVO2 Library](http://gamma.cs.unc.edu/RVO2/) — reference implementation
