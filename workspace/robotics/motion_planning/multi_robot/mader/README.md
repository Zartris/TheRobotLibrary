# MADER — Multi-Agent Decentralized Trajectory Replanning

**State-of-the-art decentralized multi-robot trajectory planning from MIT ACL.**

Reference implementation: <https://github.com/mit-acl/mader>

Each agent independently generates fast, agile trajectories through free space,
sharing only its **committed polynomial trajectory** with neighbors. Collision-freedom
is guaranteed for committed segments — no synchronized replanning round needed.

---

## Key Ideas

| Concept | Description |
|---|---|
| **Polynomial trajectories** | Bézier or piecewise polynomial — smooth, fast to evaluate |
| **Convex decomposition** | Free space decomposed into convex polyhedra (no explicit obstacle shapes) |
| **Check-before-commit** | New trajectory is only committed after verifying it doesn't intersect committed regions of neighbors |
| **Asynchronous** | Robots replan at different times — no synchronized round |
| **MINVO polytopes** | Minimum-volume outer approximations of Bézier segments for fast intersection tests |

---

## When to Use

| Good fit | Poor fit |
|---|---|
| Agile robots (drones, fast ground robots) | Slow-moving robots where DMPC suffices |
| Dense 3D environments | 2D-only grid (CBS is simpler and optimal) |
| Asynchronous replanning needed | Reliable synchronous comms available |
| Tight spaces with convex free regions | Highly non-convex clutter (many small obstacles) |

---

## High-Level Algorithm (per agent, per replanning cycle)

```
1. SENSE  → perceive obstacles, receive neighbors' committed trajectories
2. DECOMPOSE → JPS (3D) + convex hull to build free-space polyhedra
3. PLAN   → optimize polynomial trajectory within polyhedra,
             minimizing time/snap, staying inside free polyhedra
4. CHECK  → test: does new trajectory intersect any neighbor's committed region?
5a. SAFE  → commit the new trajectory; broadcast it
5b. UNSAFE → keep old committed trajectory; wait for next cycle
```

The key invariant: once a trajectory segment is committed, it is never retracted.
This guarantees collision-freedom: if agent $i$ commits $\tau_i$ after checking against
neighbor $j$'s committed $\tau_j$, the two are disjoint.

---

## Interface

```cpp
#include "mader/mader_agent.hpp"

MaderConfig cfg;
cfg.dt_replan   = 0.05;    // replanning period (seconds)
cfg.poly_degree = 7;       // Bézier degree
cfg.horizon     = 2.0;     // trajectory horizon (seconds)
cfg.v_max       = 5.0;     // m/s (e.g. for a drone)
cfg.a_max       = 7.0;     // m/s²

MaderAgent agent(cfg, robot_id);

// Called every replanning cycle:
agent.updateSensorData(point_cloud, neighbor_committed_trajs);
agent.replan(goal_position);

// Called at control rate:
State ref = agent.getReference(current_time);
// Feed ref.position, ref.velocity, ref.acceleration to your controller
```

---

## File Layout

```
mader/
├── CMakeLists.txt
├── README.md
├── docs/
│   └── theory.md          ← convex decomposition, MINVO, check-commit loop
├── include/
│   └── mader/
│       ├── mader_agent.hpp
│       ├── mader_config.hpp
│       ├── poly_trajectory.hpp       ← piecewise Bézier polynomial
│       ├── convex_decomposition.hpp  ← JPS + convex hull
│       └── minvo_checker.hpp         ← MINVO intersection tests
├── src/
│   ├── mader_agent.cpp
│   ├── poly_trajectory.cpp
│   └── convex_decomposition.cpp
└── tests/
    └── test_mader.cpp
```

---

## Dependencies

- `common/`
- `trajectory_planning/time_optimal/` — polynomial trajectory optimization
- Eigen — matrix math
- CGAL or custom — convex hull computation

---

## References

- Tordesillas & How — *MADER: Trajectory Planner in Multiagent and Dynamic Environments*, T-RO 2022
- Tordesillas et al. — *FASTER: Fast and Safe Trajectory Planner for Navigation in Unknown Environments*, IROS 2019 (single-agent ancestor)
- [mit-acl/mader](https://github.com/mit-acl/mader) — reference open-source implementation (ROS-based)
- Tordesillas et al. — *MINVO Basis: Finding Simplexes with Minimum Volume Enclosing Polynomial Curves*, T-RO 2022
