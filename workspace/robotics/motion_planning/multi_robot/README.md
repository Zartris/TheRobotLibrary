# Multi-Robot Planning

Compute **collision-free trajectories for a fleet of robots** simultaneously.
This sub-area sits inside `motion_planning/` because it extends single-robot planning
to the multi-agent case.

---

## Learning Path

Study the modules in this order — each builds on the previous:

| # | Module | Category | Key concept |
|---|---|---|---|
| 1 | [`orca/`](orca/) | Reactive | Velocity obstacles; decentralized, no communication |
| 2 | [`priority_planning/`](priority_planning/) | Coordinated | Sequential: each robot treats prior robots as obstacles |
| 3 | [`cbs/`](cbs/) | Coordinated | Optimal discrete path-level conflict resolution |
| 4 | [`dmpc/`](dmpc/) | Coordinated | Distributed MPC; trajectory-level with dynamics |
| 5 | [`mader/`](mader/) | Coordinated | Decentralized async replanning with convex decomposition |

---

## Reactive vs. Coordinated

| | Reactive | Coordinated |
|---|---|---|
| Communication | None (or minimal) | Required |
| Optimality | Local (greedy) | Near-global (planned) |
| Scalability | Excellent ($O(N^2)$ per agent) | Limited (CBS: exponential w.o. heuristics) |
| Dynamic changes | Immediate | Requires replanning |
| Typical use | Large pedestrian/drone crowds | Warehouse fleets, planned missions |

---

## Sub-Area Structure

```
multi_robot/
├── README.md            ← this file
├── docs/
│   └── theory.md        ← velocity obstacle framework, comparison table
├── orca/                ← VO → RVO → ORCA learning progression
├── priority_planning/   ← simplest coordinated baseline
├── cbs/                 ← optimal discrete (grid-based) coordination
├── dmpc/                ← distributed MPC, trajectory-level
└── mader/               ← decentralized asynchronous, convex decomposition
```

---

## Fleet Interface (Shared Contract)

All multi-robot planners in this sub-area satisfy:

```cpp
// Each planner implements:
class MultiRobotPlanner {
public:
    virtual std::vector<Trajectory> plan(
        const std::vector<RobotState>& start_states,
        const std::vector<Pose2D>&     goal_poses,
        const OccupancyGrid&           static_map
    ) = 0;
};
```

---

## References

- Overview paper: Salzman — *Motion Planning for Multiple Agents: A Survey*, 2023
- [mit-acl/mader](https://github.com/mit-acl/mader) — MADER reference implementation
