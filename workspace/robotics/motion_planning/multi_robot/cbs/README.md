# CBS — Conflict-Based Search

An **optimal** multi-agent pathfinding algorithm. Uses a two-level search that is
significantly cheaper than planning in the joint configuration space, which is
$O(|S|^N)$ in the number of agents $N$.

CBS operates at the **path level** (discrete grid). To get time-parametrized
trajectories, pass the CBS output through `velocity_profiling/` or `teb/`.

---

## When to Use

| Good fit | Poor fit |
|---|---|
| Small-to-medium fleets ($N \le 30$) | Continuous (non-grid) environments |
| Warehouse grid maps | Very large fleets ($N > 50$) without ECBS |
| Optimal solution required | Unknown or changing environments |
| Offline batch planning | Real-time reactive avoidance |

For larger fleets with acceptable sub-optimality: use **ECBS** (Enhanced CBS) with a
focal search heuristic to get $w$-suboptimal solutions much faster.

---

## Interface

```cpp
#include "cbs/cbs_planner.hpp"

CbsConfig cfg;
cfg.suboptimality_factor = 1.0;  // 1.0 = optimal; >1.0 = ECBS mode, faster

CbsPlanner planner(cfg);
std::vector<GridPath> paths = planner.plan(
    grid_map,
    start_cells,    // std::vector<GridCell>
    goal_cells      // std::vector<GridCell>
);
// GridPath: timestamped sequence of grid cells for each agent
```

---

## File Layout

```
cbs/
├── CMakeLists.txt
├── README.md
├── docs/
│   └── theory.md          ← constraint tree, low-level A*, ECBS
├── include/
│   └── cbs/
│       ├── cbs_planner.hpp
│       ├── constraint_tree.hpp   ← CT node: constraints + solution + cost
│       ├── conflict.hpp          ← vertex / edge conflict types
│       └── low_level_planner.hpp ← space-time A*
├── src/
│   ├── cbs_planner.cpp
│   └── low_level_planner.cpp
└── tests/
    └── test_cbs.cpp
```

---

## Dependencies

- `common/`
- `global_planning/astar/` — used as the low-level single-agent planner

---

## References

- Sharon et al. — *Conflict-Based Search for Optimal Multi-Agent Pathfinding*, AAAI 2012
- Sharon et al. — *Conflict-Based Search for Optimal Multi-Agent Pathfinding*, AIJ 2015 (journal)
- Barer et al. — *Suboptimal Variants of the Conflict-Based Search Algorithm for the Multi-Agent Pathfinding Problem*, 2014 (ECBS)
