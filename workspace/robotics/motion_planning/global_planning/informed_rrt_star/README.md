# informed_rrt_star

Informed RRT* global planner. Runs standard RRT* until the first feasible path is found, then restricts sampling to the prolate hyperspheroid (PHS) defined by the current best cost `c_best` and the start-goal distance `c_min`. This focuses exploration in the region that can improve the current solution, yielding dramatically faster convergence to near-optimal paths.

**Milestone:** M20 — Planning Upgrades III  
**Status:** Scaffold only — awaiting implementation

## Features

- `InformedRRTStar : IGlobalPlanner` — asymptotically optimal with heuristic-guided sampling
- Phase 1: standard RRT* until first feasible path found
- Phase 2: PHS sampling — `x_sample = C · diag(r) · x_ball + x_centre` with `r1 = c_best/2`, `r2..n = sqrt(c_best² − c_min²)/2`
- Configurable: max_iterations, step_size, rewire_radius, initial_rrt_budget
- Returns `std::nullopt` when no path exists

## Dependencies

- `common` (logging, types, `IGlobalPlanner`, `OccupancyGrid`)
- Eigen3 (matrix/vector math)
