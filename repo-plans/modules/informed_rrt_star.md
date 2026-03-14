# Module: informed_rrt_star

**Milestone:** M20 — Planning Upgrades III  
**Status:** Not Started  
**Depends on:** common, M7 (`IGlobalPlanner` interface, `RRTStar` implementation), M4 (`OccupancyGrid`)

---

### Phase 1 — Interface Design

- [ ] `include/informed_rrt_star/informed_rrt_star_planner.hpp` — `InformedRRTStar : IGlobalPlanner`
- [ ] `include/informed_rrt_star/informed_rrt_star_config.hpp` — `InformedRRTStarConfig` (max_iterations, step_size, rewire_radius, initial_rrt_budget)

### Phase 2 — Failing Tests (Red)

- [ ] `tests/CMakeLists.txt`
- [ ] `tests/test_informed_rrt_star.cpp`:
  - Obstacle-free: finds path; solution length ≤ 1.05 × `c_min` after sufficient iterations
  - Convergence speed: reaches `c_best ≤ 1.1 × c_min` in ≤ 50% of iterations vanilla RRT* requires (over 10 seeds)
  - PHS bound check: all post-first-solution samples lie within ellipsoid (`x^T * M^-1 * x ≤ 1`)
  - C-space obstacle: path correctly avoids obstacle; cost strictly non-increasing
  - No feasible path: returns `std::nullopt`; no crash

### Phase 3 — Implementation (Green)

- [ ] `src/informed_rrt_star_planner.cpp` — standard RRT* phase until first feasible path; PHS transformation: `x_sample = C · diag(r1,...,rn) · x_ball + x_centre`, where `r1 = c_best/2`, `r2 = ... = rn = sqrt(c_best² − c_min²)/2`; switch to informed sampling once `c_best < ∞`

### Phase 4 — Passing Tests

- [ ] All tests pass

### Phase 4.5 — Observability

> **This phase gates module completion.** Both human developers and AI agents must be able
> to verify correct behavior through logs and metrics — not just frontend visuals.

- [ ] `ILogger` injected into module constructor via `common::getLogger("informed_rrt_star")` (mockable in tests)
- [ ] All state transitions logged at `DEBUG` level (init, reset, mode changes, error paths, `c_best` improvement per iteration, PHS volume reduction per iteration)
- [ ] Hot-loop performance metrics logged at `TRACE` level (PHS sampling computation time in µs)
- [ ] At least one test asserts expected log lines appear via stdout capture
- [ ] Zero `ERROR`-level log entries during all nominal test runs

```bash
# Confirm logging output during tests:
cmake --build build --target informed_rrt_star_tests
cd build && ctest -R informed_rrt_star --output-on-failure 2>&1 | grep "\[DEBUG\]\|\[TRACE\]"
```

### Phase 5 — Simulation Integration

- [ ] Selectable via `PUT /api/robot/global_planner {"type":"informed_rrt_star"}`

### Phase 6 — Frontend Visualization

- [ ] Render PHS ellipse overlay; show `c_best` convergence curve in side panel

### Phase 7 — Docs Polish

- [ ] Update README.md
- [ ] Move this file to `repo-plans/modules/done/informed_rrt_star.md`
