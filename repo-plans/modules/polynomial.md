# Module: polynomial

**Milestone:** M16 — Planning Upgrades II  
**Status:** Not Started  
**Depends on:** common

---

### Phase 1 — Interface Design

- [ ] `include/polynomial/polynomial_trajectory.hpp` — `PolynomialTrajectory`, `PolynomialSegment`, `BezierCurve`
- [ ] `include/polynomial/polynomial_config.hpp` — `PolynomialConfig` (order, derivative order to minimize, segment durations)

### Phase 2 — Failing Tests (Red)

- [ ] `tests/CMakeLists.txt`
- [ ] `tests/test_polynomial_trajectory.cpp`:
  - 3-waypoint minimum-jerk: endpoint positions match; endpoint derivatives (vel, accel) match specified boundary conditions
  - Minimum-snap: 4th derivative minimized (verify via numerical integration of snap vs. spline baseline)
  - Bézier curve: all points lie strictly within convex hull of control points
  - `sample(t)` continuity: position/velocity continuous at segment junctions; acceleration C0 for min-jerk
  - Single segment: analytic polynomial coefficients match closed-form solution

### Phase 3 — Implementation (Green)

- [ ] `src/polynomial_trajectory.cpp` — coefficient QP (Eigen `LDLT`), `BezierCurve` via de Casteljau
- [ ] API: `generate(waypoints, durations)` → `PolynomialTrajectory`; `sample(t)` → position/velocity/acceleration

### Phase 4 — Passing Tests

- [ ] All tests pass

### Phase 4.5 — Observability

> **This phase gates module completion.** Both human developers and AI agents must be able
> to verify correct behavior through logs and metrics — not just frontend visuals.

- [ ] `ILogger` injected into module constructor via `common::getLogger("polynomial")` (mockable in tests)
- [ ] All state transitions logged at `DEBUG` level (init, reset, mode changes, error paths, QP solve time, condition number)
- [ ] Hot-loop performance metrics logged at `TRACE` level (sample computation time in µs)
- [ ] At least one test asserts expected log lines appear via stdout capture
- [ ] Zero `ERROR`-level log entries during all nominal test runs

```bash
# Confirm logging output during tests:
cmake --build build --target polynomial_tests
cd build && ctest -R polynomial --output-on-failure 2>&1 | grep "\[DEBUG\]\|\[TRACE\]"
```

### Phase 5 — Simulation Integration

- [ ] Trajectory planner slot — replaces `velocity_profiling` for drone/arm scenarios

### Phase 6 — Frontend Visualization

- [ ] Render polynomial trajectory with curvature color coding

### Phase 7 — Docs Polish

- [ ] Update README.md
- [ ] Move this file to `repo-plans/modules/done/polynomial.md`
