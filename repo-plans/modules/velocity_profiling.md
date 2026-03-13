# Module: velocity_profiling

**Milestone:** M1 — Minimum Viable Robot (Sub-phase M1-I)  
**Status:** Not Started  
**Depends on:** common

---

### Phase 1 — Interface Design

- [ ] `include/velocity_profiling/trapezoidal_profiler.hpp` — `TrapezoidalProfiler : IVelocityProfiler`

### Phase 2 — Failing Tests (Red)

- [ ] `tests/CMakeLists.txt`
- [ ] `tests/test_trapezoidal_profiler.cpp`:
  - Long straight path → accelerate-cruise-decelerate
  - Short path → triangular profile (no cruise phase)
  - Single-point path → zero velocity
  - Sharp turns → velocity reduced

### Phase 3 — Implementation (Green)

- [ ] `src/trapezoidal_profiler.cpp`

### Phase 4 — Passing Tests

- [ ] All tests pass

### Phase 4.5 — Observability

> **This phase gates module completion.** Both human developers and AI agents must be able
> to verify correct behavior through logs and metrics — not just frontend visuals.

- [ ] `ILogger` injected into module constructor via `common::getLogger("velocity_profiling")` (mockable in tests)
- [ ] All state transitions logged at `DEBUG` level (init, reset, mode changes, error paths)
- [ ] Hot-loop performance metrics logged at `TRACE` level (cycle time per iteration in µs, iteration count)
- [ ] At least one test asserts expected log lines appear via stdout capture
- [ ] Zero `ERROR`-level log entries during all nominal test runs

```bash
# Confirm logging output during tests:
cmake --build build --target velocity_profiling_tests
cd build && ctest -R velocity_profiling --output-on-failure 2>&1 | grep "\[DEBUG\]\|\[TRACE\]"
```

### Phase 5 — Simulation Integration

- [ ] Called in pipeline between A* and DWA: A* produces a global `Path`, velocity profiler converts it to a `TimedPath`, DWA receives the `Path` portion and uses `TimedPath` velocity hints for its cost function (M1-K)
- [ ] Note: `ILocalPlanner::compute()` takes `Path` (not `TimedPath`). The pipeline extracts speed targets from `TimedPath` and passes them as DWA config parameters (e.g. `max_vel` for current segment).

### Phase 6 — Frontend Visualization

- [ ] Velocity profile visible in UI panel (speed vs distance graph)

### Phase 7 — Docs Polish

- [ ] Update README.md
- [ ] Move this file to `repo-plans/modules/done/velocity_profiling.md`
