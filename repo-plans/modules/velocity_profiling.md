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

### Phase 5 — Simulation Integration

- [ ] Called in pipeline between A* and DWA: A* produces a global `Path`, velocity profiler converts it to a `TimedPath`, DWA receives the `Path` portion and uses `TimedPath` velocity hints for its cost function (M1-K)
- [ ] Note: `ILocalPlanner::compute()` takes `Path` (not `TimedPath`). The pipeline extracts speed targets from `TimedPath` and passes them as DWA config parameters (e.g. `max_vel` for current segment).

### Phase 6 — Frontend Visualization

- [ ] Velocity profile visible in UI panel (speed vs distance graph)

### Phase 7 — Docs Polish

- [ ] Update README.md
- [ ] Move this file to `repo-plans/modules/done/velocity_profiling.md`
