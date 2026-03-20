# Module: occupancy_grid

**Milestone:** M1 — Minimum Viable Robot (Sub-phase M1-C)  
**Status:** Not Started  
**Depends on:** common, ray_casting (for cell marking during scan update)

---

### Phase 1 — Interface Design

- [ ] `include/occupancy_grid/occupancy_grid_map.hpp` — `OccupancyGridMap` class with log-odds update

### Phase 2 — Failing Tests (Red)

- [ ] `tests/CMakeLists.txt`
- [ ] `tests/test_occupancy_grid_map.cpp`:
  - Empty grid → all cells Unknown
  - Single ray update → cells along ray Free, endpoint Occupied
  - Multiple updates → log-odds accumulate correctly
  - Coordinate conversion round-trip: `toGrid(toWorld(c)) == c`

### Phase 3 — Implementation (Green)

- [ ] `src/occupancy_grid_map.cpp` — log-odds inverse sensor model, Bresenham cell marking

### Phase 4 — Passing Tests

- [ ] All tests pass

### Phase 4.5 — Observability

> **This phase gates module completion.** Both human developers and AI agents must be able
> to verify correct behavior through logs and metrics — not just visual inspection.

- [ ] `ILogger` injected into module constructor via `common::getLogger("occupancy_grid")` (mockable in tests)
- [ ] All state transitions logged at `DEBUG` level (init, reset, mode changes, error paths)
- [ ] Hot-loop performance metrics logged at `TRACE` level (cycle time per iteration in µs, iteration count)
- [ ] At least one test asserts expected log lines appear via stdout capture
- [ ] Zero `ERROR`-level log entries during all nominal test runs

```bash
# Confirm logging output during tests:
cmake --build build --target occupancy_grid_tests
cd build && ctest -R occupancy_grid --output-on-failure 2>&1 | grep "\[DEBUG\]\|\[TRACE\]"
```

### Phase 5 — Simulation Integration

- [ ] Grid updated from filtered lidar scans each sim tick (M1-K)

### Phase 6 — Frontend Visualization

- [ ] Grid rendered as colored cells in simulation app (M1-L)

### Phase 7 — Docs Polish

- [ ] Update README.md
- [ ] Move this file to `repo-plans/modules/done/occupancy_grid.md`
