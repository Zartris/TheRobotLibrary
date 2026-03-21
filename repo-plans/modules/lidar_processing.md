# Module: lidar_processing

**Milestone:** M1 — Minimum Viable Robot (Sub-phase M1-D)  
**Status:** Not Started  
**Depends on:** common

---

### Phase 1 — Interface Design

- [ ] `include/lidar_processing/scan_filter.hpp` — `FilterConfig`, `filterScan()`

### Phase 2 — Failing Tests (Red)

- [ ] `tests/CMakeLists.txt`
- [ ] `tests/test_scan_filter.cpp`:
  - Range clipping (below min, above max → NaN)
  - Median filter with window=3 → known output
  - Passthrough on clean scan → identical
  - All-NaN input → all-NaN output

### Phase 3 — Implementation (Green)

- [ ] `src/scan_filter.cpp`

### Phase 4 — Passing Tests

- [ ] All tests pass

### Phase 4.5 — Observability

> **This phase gates module completion.** Both human developers and AI agents must be able
> to verify correct behavior through logs and metrics — not just visual inspection.

- [ ] `ILogger` injected into module constructor via `common::getLogger("lidar_processing")` (mockable in tests)
- [ ] All state transitions logged at `DEBUG` level (init, reset, mode changes, error paths)
- [ ] Hot-loop performance metrics logged at `TRACE` level (cycle time per iteration in µs, iteration count)
- [ ] At least one test asserts expected log lines appear via stdout capture
- [ ] Zero `ERROR`-level log entries during all nominal test runs

```bash
# Confirm logging output during tests:
cmake --build build --target lidar_processing_tests
cd build && ctest -R lidar_processing --output-on-failure 2>&1 | grep "\[DEBUG\]\|\[TRACE\]"
```

### Phase 5 — Simulation Integration

- [ ] Filters raw simulated lidar before occupancy grid update (M1-K)

### Phase 6 — Visualization

_No direct visualization — filtering is invisible. Raw vs filtered scans could be toggled later._

### Phase 7 — Docs Polish

- [ ] Update README.md
- [ ] Move this file to `repo-plans/modules/done/lidar_processing.md`
