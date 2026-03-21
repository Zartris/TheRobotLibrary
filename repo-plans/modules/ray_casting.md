# Module: ray_casting

**Milestone:** M1 — Minimum Viable Robot (Sub-phase M1-B)  
**Status:** Not Started  
**Depends on:** common

---

### Phase 1 — Interface Design

- [ ] `include/ray_casting/ray_caster.hpp` — `castRay()`, `castScan()`, `RayCastResult`, `ScanConfig`

### Phase 2 — Failing Tests (Red)

- [ ] `tests/CMakeLists.txt` — Catch2 target
- [ ] `tests/test_ray_caster.cpp`:
  - Ray into empty grid → max_range, no hit
  - Ray toward wall at known distance → correct range ± 1 cell
  - Diagonal ray at 45° → expected hit
  - `castScan` with 3 rays → correct LaserScan structure

### Phase 3 — Implementation (Green)

- [ ] `src/ray_caster.cpp` — Bresenham ray traversal on OccupancyGrid

### Phase 4 — Passing Tests

- [ ] All tests pass

### Phase 4.5 — Observability

> **This phase gates module completion.** Both human developers and AI agents must be able
> to verify correct behavior through logs and metrics — not just visual inspection.

- [ ] `ILogger` injected into module constructor via `common::getLogger("ray_casting")` (mockable in tests)
- [ ] All state transitions logged at `DEBUG` level (init, reset, mode changes, error paths)
- [ ] Hot-loop performance metrics logged at `TRACE` level (cycle time per iteration in µs, iteration count)
- [ ] At least one test asserts expected log lines appear via stdout capture
- [ ] Zero `ERROR`-level log entries during all nominal test runs

```bash
# Confirm logging output during tests:
cmake --build build --target ray_casting_tests
cd build && ctest -R ray_casting --output-on-failure 2>&1 | grep "\[DEBUG\]\|\[TRACE\]"
```

### Phase 5 — Simulation Integration

- [ ] Used by simulation to generate simulated lidar data (M1-J)

### Phase 6 — Visualization

- [ ] Lidar rays rendered in simulation app (M1-L)

### Phase 7 — Docs Polish

- [ ] Update README.md with usage examples
- [ ] Move this file to `repo-plans/modules/done/ray_casting.md`
