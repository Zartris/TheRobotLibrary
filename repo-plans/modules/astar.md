# Module: astar

**Milestone:** M1 — Minimum Viable Robot (Sub-phase M1-G)  
**Status:** Not Started  
**Depends on:** common

---

### Phase 1 — Interface Design

- [ ] `include/astar/astar_planner.hpp` — `AStarPlanner : IGlobalPlanner`

### Phase 2 — Failing Tests (Red)

- [ ] `tests/CMakeLists.txt`
- [ ] `tests/test_astar_planner.cpp`:
  - Straight line in empty grid → direct path
  - L-shaped obstacle → path goes around
  - No path possible (fully enclosed) → `std::nullopt`
  - Start == goal → single-point path
  - Start/goal in occupied cell → `std::nullopt`

### Phase 3 — Implementation (Green)

- [ ] `src/astar_planner.cpp` — priority queue, octile heuristic, 8-connected neighbors

### Phase 4 — Passing Tests

- [ ] All tests pass

### Phase 4.5 — Observability

> **This phase gates module completion.** Both human developers and AI agents must be able
> to verify correct behavior through logs and metrics — not just visual inspection.

- [ ] `ILogger` injected into module constructor via `common::getLogger("astar")` (mockable in tests)
- [ ] All state transitions logged at `DEBUG` level (init, reset, mode changes, error paths)
- [ ] Hot-loop performance metrics logged at `TRACE` level (cycle time per iteration in µs, iteration count)
- [ ] At least one test asserts expected log lines appear via stdout capture
- [ ] Zero `ERROR`-level log entries during all nominal test runs

```bash
# Confirm logging output during tests:
cmake --build build --target astar_tests
cd build && ctest -R astar --output-on-failure 2>&1 | grep "\[DEBUG\]\|\[TRACE\]"
```

### Phase 5 — Simulation Integration

- [ ] Default global planner in robot pipeline (M1-K)
- [ ] Selectable via ImGui module selector (global planner type: "astar")

### Phase 6 — Frontend Visualization

- [ ] Planned global path rendered as green line (M1-L)

### Phase 7 — Docs Polish

- [ ] Update README.md
- [ ] Move this file to `repo-plans/modules/done/astar.md`
