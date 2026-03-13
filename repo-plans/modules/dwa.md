# Module: dwa

**Milestone:** M1 — Minimum Viable Robot (Sub-phase M1-H)  
**Status:** Not Started  
**Depends on:** common

---

### Phase 1 — Interface Design

- [ ] `include/dwa/dwa_planner.hpp` — `DWAConfig`, `DWAPlanner : ILocalPlanner`

### Phase 2 — Failing Tests (Red)

- [ ] `tests/CMakeLists.txt`
- [ ] `tests/test_dwa_planner.cpp`:
  - Open space with goal ahead → forward velocity
  - Obstacle directly ahead → turning velocity
  - Boxed in → zero twist
  - Respects dynamic window (acceleration limits)

### Phase 3 — Implementation (Green)

- [ ] `src/dwa_planner.cpp` — velocity sampling → trajectory rollout → cost scoring → best selection

### Phase 4 — Passing Tests

- [ ] All tests pass

### Phase 4.5 — Observability

> **This phase gates module completion.** Both human developers and AI agents must be able
> to verify correct behavior through logs and metrics — not just frontend visuals.

- [ ] `ILogger` injected into module constructor via `common::getLogger("dwa")` (mockable in tests)
- [ ] All state transitions logged at `DEBUG` level (init, reset, mode changes, error paths)
- [ ] Hot-loop performance metrics logged at `TRACE` level (cycle time per iteration in µs, iteration count)
- [ ] At least one test asserts expected log lines appear via stdout capture
- [ ] Zero `ERROR`-level log entries during all nominal test runs

```bash
# Confirm logging output during tests:
cmake --build build --target dwa_tests
cd build && ctest -R dwa --output-on-failure 2>&1 | grep "\[DEBUG\]\|\[TRACE\]"
```

### Phase 5 — Simulation Integration

- [ ] Default local planner in robot pipeline (M1-K)
- [ ] Selectable via `PUT /api/robot/local_planner {"type": "dwa"}`

### Phase 6 — Frontend Visualization

- [ ] Top-N trajectory candidates rendered as thin blue arcs (M1-L)

### Phase 7 — Docs Polish

- [ ] Update README.md
- [ ] Move this file to `repo-plans/modules/done/dwa.md`
