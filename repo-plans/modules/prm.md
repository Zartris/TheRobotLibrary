# Module: prm

**Milestone:** M16 — Planning Upgrades II  
**Status:** Not Started  
**Depends on:** common, M7 (`IGlobalPlanner` interface), M4 (`OccupancyGrid`)

---

### Phase 1 — Interface Design

- [ ] `include/prm/prm_planner.hpp` — `PRMPlanner : IGlobalPlanner`
- [ ] `include/prm/prm_config.hpp` — `PRMConfig` (max_nodes, k_nearest, max_edge_length, num_samples)

### Phase 2 — Failing Tests (Red)

- [ ] `tests/CMakeLists.txt`
- [ ] `tests/test_prm_planner.cpp`:
  - Obstacle-free: after N samples, roadmap is connected; query finds short path
  - Narrow corridor: path found with success rate ≥ 90% over 100 runs (probabilistic test)
  - Multi-query: roadmap built once, two distinct queries answered correctly; roadmap not rebuilt
  - No path: all samples blocked → query returns `std::nullopt`; no crash
  - `k_nearest` parameter: larger k → denser roadmap → shorter query paths (statistical test)

### Phase 3 — Implementation (Green)

- [ ] `src/prm_planner.cpp` — random sampling in C-space, k-nearest graph construction, Dijkstra query

### Phase 4 — Passing Tests

- [ ] All tests pass

### Phase 4.5 — Observability

> **This phase gates module completion.** Both human developers and AI agents must be able
> to verify correct behavior through logs and metrics — not just visual inspection.

- [ ] `ILogger` injected into module constructor via `common::getLogger("prm")` (mockable in tests)
- [ ] All state transitions logged at `DEBUG` level (init, roadmap build stats: nodes, edges)
- [ ] Hot-loop performance metrics logged at `TRACE` level (query solve time in µs)
- [ ] At least one test asserts expected log lines appear via stdout capture
- [ ] Zero `ERROR`-level log entries during all nominal test runs

```bash
# Confirm logging output during tests:
cmake --build build --target prm_tests
cd build && ctest -R prm --output-on-failure 2>&1 | grep "\[DEBUG\]\|\[TRACE\]"
```

### Phase 5 — Simulation Integration

- [ ] Selectable via ImGui module selector (global planner type: "prm")

### Phase 6 — Visualization

- [ ] Render roadmap graph (grey edges) + planned path (highlighted)

### Phase 7 — Docs Polish

- [ ] Update README.md
- [ ] Move this file to `repo-plans/modules/done/prm.md`
