# Module: pose_graph

**Milestone:** M14 — Advanced State Estimation II  
**Status:** Not Started  
**Depends on:** common

---

### Phase 1 — Interface Design

- [ ] `include/pose_graph/pose_graph.hpp` — `PoseGraph`, `PoseNode` (SE2), `PoseEdge` (relative transform + Ω = Σ⁻¹)
- [ ] `include/pose_graph/pose_graph_optimizer.hpp` — `PoseGraphOptimizer` (GN / LM)

### Phase 2 — Failing Tests (Red)

- [ ] `tests/CMakeLists.txt`
- [ ] `tests/test_pose_graph.cpp`:
  - 4-node square loop (known relative transforms + one loop closure edge) → optimize → node positions within tolerance of ground truth
  - Chain of 10 nodes with no loop closure → optimizer converges without drift (noise = 0)
  - Adding loop closure edge → trajectory correction > 20% improvement in cumulative error
  - Large graph (100 nodes, 5 loop closures) → solver finishes in < 500 ms (Eigen SparseLU)
  - Jacobian numerical check: analytic Jacobian matches finite-difference approximation

### Phase 3 — Implementation (Green)

- [ ] `src/pose_graph.cpp` — node/edge management, gauge freedom via anchor node 0
- [ ] `src/pose_graph_optimizer.cpp` — Gauss-Newton / LM linearised graph optimization
- [ ] APIs: `addNode()`, `addEdge()`, `optimize(maxIter)` → `OptimizationResult`

### Phase 4 — Passing Tests

- [ ] All tests pass

### Phase 4.5 — Observability

> **This phase gates module completion.** Both human developers and AI agents must be able
> to verify correct behavior through logs and metrics — not just visual inspection.

- [ ] `ILogger` injected into module constructor via `common::getLogger("pose_graph")` (mockable in tests)
- [ ] All state transitions logged at `DEBUG` level (init, reset, mode changes, error paths, iteration count, residual norm)
- [ ] Hot-loop performance metrics logged at `TRACE` level (solve time per iteration in µs)
- [ ] At least one test asserts expected log lines appear via stdout capture
- [ ] Zero `ERROR`-level log entries during all nominal test runs

```bash
# Confirm logging output during tests:
cmake --build build --target pose_graph_tests
cd build && ctest -R pose_graph --output-on-failure 2>&1 | grep "\[DEBUG\]\|\[TRACE\]"
```

### Phase 5 — Simulation Integration

- [ ] Available as optional loop closure backend for `lidar_slam` and `visual_slam` (M6.5)
- [ ] No direct REST endpoint — consumed as a library by SLAM modules

### Phase 6 — Frontend Visualization

- [ ] N/A for M14 — post-M14, simulation app can render the optimized pose graph

### Phase 7 — Docs Polish

- [ ] Update README.md
- [ ] Move this file to `repo-plans/modules/done/pose_graph.md`
