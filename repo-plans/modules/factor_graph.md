# Module: factor_graph

**Milestone:** M21 — Estimation & Test Foundations  
**Status:** Not Started  
**Depends on:** common, M14 (`pose_graph` GN/LM optimizer as structural predecessor)

---

### Phase 1 — Interface Design

- [ ] `include/factor_graph/factor_graph.hpp` — `FactorGraph`; `addVariable<N>(id)` → `Variable<N>&`; `addFactor(factor)` → void; `optimize(maxIter, tol)` → `OptimizationResult`
- [ ] `include/factor_graph/variable.hpp` — `Variable<int N>` (id, `Eigen::VectorXd` value, fixed flag); `PoseVariable` (SE2), `LandmarkVariable` (x, y), `BiasVariable` (accel_bias, gyro_bias)
- [ ] `include/factor_graph/factor.hpp` — `IFactor` interface: `residual()`, `jacobian()`, `informationMatrix()`; `OdometryFactor`, `ObservationFactor`, `PriorFactor`

### Phase 2 — Failing Tests (Red)

- [ ] `tests/CMakeLists.txt`
- [ ] `tests/test_factor_graph.cpp`:
  - 2-node SE2 chain + `OdometryFactor`: optimizer converges; both poses match ground truth within 1e-6
  - 4-node SE2 loop (3 odometry + 1 loop closure): closed-loop poses converge; cumulative error < 0.05 m
  - Pose + landmark + `ObservationFactor`: landmark converges from corrupted initial guess within 10 iterations
  - GN convergence: `‖Δx‖` decreasing monotonically on noise-free problem
  - Jacobian check: analytic Jacobian matches finite-difference (< 1e-5 max element error)
  - Large graph (100 SE2 nodes, 10 loop closures): `optimize()` completes in < 1 s

### Phase 3 — Implementation (Green)

- [ ] `src/factor_graph.cpp` — GN iteration: assemble sparse Jacobian `J` + residual `r`; normal equations `J^T Ω J Δx = −J^T Ω r`; Eigen `SimplicialLDLT` solve; update variables; stop on `‖Δx‖ < tol`; gauge freedom handling

### Phase 4 — Passing Tests

- [ ] All tests pass

### Phase 4.5 — Observability

> **This phase gates module completion.** Both human developers and AI agents must be able
> to verify correct behavior through logs and metrics — not just visual inspection.

- [ ] `ILogger` injected into module constructor via `common::getLogger("factor_graph")` (mockable in tests)
- [ ] All state transitions logged at `DEBUG` level (init, reset, mode changes, error paths, iteration count + final residual norm)
- [ ] Hot-loop performance metrics logged at `TRACE` level (per-iteration solve time in µs)
- [ ] At least one test asserts expected log lines appear via stdout capture
- [ ] Zero `ERROR`-level log entries during all nominal test runs

```bash
# Confirm logging output during tests:
cmake --build build --target factor_graph_tests
cd build && ctest -R factor_graph --output-on-failure 2>&1 | grep "\[DEBUG\]\|\[TRACE\]"
```

### Phase 5 — Simulation Integration

- [ ] Integration note: `PoseGraph (M14)` can optionally be re-implemented as a thin wrapper over `FactorGraph` post-M21; not a requirement of M21 itself

### Phase 6 — Frontend Visualization

- [ ] No frontend visualization required for this infrastructure module

### Phase 7 — Docs Polish

- [ ] Update README.md
- [ ] Move this file to `repo-plans/modules/done/factor_graph.md`
