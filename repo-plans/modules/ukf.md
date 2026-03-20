# Module: ukf

**Milestone:** M14 — Advanced State Estimation II  
**Status:** Not Started  
**Depends on:** common, M5 (`IStateEstimator` interface)

---

### Phase 1 — Interface Design

- [ ] `include/ukf/ukf.hpp` — `UKF : IStateEstimator`, templated on state dimension
- [ ] `include/ukf/ukf_config.hpp` — `UKFConfig` (α, β, κ sigma-point parameters, Q, R)

### Phase 2 — Failing Tests (Red)

- [ ] `tests/CMakeLists.txt`
- [ ] `tests/test_ukf.cpp`:
  - Linear system: UKF matches KF solution exactly (unscented transform is exact for linear)
  - Nonlinear pendulum: UKF converges faster than EKF from same initial condition
  - Sigma-point count: always `2n+1` points; weights sum to 1
  - Degenerate covariance: `P` stays positive semi-definite throughout

### Phase 3 — Implementation (Green)

- [ ] `src/ukf.cpp` — sigma-point propagation, unscented transform for predict + update
- [ ] Generic process function: `std::function<VectorXd(const VectorXd&, const VectorXd&)>`
- [ ] Generic measurement function: `std::function<VectorXd(const VectorXd&)>`

### Phase 4 — Passing Tests

- [ ] All tests pass

### Phase 4.5 — Observability

> **This phase gates module completion.** Both human developers and AI agents must be able
> to verify correct behavior through logs and metrics — not just visual inspection.

- [ ] `ILogger` injected into module constructor via `common::getLogger("ukf")` (mockable in tests)
- [ ] All state transitions logged at `DEBUG` level (init, reset, mode changes, error paths, sigma-point spread, residual norm)
- [ ] Hot-loop performance metrics logged at `TRACE` level (propagation time in µs)
- [ ] At least one test asserts expected log lines appear via stdout capture
- [ ] Zero `ERROR`-level log entries during all nominal test runs

```bash
# Confirm logging output during tests:
cmake --build build --target ukf_tests
cd build && ctest -R ukf --output-on-failure 2>&1 | grep "\[DEBUG\]\|\[TRACE\]"
```

### Phase 5 — Simulation Integration

- [ ] Registerable as alternative state estimator alongside EKF via ImGui module selector (estimator type: "ukf")

### Phase 6 — Visualization

- [ ] Render state covariance ellipse (same panel as EKF)

### Phase 7 — Docs Polish

- [ ] Update README.md
- [ ] Move this file to `repo-plans/modules/done/ukf.md`
