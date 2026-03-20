# Module: mppi

**Milestone:** M18 — Advanced Nonlinear Control  
**Status:** Not Started  
**Depends on:** common, M3 (stable `IController` interface), M13 (recommended)

---

### Phase 1 — Interface Design

- [ ] `include/mppi/mppi_controller.hpp` — `MPPIController : IController`
- [ ] `include/mppi/mppi_config.hpp` — `MPPIConfig` (N rollouts, horizon H, temperature λ, noise covariance Σ, dt)
- [ ] `include/mppi/mppi_cost.hpp` — `MPPICostFn = std::function<double(const Eigen::VectorXd&, const Eigen::VectorXd&)>`

### Phase 2 — Failing Tests (Red)

- [ ] `tests/CMakeLists.txt`
- [ ] `tests/test_mppi_controller.cpp`:
  - Double integrator: MPPI drives state to target; final error < tolerance
  - Obstacle avoidance: obstacle-colliding rollouts receive near-zero importance weight
  - Weight normalization: sum of `w_k` over N rollouts = 1 for any λ > 0
  - Temperature λ: high λ → uniform weights (exploration); low λ → peaked on best rollout (exploitation)
  - N = 1 (degenerate case): no crash; produces finite control output

### Phase 3 — Implementation (Green)

- [ ] `src/mppi_controller.cpp` — Monte Carlo rollout loop, importance weight normalization, control update: `u* = Σ (w_k · ε_k)` where `w_k = exp(−J_k / λ)`
- [ ] Configurable motion model via `std::function<VectorXd(VectorXd, VectorXd)>`

### Phase 4 — Passing Tests

- [ ] All tests pass

### Phase 4.5 — Observability

> **This phase gates module completion.** Both human developers and AI agents must be able
> to verify correct behavior through logs and metrics — not just visual inspection.

- [ ] `ILogger` injected into module constructor via `common::getLogger("mppi")` (mockable in tests)
- [ ] All state transitions logged at `DEBUG` level (init, reset, mode changes, error paths, rollout cost distribution min/max/mean)
- [ ] Hot-loop performance metrics logged at `TRACE` level (per-iteration solve time in µs)
- [ ] At least one test asserts expected log lines appear via stdout capture
- [ ] Zero `ERROR`-level log entries during all nominal test runs

```bash
# Confirm logging output during tests:
cmake --build build --target mppi_tests
cd build && ctest -R mppi --output-on-failure 2>&1 | grep "\[DEBUG\]\|\[TRACE\]"
```

### Phase 5 — Simulation Integration

- [ ] Selectable via ImGui module selector (controller type: "mppi")

### Phase 6 — Frontend Visualization

- [ ] Visualise rollout fan (N sampled trajectories, grey) + selected trajectory (highlighted)

### Phase 7 — Docs Polish

- [ ] Update README.md
- [ ] Move this file to `repo-plans/modules/done/mppi.md`
