# Module: lqg

**Milestone:** M22 — Optimal Output Feedback & Automotive Perception  
**Status:** Not Started  
**Depends on:** common, M13 (`LQRController`, `IController` interface), M1 (`EKF` state observer)

---

### Phase 1 — Interface Design

- [ ] `include/lqg/lqg_controller.hpp` — `LQGController : IController`
- [ ] `include/lqg/lqg_config.hpp` — `LQGConfig` (system: `A, B, C`; LQR weights: `Q, R`; noise: process `Q_w`, measurement `R_v`; discretization `dt`)

### Phase 2 — Failing Tests (Red)

- [ ] `tests/CMakeLists.txt`
- [ ] `tests/test_lqg_controller.cpp`:
  - Double integrator, output = position only (C = [1 0]): LQG drives position and velocity to zero; velocity estimate converges even though not measured
  - Separation principle: LQR gain `K` matches `LQRController::computeGain()` independently; Kalman gain `L` matches standalone EKF construction
  - AWGN measurement noise: steady-state estimation covariance matches theoretical DARE solution
  - LQR + perfect state observer (C = I): LQG reduces to pure LQR (verify numerically)
  - Wrong noise covariance (R_v underestimated by 10×): controller still stabilizes; documents behaviour

### Phase 3 — Implementation (Green)

- [ ] `src/lqg_controller.cpp` — initialize `LQRController` with `(A, B, Q, R)` → gain `K`; initialize internal `EKF` with `(A, B, C, Q_w, R_v)` → Kalman gain; `update(y)` → EKF predict + update; `computeControl(x̂)` → `LQR u = −K x̂`

### Phase 4 — Passing Tests

- [ ] All tests pass

### Phase 4.5 — Observability

> **This phase gates module completion.** Both human developers and AI agents must be able
> to verify correct behavior through logs and metrics — not just visual inspection.

- [ ] `ILogger` injected into module constructor via `common::getLogger("lqg")` (mockable in tests)
- [ ] All state transitions logged at `DEBUG` level (init, reset, mode changes, error paths, state estimate covariance trace + control effort, EKF residual norm)
- [ ] Hot-loop performance metrics logged at `TRACE` level (EKF predict+update computation time in µs)
- [ ] At least one test asserts expected log lines appear via stdout capture
- [ ] Zero `ERROR`-level log entries during all nominal test runs

```bash
# Confirm logging output during tests:
cmake --build build --target lqg_tests
cd build && ctest -R lqg --output-on-failure 2>&1 | grep "\[DEBUG\]\|\[TRACE\]"
```

### Phase 5 — Simulation Integration

- [ ] Selectable via ImGui module selector (controller type: "lqg")

### Phase 6 — Frontend Visualization

- [ ] Render estimated state covariance ellipse; control effort vs. time; estimation residual panel

### Phase 7 — Docs Polish

- [ ] Update README.md
- [ ] Move this file to `repo-plans/modules/done/lqg.md`
