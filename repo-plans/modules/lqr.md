# Module: lqr

**Milestone:** M13 — Classical & Optimal Control  
**Status:** Not Started  
**Depends on:** common, M3 (stable `IController` interface)

---

### Phase 1 — Interface Design

- [ ] `include/lqr/lqr_controller.hpp` — `LQRController : IController`
- [ ] `include/lqr/lqr_config.hpp` — `LQRConfig` (Q, R, A, B matrices; discretization dt)

### Phase 2 — Failing Tests (Red)

- [ ] `tests/CMakeLists.txt`
- [ ] `tests/test_lqr_controller.cpp`:
  - Regulator drives state to zero from perturbation within N steps
  - Higher Q (R fixed) → faster convergence; higher R (Q fixed) → smaller control inputs
  - Provided A/B matrices for double integrator → analytic K matches theoretical solution
  - Stability: eigenvalues of `(A - BK)` lie strictly inside unit circle

### Phase 3 — Implementation (Green)

- [ ] `src/lqr_controller.cpp` — DARE solver + `u = -Kx` feedback
- [ ] Support linearised kinematic model at operating point (diff-drive linearization helper)

### Phase 4 — Passing Tests

- [ ] All tests pass

### Phase 4.5 — Observability

> **This phase gates module completion.** Both human developers and AI agents must be able
> to verify correct behavior through logs and metrics — not just frontend visuals.

- [ ] `ILogger` injected into module constructor via `common::getLogger("lqr")` (mockable in tests)
- [ ] All state transitions logged at `DEBUG` level (init, reset, mode changes, error paths)
- [ ] Hot-loop performance metrics logged at `TRACE` level (DARE solve time in µs, iteration count)
- [ ] At least one test asserts expected log lines appear via stdout capture
- [ ] Zero `ERROR`-level log entries during all nominal test runs

```bash
# Confirm logging output during tests:
cmake --build build --target lqr_tests
cd build && ctest -R lqr --output-on-failure 2>&1 | grep "\[DEBUG\]\|\[TRACE\]"
```

### Phase 5 — Simulation Integration

- [ ] Selectable via `PUT /api/robot/controller {"type":"lqr"}`

### Phase 6 — Frontend Visualization

- [ ] Render Q/R gain panel + current control effort

### Phase 7 — Docs Polish

- [ ] Update README.md
- [ ] Move this file to `repo-plans/modules/done/lqr.md`
