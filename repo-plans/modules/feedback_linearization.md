# Module: feedback_linearization

**Milestone:** M18 — Advanced Nonlinear Control  
**Status:** Not Started  
**Depends on:** common, M3 (stable `IController` interface), M13 (recommended)

---

### Phase 1 — Interface Design

- [ ] `include/feedback_linearization/feedback_linearization_controller.hpp` — `FeedbackLinearizationController : IController`
- [ ] `include/feedback_linearization/feedback_linearization_config.hpp` — `FeedbackLinearizationConfig` (lookahead `l`, inner controller gains, velocity limits)

### Phase 2 — Failing Tests (Red)

- [ ] `tests/CMakeLists.txt`
- [ ] `tests/test_feedback_linearization_controller.cpp`:
  - Straight-line tracking: virtual output tracks reference line; `(x, y, θ)` converges within tolerance
  - Circular arc: nonlinear system tracks circular reference; CTE bounded
  - Lookahead `l`: larger `l` → smoother but lagged response; smaller `l` → tighter but noisier
  - Lie derivative check: numerical `L_f h(x)` matches analytic expression (< 1e-5 relative error)
  - Singularity: `l = 0` → documented error via `std::expected`; no division by zero

### Phase 3 — Implementation (Green)

- [ ] `src/feedback_linearization_controller.cpp` — diffeomorphism: virtual output `ξ = (x + l·cos θ, y + l·sin θ)`; linearizing control law → `(v, ω)`; inverse kinematics back to robot frame
- [ ] `src/lie_derivative_check.cpp` — numerical check via finite-difference vs. analytic `L_f h(x)`

### Phase 4 — Passing Tests

- [ ] All tests pass

### Phase 4.5 — Observability

> **This phase gates module completion.** Both human developers and AI agents must be able
> to verify correct behavior through logs and metrics — not just visual inspection.

- [ ] `ILogger` injected into module constructor via `common::getLogger("feedback_linearization")` (mockable in tests)
- [ ] All state transitions logged at `DEBUG` level (init, reset, mode changes, error paths, virtual output error, Lie derivative residual)
- [ ] Hot-loop performance metrics logged at `TRACE` level (feedback linearization computation time in µs)
- [ ] At least one test asserts expected log lines appear via stdout capture
- [ ] Zero `ERROR`-level log entries during all nominal test runs

```bash
# Confirm logging output during tests:
cmake --build build --target feedback_linearization_tests
cd build && ctest -R feedback_linearization --output-on-failure 2>&1 | grep "\[DEBUG\]\|\[TRACE\]"
```

### Phase 5 — Simulation Integration

- [ ] Selectable via ImGui module selector (controller type: "feedback_linearization")

### Phase 6 — Frontend Visualization

- [ ] Render virtual output point trajectory overlay

### Phase 7 — Docs Polish

- [ ] Update README.md
- [ ] Move this file to `repo-plans/modules/done/feedback_linearization.md`
