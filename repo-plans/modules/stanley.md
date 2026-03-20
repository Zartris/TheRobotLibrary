# Module: stanley

**Milestone:** M13 — Classical & Optimal Control  
**Status:** Not Started  
**Depends on:** common, M3 (stable `IController` interface)

---

### Phase 1 — Interface Design

- [ ] `include/stanley/stanley_controller.hpp` — `StanleyController : IController`
- [ ] `include/stanley/stanley_config.hpp` — `StanleyConfig` (gain k, velocity softening ε, max steering angle)

### Phase 2 — Failing Tests (Red)

- [ ] `tests/CMakeLists.txt`
- [ ] `tests/test_stanley_controller.cpp`:
  - Straight path: CTE decays exponentially from initial offset (verify gain k effect)
  - Curved path: tracks within tolerance on circular arc
  - Heading-only error (CTE = 0): pure heading correction, no CTE term
  - Zero velocity: softening ε prevents division by zero; output bounded
  - Reversing: heading error flips sign correctly

### Phase 3 — Implementation (Green)

- [ ] `src/stanley_controller.cpp` — `δ = ψ_e + arctan(k·e_cte / (v + ε))`

### Phase 4 — Passing Tests

- [ ] All tests pass

### Phase 4.5 — Observability

> **This phase gates module completion.** Both human developers and AI agents must be able
> to verify correct behavior through logs and metrics — not just visual inspection.

- [ ] `ILogger` injected into module constructor via `common::getLogger("stanley")` (mockable in tests)
- [ ] All state transitions logged at `DEBUG` level (init, reset, mode changes, error paths, CTE + heading error per step)
- [ ] Hot-loop performance metrics logged at `TRACE` level (per-step σ in µs)
- [ ] At least one test asserts expected log lines appear via stdout capture
- [ ] Zero `ERROR`-level log entries during all nominal test runs

```bash
# Confirm logging output during tests:
cmake --build build --target stanley_tests
cd build && ctest -R stanley --output-on-failure 2>&1 | grep "\[DEBUG\]\|\[TRACE\]"
```

### Phase 5 — Simulation Integration

- [ ] Selectable via ImGui module selector (controller type: "stanley")
- [ ] Note: Stanley outputs a steering angle δ for Ackermann kinematics. Sim integration may
  require a bicycle-model kinematic layer. If not available, defer Phase 5 to a post-M13
  sim upgrade; document the deferral in this file.

### Phase 6 — Visualization

- [ ] Cross-track error overlay (same style as `frenet` from M11)

### Phase 7 — Docs Polish

- [ ] Update README.md
- [ ] Move this file to `repo-plans/modules/done/stanley.md`
