# Module: pid

**Milestone:** M1 — Minimum Viable Robot (Sub-phase M1-F)  
**Status:** Not Started  
**Depends on:** common

---

### Phase 1 — Interface Design

- [ ] `include/pid/pid_controller.hpp` — `PIDConfig`, `PIDController`, `HeadingSpeedController : IController`

### Phase 2 — Failing Tests (Red)

- [ ] `tests/CMakeLists.txt`
- [ ] `tests/test_pid_controller.cpp`:
  - P-only: output proportional to error
  - PI: steady-state error → zero
  - PID: derivative damps overshoot
  - Anti-windup: integral doesn't explode under saturation
  - Reset clears state

### Phase 3 — Implementation (Green)

- [ ] `src/pid_controller.cpp`

### Phase 4 — Passing Tests

- [ ] All tests pass

### Phase 4.5 — Observability

> **This phase gates module completion.** Both human developers and AI agents must be able
> to verify correct behavior through logs and metrics — not just frontend visuals.

- [ ] `ILogger` injected into module constructor via `common::getLogger("pid")` (mockable in tests)
- [ ] All state transitions logged at `DEBUG` level (init, reset, mode changes, error paths)
- [ ] Hot-loop performance metrics logged at `TRACE` level (cycle time per iteration in µs, iteration count)
- [ ] At least one test asserts expected log lines appear via stdout capture
- [ ] Zero `ERROR`-level log entries during all nominal test runs

```bash
# Confirm logging output during tests:
cmake --build build --target pid_tests
cd build && ctest -R pid --output-on-failure 2>&1 | grep "\[DEBUG\]\|\[TRACE\]"
```

### Phase 5 — Simulation Integration

- [ ] Default controller in robot pipeline (M1-K)
- [ ] Selectable via `PUT /api/robot/controller {"type": "pid"}`

### Phase 6 — Frontend Visualization

- [ ] Controller output visible in robot state panel

### Phase 7 — Docs Polish

- [ ] Update README.md
- [ ] Move this file to `repo-plans/modules/done/pid.md`
