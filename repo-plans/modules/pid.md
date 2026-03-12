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

### Phase 5 — Simulation Integration

- [ ] Default controller in robot pipeline (M1-K)
- [ ] Selectable via `PUT /api/robot/controller {"type": "pid"}`

### Phase 6 — Frontend Visualization

- [ ] Controller output visible in robot state panel

### Phase 7 — Docs Polish

- [ ] Update README.md
- [ ] Move this file to `repo-plans/modules/done/pid.md`
