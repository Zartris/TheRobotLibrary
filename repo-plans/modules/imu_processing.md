# Module: imu_processing

**Milestone:** M15 — Visual-Inertial Odometry  
**Status:** Not Started  
**Depends on:** common

---

### Phase 1 — Interface Design

- [ ] `include/imu_processing/imu_types.hpp` — `ImuMeasurement` (accel, gyro, dt), `ImuBias` (accel_bias, gyro_bias), `ImuState` (orientation, velocity, position)
- [ ] `include/imu_processing/imu_filter.hpp` — `ImuFilter` (complementary filter, bias estimation)
- [ ] `include/imu_processing/imu_preintegrator.hpp` — `ImuPreintegrator` (accumulates ΔR, Δv, Δp between keyframes, Jacobians w.r.t. initial bias)

### Phase 2 — Failing Tests (Red)

- [ ] `tests/CMakeLists.txt`
- [ ] `tests/test_imu_processing.cpp`:
  - Static IMU (zero motion): complementary filter → orientation converges; bias estimate converges within 50 steps
  - Known constant rotation: gyro integration matches analytical result
  - Pre-integration over N steps: `ΔR, Δv, Δp` match numerical integration of same sequence (< 1e-6 error)
  - Bias Jacobian: analytic matches finite-difference for `∂Δp/∂b_a`

### Phase 3 — Implementation (Green)

- [ ] `src/imu_filter.cpp` — complementary filter + exponential moving average bias estimation
- [ ] `src/imu_preintegrator.cpp` — accumulate ΔR, Δv, Δp; compute Jacobians w.r.t. bias

### Phase 4 — Passing Tests

- [ ] All tests pass

### Phase 4.5 — Observability

> **This phase gates module completion.** Both human developers and AI agents must be able
> to verify correct behavior through logs and metrics — not just frontend visuals.

- [ ] `ILogger` injected into module constructor via `common::getLogger("imu_processing")` (mockable in tests)
- [ ] All state transitions logged at `DEBUG` level (init, reset, mode changes, error paths, bias estimates)
- [ ] Hot-loop performance metrics logged at `TRACE` level (integration step time in µs)
- [ ] At least one test asserts expected log lines appear via stdout capture
- [ ] Zero `ERROR`-level log entries during all nominal test runs

```bash
# Confirm logging output during tests:
cmake --build build --target imu_processing_tests
cd build && ctest -R imu_processing --output-on-failure 2>&1 | grep "\[DEBUG\]\|\[TRACE\]"
```

### Phase 5 — Simulation Integration

- [ ] Part of the VIO pipeline slot: `ImuPreintegrator` → `VisualInertialOdometry`
- [ ] No standalone REST endpoint — consumed by `visual_inertial_odometry`

### Phase 6 — Frontend Visualization

- [ ] Displayed indirectly via VIO bias drift panel (M15 VIO module)

### Phase 7 — Docs Polish

- [ ] Update README.md
- [ ] Move this file to `repo-plans/modules/done/imu_processing.md`
