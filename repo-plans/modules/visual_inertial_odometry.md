# Module: visual_inertial_odometry

**Milestone:** M15 — Visual-Inertial Odometry  
**Status:** Not Started  
**Depends on:** common, M6 (`visual_odometry`, `common/camera.hpp`), M14 (UKF), `imu_processing` (within M15)

---

### Phase 1 — Interface Design

- [ ] `include/visual_inertial_odometry/visual_inertial_odometry.hpp` — `VisualInertialOdometry : IStateEstimator`
- [ ] `include/visual_inertial_odometry/vio_state.hpp` — `VIOState` (SE3 pose, velocity, `ImuBias`)
- [ ] `include/visual_inertial_odometry/vio_config.hpp` — `VIOConfig` (noise params, update frequency)

### Phase 2 — Failing Tests (Red)

- [ ] `tests/CMakeLists.txt`
- [ ] `tests/test_visual_inertial_odometry.cpp`:
  - IMU-only propagation (no VO updates): drift matches hand-calculated dead-reckoning bound
  - VO update corrects IMU drift: trajectory error decreases vs IMU-only baseline
  - Simulated IMU + camera sequence (hand-crafted): end-to-end drift < 1% of path length
  - Bias observability: after N VO updates, bias estimate converges toward true injected bias

### Phase 3 — Implementation (Green)

- [ ] `src/visual_inertial_odometry.cpp` — predict via IMU pre-integration; update via VO relative pose
- [ ] Loosely-coupled architecture: VO updates at keyframe rate; IMU propagates between frames
- [ ] Inputs via `common/` types only — does not link against `imu_processing` or `visual_odometry` directly

### Phase 4 — Passing Tests

- [ ] All tests pass

### Phase 4.5 — Observability

> **This phase gates module completion.** Both human developers and AI agents must be able
> to verify correct behavior through logs and metrics — not just visual inspection.

- [ ] `ILogger` injected into module constructor via `common::getLogger("visual_inertial_odometry")` (mockable in tests)
- [ ] All state transitions logged at `DEBUG` level (init, reset, predict/update cycle, error paths)
- [ ] Hot-loop performance metrics logged at `TRACE` level (cycle time in µs, residual norm)
- [ ] At least one test asserts expected log lines appear via stdout capture
- [ ] Zero `ERROR`-level log entries during all nominal test runs

```bash
# Confirm logging output during tests:
cmake --build build --target visual_inertial_odometry_tests
cd build && ctest -R visual_inertial_odometry --output-on-failure 2>&1 | grep "\[DEBUG\]\|\[TRACE\]"
```

### Phase 5 — Simulation Integration

- [ ] SLAM pipeline slot: `ImuPreintegrator` + `VisualOdometry` + `VisualInertialOdometry`

### Phase 6 — Visualization

- [ ] VIO trajectory trace alongside raw VO trace
- [ ] Bias drift panel

### Phase 7 — Docs Polish

- [ ] Update README.md
- [ ] Move this file to `repo-plans/modules/done/visual_inertial_odometry.md`
