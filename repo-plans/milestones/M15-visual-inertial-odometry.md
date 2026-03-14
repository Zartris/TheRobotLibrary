# M15 — Visual-Inertial Odometry

**Status:** Not Started  
**Dependencies:** M6 (`visual_odometry`, `common/camera.hpp`), M14 (UKF preferred for VIO state propagation). M5 (`IStateEstimator`) as structural prerequisite.  
**Scope:** Complete "visual-inertial chapter." IMU processing (filter + pre-integrator) is the direct prerequisite of loosely-coupled VIO. Both modules are developed in sequence within M15.

---

## Goal

Provide the complete visual-inertial estimation stack. `imu_processing` is the entry point:
it filters raw IMU measurements and pre-integrates them between keyframes.
`visual_inertial_odometry` is the culmination: it fuses the IMU prior with VO pose updates
through a loosely-coupled EKF/UKF architecture, achieving lower drift than either source alone.

### Within-Milestone Dependency

`perception/imu_processing` → `state_estimation/visual_inertial_odometry`

---

## Modules

### perception/imu_processing

Complementary filter for orientation, bias estimation via exponential moving average, and
IMU pre-integration between keyframes. All types are self-contained in `ImuTypes`.

- [ ] `include/imu_processing/imu_types.hpp` — `ImuMeasurement` (accel, gyro, dt), `ImuBias` (accel_bias, gyro_bias), `ImuState` (orientation, velocity, position)
- [ ] `include/imu_processing/imu_filter.hpp` — `ImuFilter` (complementary filter, bias estimation)
- [ ] `include/imu_processing/imu_preintegrator.hpp` — `ImuPreintegrator` (accumulates ΔR, Δv, Δp between keyframes, Jacobians w.r.t. initial bias)
- [ ] `src/imu_filter.cpp` + `src/imu_preintegrator.cpp`
- [ ] `tests/test_imu_processing.cpp`:
  - Static IMU (zero motion): complementary filter → orientation converges; bias estimate converges within 50 steps
  - Known constant rotation: gyro integration matches analytical result
  - Pre-integration over N steps: `ΔR, Δv, Δp` match numerical integration of same sequence (< 1e-6 error)
  - Bias Jacobian: analytic matches finite-difference for `∂Δp/∂b_a`
- [ ] Phase 4.5: `ILogger`, bias estimates at `DEBUG`, integration step time at `TRACE`

### state_estimation/visual_inertial_odometry

Loosely-coupled VIO: EKF/UKF fusing IMU pre-integrated motion priors + VO pose updates.
Accepts `ImuPreintegrationResult` (from `imu_processing`) and `OdometryResult` (from
`visual_odometry`) as inputs — does not link against those modules directly; inputs are
provided by the caller through `common/` types.

- [ ] `include/visual_inertial_odometry/visual_inertial_odometry.hpp` — `VisualInertialOdometry : IStateEstimator`
- [ ] `include/visual_inertial_odometry/vio_state.hpp` — `VIOState` (SE3 pose, velocity, `ImuBias`)
- [ ] `include/visual_inertial_odometry/vio_config.hpp` — `VIOConfig` (noise params, update frequency)
- [ ] `src/visual_inertial_odometry.cpp` — predict via IMU pre-integration, update via VO relative pose
- [ ] Loosely-coupled architecture: VO updates at keyframe rate; IMU propagates between frames
- [ ] `tests/test_visual_inertial_odometry.cpp`:
  - IMU-only propagation (no VO updates): drift matches hand-calculated dead-reckoning bound
  - VO update corrects IMU drift: trajectory error decreases vs IMU-only baseline
  - Simulated IMU + camera sequence (hand-crafted): end-to-end drift < 1% of path length
  - Bias observability: after N VO updates, bias estimate converges toward true injected bias
- [ ] Phase 4.5: `ILogger`, predict/update cycle at `DEBUG`, cycle time + residual at `TRACE`
- [ ] Sim: SLAM pipeline slot — `ImuPreintegrator` + `VisualOdometry` + `VisualInertialOdometry`
- [ ] Frontend: VIO trajectory trace alongside raw VO trace; bias drift panel

---

## Deliverables

- [ ] `perception/imu_processing` module: interface, implementation, tests
- [ ] `state_estimation/visual_inertial_odometry` module: interface, implementation, tests
- [ ] VIO pipeline slot wired in simulation
- [ ] Frontend shows VIO trace vs VO trace + bias drift panel
- [ ] All modules pass Phase 4.5 — Observability gate

---

## Exit Criteria

1. Static IMU: orientation converges; bias estimate stabilizes
2. Pre-integration error < 1e-6 vs numerical integration reference
3. VIO trajectory drift < pure VO drift on same simulated sequence
4. `VIOState` (pose + velocity + bias) outputs valid, non-diverging data
5. All unit tests pass, CI green
6. All modules pass Phase 4.5 — Observability gate

---

## NOT IN

Tightly-coupled VIO (feature reprojection into estimator state), marginalization, IMU
initialization from still stand (→ future), online extrinsic calibration, SE3 pose graph
integration (uses loosely-coupled approach only).

> **Split note:** If M15 is too large for a single iteration, it may be split into M15a
> (`imu_processing` only) and M15b (`visual_inertial_odometry`) without changing any
> module or milestone content.
