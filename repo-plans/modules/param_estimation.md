# Module: `param_estimation`

**Milestone:** M3.5 — Vehicle Dynamics
**Status:** Not Started
**Depends on:** common (including common/robot/ for VehicleParams, common/kinematics/ for IDynamicModel)

---

### Phase 1 — Interface Design

- [ ] `include/param_estimation/step_response_fitter.hpp` — `StepResponseFitter`
  - Straight-line acceleration/deceleration test → estimates mass and drag coefficient via least-squares
- [ ] `include/param_estimation/circle_test_fitter.hpp` — `CircleTestFitter`
  - Steady-state circle at known speed → estimates cornering stiffness (Cf, Cr) and yaw inertia (Iz) from steady-state yaw rate equation
- [ ] `include/param_estimation/cog_estimator.hpp` — `CogEstimator`
  - Static weight measurement (front/rear axle loads) → CoG position (lf, lr)
- [ ] `include/param_estimation/fit_types.hpp` — `FittedParams`, `FitError`
  - All fitters return `std::expected<FittedParams, FitError>` with residual and confidence
- [ ] Document inputs, outputs, and error conditions in header comments

### Phase 2 — Failing Tests (Red)

- [ ] Create `tests/CMakeLists.txt` with Catch2 target (`param_estimation_tests`)
- [ ] `StepResponseFitter`: synthetic data with known ground-truth mass/drag → fitted values within 5% tolerance
- [ ] `StepResponseFitter`: noisy data (SNR 20 dB) → fitted values within 15% tolerance with wider confidence interval
- [ ] `CircleTestFitter`: synthetic steady-state circle data → Cf, Cr, Iz within 5% of ground truth
- [ ] `CircleTestFitter`: noisy data (SNR 20 dB) → within 15% tolerance
- [ ] `CogEstimator`: known axle loads → lf, lr within 5% of ground truth
- [ ] All fitters: insufficient data → returns `FitError` (not a crash)
- [ ] Verify tests fail (module not yet implemented)

```bash
cmake --build build --target param_estimation_tests
cd build && ctest -R param_estimation --output-on-failure
```

### Phase 3 — Implementation (Green)

- [ ] `src/step_response_fitter.cpp` — least-squares fit for mass + drag from acceleration profile
  - Input: time-series of (time, velocity, applied_force)
  - Model: m × dv/dt = F - drag × v
  - Fit: solve for m and drag via ordinary least squares
- [ ] `src/circle_test_fitter.cpp` — steady-state yaw rate equation → Cf, Cr, Iz
  - Input: (speed, steering_angle, measured_yaw_rate) at steady state
  - Model: steady-state yaw rate = v × δ / (L + K_us × v²) where K_us depends on Cf, Cr
  - Fit: solve for cornering stiffnesses and inertia
- [ ] `src/cog_estimator.cpp` — front/rear axle loads → lf, lr
  - Input: front_load, rear_load, wheelbase
  - Compute: lr = wheelbase × front_load / (front_load + rear_load), lf = wheelbase - lr
- [ ] Only depend on `common` (+ Eigen transitively)
- [ ] Follow conventions: `PascalCase` types, `camelCase` methods, `m_` member prefix
- [ ] Use `std::expected<T,E>` for recoverable errors at public API boundaries

### Phase 4 — Passing Tests

- [ ] All unit tests pass (5% clean, 15% noisy tolerances)
- [ ] Add any additional tests discovered during implementation
- [ ] CI green

```bash
cmake --build build --target param_estimation_tests
cd build && ctest -R param_estimation --output-on-failure
```

### Phase 4.5 — Observability

> **This phase gates module completion.**

- [ ] `ILogger` injected via `common::getLogger("param_estimation")` (mockable in tests)
- [ ] Fit convergence logged at `DEBUG` level (iterations, final residual)
- [ ] Per-iteration residuals logged at `TRACE` level
- [ ] At least one test asserts expected log lines appear via stdout capture
- [ ] Zero `ERROR`-level log entries during all nominal test runs

```bash
cmake --build build --target param_estimation_tests
cd build && ctest -R param_estimation --output-on-failure 2>&1 | grep "\[DEBUG\]\|\[TRACE\]"
```

### Phase 5 — Simulation Integration

- [ ] Calibration scenario: robot runs step-response test (straight-line accel/decel), then circle test
- [ ] Fitters auto-run on collected data, produce fitted `VehicleParams`
- [ ] Fitted params applied to `IDynamicModel` via `setParams()`
- [ ] Integration test: calibrated model matches ground truth within tolerance

### Phase 6 — Frontend Visualization

- [ ] Parameter estimation results overlay (fitted values, residuals, confidence)
- [ ] Toggle visibility from UI panel

### Phase 7 — Docs Polish

- [ ] `README.md` — usage examples, how to run calibration scenarios
- [ ] `docs/theory.md` — one-line redirect: "See `workspace/robotics/control/vehicle_dynamics/docs/theory.md`."
- [ ] Move this file to `repo-plans/modules/done/param_estimation.md`
