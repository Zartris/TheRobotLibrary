# Module: `motor_model`

**Milestone:** M3.5 — Vehicle Dynamics
**Status:** Not Started
**Depends on:** common (including common/robot/ for MotorParams)

---

### Phase 1 — Interface Design

- [ ] `include/motor_model/motor_model.hpp` — `DcMotorModel`
  - Torque-speed curve: τ = τ_stall × (1 - ω/ω_no_load)
  - Back-EMF modeling
  - Gear ratio and efficiency losses
  - Accepts `MotorParams` from `common/robot/`
- [ ] `include/motor_model/actuator_limiter.hpp` — `ActuatorLimiter`
  - Decorator that wraps any `VehicleInput` and clips to what the motor can physically deliver at current wheel speed
- [ ] Document inputs, outputs, and error conditions in header comments

### Phase 2 — Failing Tests (Red)

- [ ] Create `tests/CMakeLists.txt` with Catch2 target (`motor_model_tests`)
- [ ] Stall torque at zero speed matches `MotorParams.stall_torque`
- [ ] Zero torque at no-load speed matches `MotorParams.no_load_speed`
- [ ] Gear ratio scales torque/speed correctly (torque × ratio, speed ÷ ratio)
- [ ] Efficiency losses reduce output torque appropriately
- [ ] `ActuatorLimiter` clips impossible commands — commanded torque exceeding motor capability is clamped
- [ ] Motor saturation reduces acceleration below F=ma prediction
- [ ] Verify tests fail (module not yet implemented)

```bash
cmake --build build --target motor_model_tests
cd build && ctest -R motor_model --output-on-failure
```

### Phase 3 — Implementation (Green)

- [ ] `src/motor_model.cpp` — `DcMotorModel` implementation
  - Linear torque-speed curve: τ = τ_stall × (1 - ω/ω_no_load)
  - Gear ratio applied: output_torque = motor_torque × gear_ratio × efficiency
  - Output speed = motor_speed / gear_ratio
- [ ] `src/actuator_limiter.cpp` — `ActuatorLimiter` implementation
  - Given current wheel speed, compute max available torque from motor curve
  - Clip `VehicleInput` longitudinal force to achievable range
  - Clip braking force similarly
- [ ] Only depend on `common` (+ Eigen transitively)
- [ ] Follow conventions: `PascalCase` types, `camelCase` methods, `m_` member prefix
- [ ] Use `std::expected<T,E>` for recoverable errors at public API boundaries

### Phase 4 — Passing Tests

- [ ] All unit tests pass
- [ ] Add any additional tests discovered during implementation
- [ ] CI green

```bash
cmake --build build --target motor_model_tests
cd build && ctest -R motor_model --output-on-failure
```

### Phase 4.5 — Observability

> **This phase gates module completion.**

- [ ] `ILogger` injected via `common::getLogger("motor_model")` (mockable in tests)
- [ ] Torque clipping events logged at `DEBUG` level (when ActuatorLimiter clips a command)
- [ ] Per-tick torque values logged at `TRACE` level (commanded vs actual)
- [ ] At least one test asserts expected log lines appear via stdout capture
- [ ] Zero `ERROR`-level log entries during all nominal test runs

```bash
cmake --build build --target motor_model_tests
cd build && ctest -R motor_model --output-on-failure 2>&1 | grep "\[DEBUG\]\|\[TRACE\]"
```

### Phase 5 — Simulation Integration

- [ ] Motor torque actual vs commanded added to WebSocket state stream
- [ ] Motor saturation visualized: actual torque vs commanded torque in `dynamic_state`
- [ ] Integration test: motor-limited acceleration visibly lower than F=ma prediction

### Phase 6 — Frontend Visualization

- [ ] Motor saturation indicator (actual vs commanded torque)
- [ ] Toggle visibility from UI panel
- [ ] (Later — M9) Mirror rendering in web frontend Canvas 2D

### Phase 7 — Docs Polish

- [ ] `README.md` — usage examples, motor parameter configuration
- [ ] `docs/theory.md` — one-line redirect: "See `workspace/robotics/control/vehicle_dynamics/docs/theory.md`."
- [ ] Move this file to `repo-plans/modules/done/motor_model.md`
