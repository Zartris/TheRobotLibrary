# Module: `vehicle_dynamics`

**Milestone:** M3.5 — Vehicle Dynamics
**Status:** Not Started
**Depends on:** common (including common/kinematics/, common/robot/, common/environment/)

---

### Phase 1 — Interface Design

- [ ] `include/vehicle_dynamics/vehicle_dynamics.hpp` — `BicycleDynamicModel : IDynamicModel`
  - Planar Newton-Euler (3-DOF: x, y, yaw)
  - Linear tire model: lateral force = cornering_stiffness × slip_angle, scaled by `TerrainProperties.mu`
  - Weight transfer: longitudinal (braking/accel shifts load front↔rear) and lateral (cornering shifts load left↔right)
  - `step()` receives `TerrainProperties` for the current cell — friction coefficient scales tire grip, slope angle adds gravitational force component
- [ ] `include/vehicle_dynamics/kinematic_adapter.hpp` — `KinematicAdapter`
  - Wraps `IDynamicModel`, accepts velocity commands (like `IKinematicModel`)
  - Internally runs a PD controller to convert velocity → force (`VehicleInput`)
  - Lets existing controllers run on the dynamic model unchanged
- [ ] Document inputs, outputs, and error conditions in header comments

### Phase 2 — Failing Tests (Red)

- [ ] Create `tests/CMakeLists.txt` with Catch2 target (`vehicle_dynamics_tests`)
- [ ] Straight-line acceleration matches F=ma (known mass, known force → expected acceleration)
- [ ] Steady-state circle matches analytical yaw rate (v/R = v × δ / L for small angles)
- [ ] Weight transfer shifts tire load correctly under braking and cornering
- [ ] `KinematicAdapter` tracks velocity reference within tolerance
- [ ] Low-mu terrain (via `TerrainProperties`) reduces grip — higher slip angles for same lateral acceleration
- [ ] Slope adds correct gravitational force component
- [ ] Verify tests fail (module not yet implemented)

```bash
cmake --build build --target vehicle_dynamics_tests
cd build && ctest -R vehicle_dynamics --output-on-failure
```

### Phase 3 — Implementation (Green)

- [ ] `src/vehicle_dynamics.cpp` — `BicycleDynamicModel::step()` implementation
  - Planar Newton-Euler 3-DOF equations
  - Linear tire model: F_lateral = C × slip_angle × mu
  - Weight transfer (longitudinal + lateral)
  - Semi-implicit Euler with 1ms internal sub-stepping for stability
  - `setParams(VehicleParams)` for applying calibrated parameters from `param_estimation`
- [ ] `src/kinematic_adapter.cpp` — PD controller: velocity error → force command
  - Proportional + derivative gains on (vx_error, vy_error, omega_error)
  - Output: `VehicleInput` (longitudinal force, steering angle, braking force)
- [ ] Only depend on `common` (+ Eigen transitively)
- [ ] Follow conventions: `PascalCase` types, `camelCase` methods, `m_` member prefix
- [ ] Use `std::expected<T,E>` for recoverable errors at public API boundaries

### Phase 4 — Passing Tests

- [ ] All unit tests pass
- [ ] Add any additional tests discovered during implementation
- [ ] CI green

```bash
cmake --build build --target vehicle_dynamics_tests
cd build && ctest -R vehicle_dynamics --output-on-failure
```

### Phase 4.5 — Observability

> **This phase gates module completion.**

- [ ] `ILogger` injected via `common::getLogger("vehicle_dynamics")` (mockable in tests)
- [ ] State transitions logged at `DEBUG` level (init, reset, mode changes, error paths)
- [ ] Slip angles and tire forces logged at `TRACE` level (per-tick diagnostics)
- [ ] At least one test asserts expected log lines appear via stdout capture
- [ ] Zero `ERROR`-level log entries during all nominal test runs

```bash
cmake --build build --target vehicle_dynamics_tests
cd build && ctest -R vehicle_dynamics --output-on-failure 2>&1 | grep "\[DEBUG\]\|\[TRACE\]"
```

### Phase 5 — Simulation Integration

- [ ] Sim gains `physics_mode` config: `"kinematic"` (default) or `"dynamic"`
- [ ] In dynamic mode, sim calls `IDynamicModel::step()` instead of `IKinematicModel::step()`
- [ ] `TerrainMap` (sim-owned) loaded from scenario JSON; sim looks up `TerrainProperties` for robot's current cell
- [ ] `DynamicState` added to bridge state adapter when in dynamic mode
- [ ] ImGui panel: physics mode toggle ("kinematic" | "dynamic")
- [ ] Integration test: sim runs with dynamic model active, state is valid

### Phase 6 — Visualization

- [ ] Dynamic state overlay: slip angles, tire forces, terrain mu displayed
- [ ] Toggle visibility from UI panel

### Phase 7 — Docs Polish

- [ ] `README.md` — usage examples, how to switch kinematic→dynamic mode
- [ ] `docs/theory.md` — comprehensive theory document covering chapters 1–10:
  1. Rigid-body mechanics (Newton-Euler, inertia tensor, CoG)
  2. Vehicle dynamic models (bicycle model, slip angle, understeer/oversteer)
  3. Tire models (linear → brush → Pacejka magic formula)
  4. Actuator dynamics (DC motor, gearbox, bandwidth)
  5. Terrain interaction (friction, rolling resistance, slope)
  6. Parameter identification (step-response, circle test, least-squares)
  7. Dynamics for control (feedforward, computed torque, kinematic degradation)
  8. Multi-body dynamics (Lagrangian, Newton-Euler recursive)
  9. High-fidelity simulation (MuJoCo, Bullet, DART, Drake overview)
  10. Upgrade paths (linear→Pacejka, 2D→3D, single body→multi-body, custom→MuJoCo)
- [ ] Move this file to `repo-plans/modules/done/vehicle_dynamics.md`
