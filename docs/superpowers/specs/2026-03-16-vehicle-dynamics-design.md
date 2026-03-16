# Design: M3.5 — Vehicle Dynamics

**Date:** 2026-03-16
**Status:** Approved
**Author:** Brainstorming session

---

## Context

TheRobotLibrary's entire simulation and control stack operates on kinematic models — no forces, no mass, no friction. The gap analysis (2026-03-13) identified zero coverage of vehicle dynamics, tire models, actuator physics, or terrain interaction. This spec defines a new milestone M3.5 that adds a physics-based vehicle modeling layer as a parallel alternative to kinematic models.

---

## Architecture Rule (enforced throughout)

> **Robotics modules may only depend on `common`.** Never on each other, simulation, or frontends.

This means `vehicle_dynamics` does not link against `motor_model` or `terrain_model`. The simulation assembles the full physics pipeline: `motor_model` → `vehicle_dynamics` → `terrain_model` feedback. Data flows via shared types in `common/dynamics/`.

---

## Decisions

### 1. Physics depth: Educational dynamics (implementation) + full spectrum (theory)

Implementation scope is educational: linear tire model (cornering stiffness), rigid-body Newton-Euler in 2D (planar forces + yaw moment), simple DC motor model. Enough to demonstrate slip, weight transfer, and actuator limits while keeping everything Eigen-only with no external physics engine.

The theory doc covers the full spectrum from educational through high-fidelity (Pacejka tires, Lagrangian mechanics, multi-body dynamics, MuJoCo integration) so users understand the upgrade paths.

### 2. Parallel interfaces — `IDynamicModel` alongside `IKinematicModel`

`IDynamicModel` is a new interface parallel to `IKinematicModel`. The sim lets users pick which mode: kinematic (existing, fast, no slip) or dynamic (new, physics-based). A `KinematicAdapter` converts velocity commands → force inputs via a PD loop so existing controllers work on the dynamic model unchanged.

This follows the library's existing pattern of swappable implementations behind interfaces (like `IStateEstimator` has EKF/UKF/particle filter).

### 3. Milestone placement: M3.5 (after M3, before M4)

M3.5 depends on M3 (needs the Ackermann/swerve kinematic model variants as test cases for `KinematicAdapter`, and the additional `IController` implementations for integration testing). Note: `IKinematicModel` and `IController` interfaces are defined in M1's `common/` and frozen in M2 — M3 provides the implementations, not the interfaces. M4, M5, M6, M7 branch from M2 independently and are unaffected. M3.5 sits on the control branch: M3 → M3.5 → M11 → M13 → M18.

### 4. Parameter identification included in M3.5

Offline calibration tools (step-response fitting, circle-test fitting, CoG estimation) live in M3.5 alongside the dynamic model they calibrate. M11's RLS estimator remains as the online/adaptive variant — M3.5 provides the offline/calibration variant. M11 gets a documentation note that RLS can operate on structured dynamic model parameters when M3.5 is available.

---

## Modules

### `common/dynamics` (header-only INTERFACE)

Shared types consumed by all dynamics modules and the simulation. Added to existing `common` target.

- `IDynamicModel` — interface: `step(DynamicState, VehicleInput, dt) → DynamicState`, `getParams() → VehicleParams`, `setParams(VehicleParams)` (allows applying calibrated parameters from `param_estimation`)
- `DynamicState` — pose (SE2), linear velocity (vx, vy), yaw rate (omega), acceleration, `DynamicDiagnostics` (front/rear lateral tire forces, front/rear slip angles, front/rear normal loads, total longitudinal force)
- `VehicleInput` — longitudinal force, steering angle, braking force (renamed from `VehicleInput` since steering angle is geometric, not a force)
- `VehicleParams` — mass, yaw inertia (Iz), CoG position (lf, lr — front/rear axle distances), wheel radius, track width
- `TireParams` — cornering stiffness (Cf, Cr), max friction coefficient (mu)
- `MotorParams` — stall torque, no-load speed, gear ratio, efficiency

No implementation — just types and the interface. Same pattern as `common/camera.hpp`.

**Header files:**
- `workspace/robotics/common/include/common/dynamics.hpp` — single header containing all types and the `IDynamicModel` interface (flat file, not a subdirectory — keeps it simple like `camera.hpp`)

### `control/vehicle_dynamics`

The core dynamic model. Implements `IDynamicModel`.

- `BicycleDynamicModel : IDynamicModel` — planar Newton-Euler (3-DOF: x, y, yaw)
- Linear tire model: lateral force = cornering_stiffness × slip_angle
- Weight transfer: longitudinal (braking/accel shifts load front↔rear) and lateral (cornering shifts load left↔right)
- `KinematicAdapter` — wraps `IDynamicModel`, accepts velocity commands (like `IKinematicModel`), internally runs a PD controller to convert velocity → force. Lets existing controllers run on the dynamic model unchanged.
- Tests: straight-line acceleration matches F=ma; steady-state circle matches analytical yaw rate; weight transfer shifts tire load correctly; adapter tracks velocity reference within tolerance

### `control/motor_model`

Actuator dynamics between controller output and wheel torque.

- `DcMotorModel` — torque-speed curve (linear: τ = τ_stall × (1 - ω/ω_no_load)), back-EMF, gear ratio, efficiency losses
- `ActuatorLimiter` — decorator that wraps any `VehicleInput` and clips to what the motor can physically deliver at current wheel speed
- Tests: stall torque at zero speed; zero torque at no-load speed; gear ratio scales torque/speed correctly; limiter clips impossible commands

### `perception/terrain_model`

Per-cell terrain properties that affect tire grip and rolling resistance.

- `TerrainProperties` — friction coefficient (mu), rolling resistance, slope angle
- `TerrainMap` — maps grid cell → `TerrainProperties` (extends occupancy grid metadata, same pattern as wall material IDs in M6's camera renderer)
- `SlipDetector` — `detect(Eigen::Vector2d commanded_velocity, Eigen::Vector2d measured_velocity) → SlipEvent` — takes raw velocity values as arguments (no module references; the caller provides values from whatever dynamic model and state estimator it uses)
- Scenario JSON: `"terrain": {"default_mu": 0.8, "patches": [{"region": [x1,y1,x2,y2], "mu": 0.3, "type": "ice"}]}`
- Tests: high-friction cell → no slip at moderate speed; ice patch → slip detected; slope adds gravitational force component

### `control/param_estimation`

Offline calibration of dynamic model parameters from test data.

- `StepResponseFitter` — straight-line acceleration/deceleration test → estimates mass and drag coefficient via least-squares
- `CircleTestFitter` — steady-state circle at known speed → estimates cornering stiffness (Cf, Cr) and yaw inertia (Iz) from steady-state yaw rate equation
- `CogEstimator` — static weight measurement (front/rear axle loads) → CoG position (lf, lr)
- All fitters return `std::expected<FittedParams, FitError>` with residual and confidence
- Tests: synthetic data with known ground-truth params → fitted values within 5% tolerance; noisy data (SNR 20 dB) → fitted values within 15% tolerance with wider confidence interval

---

## Simulation Integration

- Sim gains `physics_mode` config: `"kinematic"` (default, existing behavior) or `"dynamic"` (new)
- In dynamic mode, sim calls `IDynamicModel::step()` instead of `IKinematicModel::step()`
- `TerrainMap` loaded from scenario JSON alongside existing occupancy grid
- `DynamicState` (velocities, forces, slip angles) added to WebSocket state stream when in dynamic mode
- Motor saturation visualized: actual torque vs commanded torque in state stream
- REST API: `PUT /api/sim/physics_mode {"mode": "kinematic" | "dynamic"}` — switches simulation physics mode at runtime
- WebSocket schema extension (dynamic mode adds to existing `robot` object):
  ```json
  {
    "robot": {
      "dynamic_state": {
        "vx": 1.2, "vy": 0.01, "omega": 0.3,
        "slip_angle_front": 0.02, "slip_angle_rear": 0.01,
        "normal_load_front": 45.0, "normal_load_rear": 55.0,
        "motor_torque_actual": 0.8, "motor_torque_commanded": 1.2
      },
      "terrain": {"mu": 0.8, "type": "asphalt"}
    }
  }
  ```
- Scenario JSON: `"terrain"` is an optional top-level key alongside `"map"`. When absent, all cells use default friction (mu=1.0, no slope)
- Sub-stepping: `IDynamicModel::step()` may sub-step internally when `dt` exceeds the model's stability limit. `BicycleDynamicModel` uses semi-implicit Euler with an internal max step of 1ms for motor electrical dynamics. The sim loop does not need a separate physics tick rate.

---

## Integration Test

Lives in `workspace/simulation/tests/` (sim links modules, not the reverse).

**Kinematic vs dynamic comparison test:** Same PID + DWA controller follows a curved path at increasing speeds. In kinematic mode, perfect tracking. In dynamic mode, slip and overshoot appear above a speed threshold. Test asserts: at low speed, kinematic and dynamic trajectories match within 0.1m; at high speed, dynamic trajectory diverges by > 0.5m (proving the physics layer produces meaningful behavior).

---

## CMake Targets

| Module | Target | Alias |
|--------|--------|-------|
| `control/vehicle_dynamics` | `vehicle_dynamics` | `robotlib::vehicle_dynamics` |
| `control/motor_model` | `motor_model` | `robotlib::motor_model` |
| `perception/terrain_model` | `terrain_model` | `robotlib::terrain_model` |
| `control/param_estimation` | `param_estimation` | `robotlib::param_estimation` |

All link against `common` only.

---

## Module Documentation

Each of the four new modules gets its own `README.md` following the standard template. The theory doc lives in `workspace/robotics/control/vehicle_dynamics/docs/theory.md` and covers all four modules (dynamics, motor, terrain, parameter estimation) since the topics are deeply intertwined. The other three modules' `docs/theory.md` files contain a one-line redirect: "See `workspace/robotics/control/vehicle_dynamics/docs/theory.md`."

---

## Theory Doc Scope

The theory doc (`workspace/robotics/control/vehicle_dynamics/docs/theory.md`) covers the full spectrum:

| Chapter | Content |
|---------|---------|
| 1. Rigid-body mechanics | Newton-Euler equations (2D planar → 3D), free-body diagrams, inertia tensor, CoG computation from geometry + mass distribution |
| 2. Vehicle dynamic models | Bicycle model (single-track), linear tire model (cornering stiffness α), weight transfer (longitudinal + lateral), slip angle vs sideslip angle, steady-state yaw rate, understeer/oversteer gradient |
| 3. Tire models | Linear → Brush model → Pacejka magic formula (B/C/D/E coefficients, combined slip), Dugoff model, when to use which |
| 4. Actuator dynamics | DC motor (torque constant, back-EMF, electrical time constant), gearbox (ratio, efficiency, backlash), actuator bandwidth and its effect on control loop stability |
| 5. Terrain interaction | Coulomb + viscous friction, rolling resistance, slope forces (grade resistance), terrain-dependent friction coefficients, wheel slip ratio (longitudinal) vs slip angle (lateral) |
| 6. Parameter identification | Step-response tests (mass, drag), steady-state circle test (cornering stiffness), least-squares fitting, frequency-domain ID (Bode plots), observability conditions — "which parameters can you actually identify from which experiments?" |
| 7. Dynamics for control | Why dynamics matters for control design: model-based feedforward, computed torque control, how kinematic controllers degrade under slip, when to switch from kinematic to dynamic models (speed thresholds), LQR/MPC on linearized dynamic model |
| 8. Multi-body dynamics | Lagrangian mechanics (generalized coordinates, constraints), Newton-Euler recursive algorithm, articulated bodies (arms, trailers), constraint forces |
| 9. High-fidelity simulation | Physics engines overview (MuJoCo, Bullet, DART, Drake), MJCF/URDF model formats, contact models (spring-damper, complementarity), time integration (semi-implicit Euler, Runge-Kutta), how to wrap MuJoCo as an `IDynamicModel` backend |
| 10. Upgrade paths | From this library's linear tire → Pacejka, from 2D planar → 3D, from single rigid body → multi-body, from custom integrator → MuJoCo plugin |

---

## Exit Criteria

1. `vehicle_dynamics_tests` pass — F=ma, steady-state yaw rate, weight transfer
2. `motor_model_tests` pass — torque curve, gear ratio, actuator limiter
3. `terrain_model_tests` pass — friction variation, slip detection, slope forces
4. `param_estimation_tests` pass — fitted params within 5% (clean) / 15% (noisy, SNR 20 dB) of ground truth
5. `KinematicAdapter` passes — existing velocity-based controllers work on dynamic model within tolerance
6. Integration test: kinematic vs dynamic divergence demonstrated at high speed; motor saturation visibly clips torque and reduces acceleration below F=ma prediction
7. Theory doc covers chapters 1–10 (educational through MuJoCo upgrade path)
8. All modules pass Phase 4.5 Observability gate

---

## NOT IN

- Pacejka magic formula tires (theory doc covers it; implementation stays with linear model)
- 3D dynamics / suspension / multi-body (theory doc covers upgrade path)
- MuJoCo or any external physics engine integration (theory doc covers how to wrap it; no runtime dependency)
- Arm kinematics / manipulation (different axis entirely)
- Online adaptive parameter estimation (M11 — RLS estimator)
- GPU acceleration

---

## Dependency Impact

```
M3 (control upgrades)
 └→ M3.5 (vehicle dynamics)
      └→ M11 (advanced control — RLS becomes online complement to M3.5's offline param ID)
           └→ M13 → M18
```

M4, M5, M6, M7, M9 branch from M2 independently — unaffected by M3.5 insertion.

**M11 interaction note:** M11's RLS parameter estimator currently identifies `[mass, friction, drag]` as lumped black-box parameters. With M3.5's structured dynamic model available, M11 gains the option to estimate structured parameters (cornering stiffness, inertia) instead. This is a documentation note — M11's scope does not change.

---

## Files Created/Modified

| Action | File |
|--------|------|
| Create | `repo-plans/milestones/M3.5-vehicle-dynamics.md` |
| Create | `repo-plans/modules/vehicle_dynamics.md` |
| Create | `repo-plans/modules/motor_model.md` |
| Create | `repo-plans/modules/terrain_model.md` |
| Create | `repo-plans/modules/param_estimation.md` |
| Modify | `repo-plans/README.md` — add M3.5 row to table, update dependency graph |
| Modify | `repo-plans/modules/common.md` — add `common/dynamics` section |
| Modify | `repo-plans/todos.md` — add entry for vehicle dynamics |
