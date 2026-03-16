# M3.5 Vehicle Dynamics — Milestone Documentation Plan

> **For agentic workers:** REQUIRED: Use superpowers:subagent-driven-development (if subagents available) or superpowers:executing-plans to implement this plan. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add M3.5 Vehicle Dynamics milestone to the roadmap — milestone doc, module task files, README updates, common.md additions, and todos entry. NO C++ code.

**Architecture:** M3.5 inserts between M3 and M4 on the control branch: M3 → M3.5 → M11. Three domain modules (vehicle_dynamics, motor_model, param_estimation) plus shared types in common/robot/, common/environment/, and common/kinematics/. TerrainMap lives in simulation, not common.

**Spec:** `docs/superpowers/specs/2026-03-16-vehicle-dynamics-design.md`

---

## Chunk 1: Milestone doc + common.md

### Task 1: Create M3.5 milestone document

**Files:**
- Create: `repo-plans/milestones/M3.5-vehicle-dynamics.md`

- [ ] **Step 1: Create the milestone file**

Write `repo-plans/milestones/M3.5-vehicle-dynamics.md` with the following content. Use the spec at `docs/superpowers/specs/2026-03-16-vehicle-dynamics-design.md` as the authoritative source for all details.

```markdown
# M3.5 — Vehicle Dynamics

**Status:** Not Started
**Dependencies:** M3 (control upgrades — Ackermann/swerve kinematic variants, IController implementations)
**Scope:** Physics-based vehicle modeling as a parallel alternative to kinematic models. Dynamic vehicle model with tire physics, motor/drivetrain modeling, terrain interaction via common/ types, and offline parameter identification.

---

## Goal

The robot gains a physics layer. Users can switch between kinematic mode (existing, no slip) and dynamic mode (forces, slip, weight transfer). Controllers designed for kinematics still work via a velocity→force adapter, but users can see how they degrade under slip and actuator limits. A headless integration test demonstrates the difference: same controller, same path, kinematic vs dynamic — showing drift divergence under high-speed cornering.

---

## Architecture Note

`IDynamicModel` is a parallel interface to `IKinematicModel` — both live in `common/kinematics/`. The sim lets users pick which mode. All dynamics modules depend only on `common`. The simulation assembles the full physics pipeline: `motor_model` → `vehicle_dynamics` (with `TerrainProperties` passed per-tick from sim-owned `TerrainMap`).

**common/ reorganization (introduced in M3.5):**

```
common/
├── *(flat)*            ← Pose2D, Twist, IController, IStateEstimator, etc. (existing)
├── kinematics/         ← IKinematicModel (existing) + IDynamicModel (NEW)
├── robot/              ← NEW: VehicleParams, MotorParams, WheelConfig, TireParams
├── environment/        ← NEW: TerrainProperties, SlipDetector, SlipEvent
├── logging/            ← (existing)
├── transforms/         ← (existing)
├── camera.hpp          ← (existing, M6)
└── noise_models/       ← (existing, M21)
```

---

## Modules

### common/kinematics/ addition

`IDynamicModel` added alongside existing `IKinematicModel`.

- [ ] `include/common/kinematics/i_dynamic_model.hpp` — `IDynamicModel`: `step(DynamicState, VehicleInput, TerrainProperties, dt) → DynamicState`, `getParams() → VehicleParams`, `setParams(VehicleParams)`
- [ ] `DynamicState` — pose (SE2), linear velocity (vx, vy), yaw rate (omega), acceleration, `DynamicDiagnostics` (front/rear lateral tire forces, front/rear slip angles, front/rear normal loads, total longitudinal force)
- [ ] `VehicleInput` — longitudinal force, steering angle, braking force

### common/robot/

Physical parameter types.

- [ ] `include/common/robot/vehicle_params.hpp` — `VehicleParams` (mass, Iz, lf, lr, wheel_radius, track_width), `TireParams` (Cf, Cr, mu), `MotorParams` (stall_torque, no_load_speed, gear_ratio, efficiency), `WheelConfig` (radius, width, position relative to CoG)

### common/environment/

Environment description types.

- [ ] `include/common/environment/terrain.hpp` — `TerrainProperties` (mu, rolling_resistance, slope_angle), `SlipDetector::detect(commanded_vel, measured_vel) → SlipEvent`, `SlipEvent` (detected, slip_ratio, severity)

### vehicle_dynamics

- [ ] **Phase 1 — Interface Design**
  - [ ] `include/vehicle_dynamics/vehicle_dynamics.hpp` — `BicycleDynamicModel : IDynamicModel`, `KinematicAdapter`
- [ ] **Phase 2 — Failing Tests (Red)**
  - [ ] `tests/vehicle_dynamics_tests.cpp` — straight-line F=ma; steady-state circle yaw rate; weight transfer; KinematicAdapter velocity tracking; low-mu terrain reduces grip
- [ ] **Phase 3 — Implementation (Green)**
  - [ ] `src/vehicle_dynamics.cpp` — planar Newton-Euler (3-DOF), linear tire model (lateral force = Cf × slip_angle × TerrainProperties.mu), weight transfer (longitudinal + lateral), semi-implicit Euler with 1ms internal sub-stepping
  - [ ] `src/kinematic_adapter.cpp` — PD controller converting velocity commands → VehicleInput
- [ ] **Phase 4 — Passing Tests**
- [ ] **Phase 4.5 — Observability** — `common::getLogger("vehicle_dynamics")`, state transitions at DEBUG, slip angles at TRACE
- [ ] **Phase 5 — Simulation Integration** — sim `physics_mode` toggle, `PUT /api/sim/physics_mode`, `TerrainMap` lookup, WebSocket `dynamic_state` extension
- [ ] **Phase 6 — Frontend** — dynamic state overlay (slip angles, tire forces, terrain mu)
- [ ] **Phase 7 — Docs Polish** — README + theory.md (chapters 1–10, covers all M3.5 topics)

### motor_model

- [ ] **Phase 1 — Interface Design**
  - [ ] `include/motor_model/motor_model.hpp` — `DcMotorModel`, `ActuatorLimiter`
- [ ] **Phase 2 — Failing Tests (Red)**
  - [ ] `tests/motor_model_tests.cpp` — stall torque at zero speed; zero torque at no-load speed; gear ratio scales correctly; ActuatorLimiter clips impossible commands; motor saturation reduces acceleration below F=ma
- [ ] **Phase 3 — Implementation (Green)**
  - [ ] `src/motor_model.cpp` — torque-speed curve (τ = τ_stall × (1 - ω/ω_no_load)), gear ratio, efficiency losses
  - [ ] `src/actuator_limiter.cpp` — clips VehicleInput to motor capability at current wheel speed
- [ ] **Phase 4 — Passing Tests**
- [ ] **Phase 4.5 — Observability** — `common::getLogger("motor_model")`, torque clipping events at DEBUG, per-tick torque at TRACE
- [ ] **Phase 5 — Simulation Integration** — motor_torque_actual vs motor_torque_commanded in WebSocket stream
- [ ] **Phase 6 — Frontend** — motor saturation indicator
- [ ] **Phase 7 — Docs Polish** — README + theory.md redirect to vehicle_dynamics/docs/theory.md

### param_estimation

- [ ] **Phase 1 — Interface Design**
  - [ ] `include/param_estimation/param_estimation.hpp` — `StepResponseFitter`, `CircleTestFitter`, `CogEstimator`, `FittedParams`, `FitError`
- [ ] **Phase 2 — Failing Tests (Red)**
  - [ ] `tests/param_estimation_tests.cpp` — synthetic clean data → fitted within 5%; noisy data (SNR 20 dB) → within 15%; insufficient data → FitError returned
- [ ] **Phase 3 — Implementation (Green)**
  - [ ] `src/step_response_fitter.cpp` — least-squares fit for mass + drag from acceleration profile
  - [ ] `src/circle_test_fitter.cpp` — steady-state yaw rate equation → Cf, Cr, Iz
  - [ ] `src/cog_estimator.cpp` — front/rear axle loads → lf, lr
- [ ] **Phase 4 — Passing Tests**
- [ ] **Phase 4.5 — Observability** — `common::getLogger("param_estimation")`, fit convergence at DEBUG, residuals at TRACE
- [ ] **Phase 5 — Simulation Integration** — calibration scenario: robot runs step + circle tests, auto-fits params
- [ ] **Phase 6 — Frontend** — parameter estimation results overlay
- [ ] **Phase 7 — Docs Polish** — README + theory.md redirect to vehicle_dynamics/docs/theory.md

---

## Simulation: TerrainMap + Physics Mode

Sim-owned terrain lookup and physics mode toggle.

- [ ] `workspace/simulation/include/simulation/terrain_map.hpp` — `TerrainMap`: grid-cell → `TerrainProperties` lookup; loaded from scenario JSON
- [ ] `workspace/simulation/src/terrain_map.cpp` — parse `"terrain"` key from scenario JSON; default mu=1.0, no slope when absent
- [ ] Physics mode toggle: `"kinematic"` (default) or `"dynamic"` — selects `IKinematicModel::step()` vs `IDynamicModel::step()`
- [ ] REST API: `PUT /api/sim/physics_mode {"mode": "kinematic" | "dynamic"}`
- [ ] WebSocket extension: `dynamic_state` object + `terrain` object under `robot` (see spec for schema)
- [ ] Scenario JSON: `"terrain": {"default_mu": 0.8, "patches": [{"region": [x1,y1,x2,y2], "mu": 0.3, "type": "ice"}]}`
- [ ] `workspace/simulation/tests/test_terrain_map.cpp` — known grid → correct friction lookup; missing terrain key → defaults

---

## Integration Test

- [ ] `workspace/simulation/tests/test_dynamics_integration.cpp`
  - Same PID + DWA controller, curved path, increasing speeds
  - Kinematic mode: perfect tracking
  - Dynamic mode: slip and overshoot above speed threshold
  - Assert: low speed → trajectories match within 0.1m; high speed → dynamic diverges by > 0.5m
  - Assert: motor saturation clips torque, reduces acceleration below F=ma prediction

---

## Exit Criteria

1. `vehicle_dynamics_tests` pass — F=ma, steady-state yaw rate, weight transfer
2. `motor_model_tests` pass — torque curve, gear ratio, actuator limiter
3. `common` terrain tests pass — SlipDetector, low-mu grip reduction, slope forces
4. `param_estimation_tests` pass — 5% clean / 15% noisy (SNR 20 dB)
5. `KinematicAdapter` passes — velocity-based controllers work on dynamic model
6. Integration test: kinematic vs dynamic divergence; motor saturation visible
7. Theory doc covers chapters 1–10 (educational through MuJoCo upgrade path)
8. All modules pass Phase 4.5 Observability gate

## NOT IN

- Pacejka magic formula tires (theory doc only)
- 3D dynamics / suspension / multi-body (theory doc only)
- MuJoCo or external physics engine (theory doc only)
- Arm kinematics / manipulation
- Online adaptive parameter estimation (→ M11 RLS)
- GPU acceleration
```

- [ ] **Step 2: Verify the file**

Run: `head -5 repo-plans/milestones/M3.5-vehicle-dynamics.md`
Expected: `# M3.5 — Vehicle Dynamics` header visible.

- [ ] **Step 3: Commit**

```bash
git add repo-plans/milestones/M3.5-vehicle-dynamics.md
git commit -m "plan(M3.5): create Vehicle Dynamics milestone document"
```

---

### Task 2: Add common/robot/ and common/environment/ sections to common.md

**Files:**
- Modify: `repo-plans/modules/common.md`

- [ ] **Step 1: Add common/robot/ and common/environment/ sections**

Append to the end of `repo-plans/modules/common.md` (after the M6 camera.hpp section):

```markdown
---

## M3.5 Addition: `common/robot/` — Physical Parameter Types

**Milestone:** M3.5 — Vehicle Dynamics
**Tracked here** because these are header-only additions to the `common` library target.

- [ ] `include/common/robot/vehicle_params.hpp` — `VehicleParams` (mass, yaw inertia Iz, CoG position lf/lr, wheel_radius, track_width), `TireParams` (cornering stiffness Cf/Cr, max friction mu), `MotorParams` (stall_torque, no_load_speed, gear_ratio, efficiency), `WheelConfig` (radius, width, position relative to CoG)
- [ ] Tests: construction with defaults; VehicleParams validates positive mass/inertia

## M3.5 Addition: `common/kinematics/` — IDynamicModel Interface

**Milestone:** M3.5 — Vehicle Dynamics
**Tracked here** because `IDynamicModel` is added alongside existing `IKinematicModel` in `common/kinematics/`.

- [ ] `include/common/kinematics/i_dynamic_model.hpp` — `IDynamicModel`: `step(DynamicState, VehicleInput, TerrainProperties, dt) → DynamicState`, `getParams() → VehicleParams`, `setParams(VehicleParams)`; `DynamicState` (pose SE2, vx, vy, omega, acceleration, `DynamicDiagnostics`); `VehicleInput` (longitudinal force, steering angle, braking force)
- [ ] Tests: DynamicState default construction; VehicleInput zero-init

## M3.5 Addition: `common/environment/` — Terrain Types

**Milestone:** M3.5 — Vehicle Dynamics
**Tracked here** because these are header-only additions to the `common` library target.

- [ ] `include/common/environment/terrain.hpp` — `TerrainProperties` (mu, rolling_resistance, slope_angle), `SlipDetector::detect(commanded_vel, measured_vel) → SlipEvent`, `SlipEvent` (detected bool, slip_ratio, severity)
- [ ] Tests: SlipDetector with matching velocities → no slip; divergent velocities → slip detected; TerrainProperties default mu=1.0
```

- [ ] **Step 2: Verify the additions**

Run: `grep -c "M3.5" repo-plans/modules/common.md`
Expected: At least 3 matches (one per section header).

- [ ] **Step 3: Commit**

```bash
git add repo-plans/modules/common.md
git commit -m "plan(common): add common/robot/, common/kinematics/ IDynamicModel, common/environment/ sections for M3.5"
```

---

## Chunk 2: Module task files

### Task 3: Create vehicle_dynamics module task file

**Files:**
- Create: `repo-plans/modules/vehicle_dynamics.md`

- [ ] **Step 1: Create the file**

Write `repo-plans/modules/vehicle_dynamics.md` following the 7-phase module task template from `repo-plans/module-task-template.md`. Key details from the spec:

- **Milestone:** M3.5 — Vehicle Dynamics
- **Depends on:** common (including common/kinematics/, common/robot/, common/environment/)
- **Phase 1:** `BicycleDynamicModel : IDynamicModel`, `KinematicAdapter`
- **Phase 2:** F=ma straight-line; steady-state circle yaw rate; weight transfer; KinematicAdapter velocity tracking; low-mu terrain grip reduction; slope gravitational force
- **Phase 3:** Planar Newton-Euler 3-DOF, linear tire model (Cf × slip_angle × mu), weight transfer (longitudinal + lateral), semi-implicit Euler with 1ms sub-stepping, KinematicAdapter PD loop
- **Phase 4.5:** `common::getLogger("vehicle_dynamics")`, state transitions at DEBUG, slip angles/tire forces at TRACE
- **Phase 5:** Sim physics_mode toggle, TerrainMap lookup per-tick, WebSocket dynamic_state, REST PUT /api/sim/physics_mode
- **Phase 7:** Theory doc at `workspace/robotics/control/vehicle_dynamics/docs/theory.md` covers chapters 1–10 (all M3.5 modules)

- [ ] **Step 2: Commit**

```bash
git add repo-plans/modules/vehicle_dynamics.md
git commit -m "plan(modules): add vehicle_dynamics task file for M3.5"
```

---

### Task 4: Create motor_model module task file

**Files:**
- Create: `repo-plans/modules/motor_model.md`

- [ ] **Step 1: Create the file**

Write `repo-plans/modules/motor_model.md` following the 7-phase template. Key details:

- **Milestone:** M3.5 — Vehicle Dynamics
- **Depends on:** common (including common/robot/ for MotorParams)
- **Phase 1:** `DcMotorModel`, `ActuatorLimiter`
- **Phase 2:** Stall torque at zero speed; zero torque at no-load speed; gear ratio scales torque/speed; ActuatorLimiter clips impossible commands
- **Phase 3:** Torque-speed curve (τ = τ_stall × (1 - ω/ω_no_load)), back-EMF, gear ratio, efficiency
- **Phase 4.5:** `common::getLogger("motor_model")`, torque clipping at DEBUG, per-tick torque at TRACE
- **Phase 7:** theory.md is a one-line redirect to `workspace/robotics/control/vehicle_dynamics/docs/theory.md`

- [ ] **Step 2: Commit**

```bash
git add repo-plans/modules/motor_model.md
git commit -m "plan(modules): add motor_model task file for M3.5"
```

---

### Task 5: Create param_estimation module task file

**Files:**
- Create: `repo-plans/modules/param_estimation.md`

- [ ] **Step 1: Create the file**

Write `repo-plans/modules/param_estimation.md` following the 7-phase template. Key details:

- **Milestone:** M3.5 — Vehicle Dynamics
- **Depends on:** common (including common/robot/ for VehicleParams, common/kinematics/ for IDynamicModel)
- **Phase 1:** `StepResponseFitter`, `CircleTestFitter`, `CogEstimator`, `FittedParams`, `FitError`
- **Phase 2:** Synthetic clean data → 5% tolerance; noisy (SNR 20 dB) → 15%; insufficient data → FitError
- **Phase 3:** Least-squares mass+drag, steady-state yaw rate equation for Cf/Cr/Iz, axle load → CoG
- **Phase 4.5:** `common::getLogger("param_estimation")`, fit convergence at DEBUG, residuals at TRACE
- **Phase 7:** theory.md is a one-line redirect to `workspace/robotics/control/vehicle_dynamics/docs/theory.md`

- [ ] **Step 2: Commit**

```bash
git add repo-plans/modules/param_estimation.md
git commit -m "plan(modules): add param_estimation task file for M3.5"
```

---

## Chunk 3: README and todos updates

### Task 6: Update README.md — add M3.5 row and update dependency graph

**Files:**
- Modify: `repo-plans/README.md`

- [ ] **Step 1: Add M3.5 row to milestone table**

In `repo-plans/README.md`, insert a new row after the M3 row (line 16) and before the M4 row (line 17):

Old text to find:
```
| **M3** | [Control Upgrades](milestones/M3-control-upgrades.md) | pure_pursuit, MPC (acados), CBF safety filter, additional kinematics | Not Started |
| **M4** | [Perception Upgrades](milestones/M4-perception-upgrades.md) | obstacle_detection, RANSAC, inflation, dynamic obstacles | Not Started |
```

Replace with:
```
| **M3** | [Control Upgrades](milestones/M3-control-upgrades.md) | pure_pursuit, MPC (acados), CBF safety filter, additional kinematics | Not Started |
| **M3.5** | [Vehicle Dynamics](milestones/M3.5-vehicle-dynamics.md) | Bicycle dynamic model, DC motor, terrain interaction, param identification | Not Started |
| **M4** | [Perception Upgrades](milestones/M4-perception-upgrades.md) | obstacle_detection, RANSAC, inflation, dynamic obstacles | Not Started |
```

- [ ] **Step 2: Update dependency graph**

Replace the existing dependency graph block in `repo-plans/README.md`. Find the old graph (starts with `M0 (infra)`, ends with `M10 (polish & showcase) ←── M8, M9`) and replace it with:

```
M0 (infra)
 └→ M1 (minimum viable robot + common/transforms)
      └→ M2 (hardening & testing)
           ├→ M3 (control upgrades)
           │    └→ M3.5 (vehicle dynamics) → M11 (advanced control) → M13 (classical & optimal control)
           │         └→ M13 ────────────────────────────────────────→ M18 (advanced nonlinear control)
           │         └→ M13 ────────────────────────────────────────→ M22 (optimal output feedback)
           ├→ M4 (perception upgrades)
           │    └→ M6 (visual perception)
           │    │    ├→ M6.5 (SLAM) ←── M5 ──── M14 (state estimation II) → M15 (VIO)
           │    │    ├→ M15 (VIO) ←── M14
           │    │    ├→ M17 (camera perception II)
           │    │    ├→ M19 (depth perception & 3D) ←── M4
           │    │    ├→ M22 (automotive perception) ←── M4
           │    │    └→ M23 (lattice & semantic)
           │    └→ M19 (3D detection) ←── M6
           │    └→ M20 (planning upgrades III) ←── M7
           ├→ M5 (state estimation upgrades)
           │    └→ M14 (state estimation II) → M15 (VIO)
           │         └→ M21 (estimation & test foundations)
           ├→ M7 (advanced planning) ←── M4
           │    ├→ M16 (planning upgrades II) ←── M4
           │    ├→ M20 (planning upgrades III) ←── M4
           │    └→ M23 (lattice & semantic) ←── M6
           ├→ M8 (multi-robot) ←── M3, M7    → M12 (fleet management)
           │                                       └→ M24 (fleet operations)
           ├→ M9 (web frontend)
           └→ M10 (polish & showcase) ←── M8, M9
```

- [ ] **Step 3: Update the dependency paragraph**

After the graph, in the paragraph starting "After M2, milestones M3–M5 and M9...", add after the sentence about M11:

`M3.5 requires M3 (Ackermann/swerve kinematic variants for KinematicAdapter testing). M3.5 does not block M4/M5/M6/M7 — they branch from M2 independently.`

- [ ] **Step 4: Verify links**

Run: `test -f repo-plans/milestones/M3.5-vehicle-dynamics.md && echo "OK" || echo "MISSING"`
Expected: `OK`

- [ ] **Step 5: Commit**

```bash
git add repo-plans/README.md
git commit -m "plan(README): add M3.5 Vehicle Dynamics to milestone table and dependency graph"
```

---

### Task 7: Add vehicle dynamics entry to todos.md

**Files:**
- Modify: `repo-plans/todos.md`

- [ ] **Step 1: Add entry to Quick todo list**

In `repo-plans/todos.md`, add a new resolved entry after the existing P2 entries (after the `feature_extraction` resolution entry, before `[P2] state_estimation/ukf`). Insert:

```markdown
### ✅ Vehicle Dynamics — Dynamic model, motor, terrain, param ID
- **Domain:** `control/vehicle_dynamics`, `control/motor_model`, `control/param_estimation`, `common/robot/`, `common/environment/`
- **✅ Resolution (2026-03-16):** M3.5 milestone created. `BicycleDynamicModel : IDynamicModel` (linear tire, Newton-Euler 2D), `DcMotorModel` + `ActuatorLimiter`, `StepResponseFitter`/`CircleTestFitter`/`CogEstimator` for offline calibration. Shared types in `common/robot/` (VehicleParams, MotorParams, TireParams) and `common/environment/` (TerrainProperties, SlipDetector). TerrainMap owned by simulation. Theory doc covers full spectrum (Pacejka, Lagrangian, MuJoCo upgrade path).
```

- [ ] **Step 2: Commit**

```bash
git add repo-plans/todos.md
git commit -m "plan(todos): add vehicle dynamics entry with M3.5 resolution"
```

---

## Verification

After all tasks complete, run these checks:

- [ ] `test -f repo-plans/milestones/M3.5-vehicle-dynamics.md` → exists
- [ ] `test -f repo-plans/modules/vehicle_dynamics.md` → exists
- [ ] `test -f repo-plans/modules/motor_model.md` → exists
- [ ] `test -f repo-plans/modules/param_estimation.md` → exists
- [ ] `grep "M3.5" repo-plans/README.md | wc -l` → at least 3 (table + graph + paragraph)
- [ ] `grep "common/robot/" repo-plans/modules/common.md` → match found
- [ ] `grep "common/environment/" repo-plans/modules/common.md` → match found
- [ ] `grep "Vehicle Dynamics" repo-plans/todos.md` → match found
- [ ] All README milestone links resolve: `grep -oP 'milestones/[^)]+\.md' repo-plans/README.md | while read f; do [ -f "repo-plans/$f" ] && echo "OK: $f" || echo "MISSING: $f"; done`
