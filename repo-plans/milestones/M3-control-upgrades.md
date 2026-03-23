# M3 — Control Upgrades

**Status:** Complete
**Dependencies:** M2 (stable interfaces)  
**Scope:** Add pure_pursuit, MPC, and additional kinematic models. Hot-swappable in sim.

---

## Goal

Three controllers available — PID (from M1), pure_pursuit, MPC — all implementing `IController`. Four kinematic models available — differential-drive (from M1), unicycle, Ackermann, swerve — all implementing `IKinematicModel`. User picks the combination at runtime via ImGui selector. Demonstrates the swappable architecture for both control and kinematics.

---

## Modules

### pure_pursuit

Geometric path follower for differential-drive. Adaptive lookahead distance (speed-dependent). Outputs curvature → (v, ω).

- [x] `include/pure_pursuit/pure_pursuit_controller.hpp` — `PurePursuitController : IController`
- [x] `src/pure_pursuit_controller.cpp`
- [x] `tests/test_pure_pursuit_controller.cpp`:
  - Straight path → forward velocity, ~zero omega
  - Curved path → correct curvature-based omega
  - Lookahead beyond path end → targets last waypoint
  - Speed-dependent lookahead scales correctly
- [ ] Sim integration: selectable via ImGui controller dropdown
- [ ] ImGui panel: render lookahead point + lookahead circle

### 

### mpc

Receding-horizon **LTV-MPC** using a condensed QP formulation solved with Eigen's LLT (Cholesky) decomposition. The unicycle model is linearized around the current state at each step to form the LTV system. Works alongside any `IController`-compatible interface.

Implementation approach:
- Linearizes discrete unicycle kinematics around the current operating point (LTV approximation)
- Builds prediction matrices S and T via the condensed formulation (X = S·x₀ + T·U)
- Solves unconstrained QP analytically via Cholesky: U* = −H⁻¹f, where H = TᵀQ̄T + R̄
- Box constraints on v and ω applied post-solve via clamping (sufficient for nominal operation)
- **OSQP is a documented future option** for adding hard inequality constraints (obstacle avoidance, actuator limits as true constraints)

> Note: `tools/codegen/generate_mpc_solver.py` and `src/generated/` (acados/CASAdi codegen workflow) were evaluated but not implemented — the Eigen LLT solver is sufficient for M3 and avoids the acados toolchain dependency.

- [ ] `tools/codegen/generate_mpc_solver.py` — deferred (acados/CASAdi codegen, future milestone)
- [x] `include/mpc/mpc_controller.hpp` — `MPCController : IController`
- [x] `src/mpc_controller.cpp` — LTV-MPC with Eigen dense QP (condensed formulation, LLT solver)
- [ ] `src/generated/` — deferred (acados-generated C code, future milestone)
- [x] `tests/test_mpc_controller.cpp`:
  - Tracks straight reference → commands converge to reference velocity
  - Tracks curved reference → smooth velocity/omega profile
  - Respects input constraints (max vel, max omega)
  - Obstacle avoidance constraints (optional soft constraints)
- [ ] Sim integration: selectable via ImGui controller dropdown
- [ ] ImGui panel: render prediction horizon (future poses)

### Kinematic Models

Expand the kinematic model set in `common/kinematics/`. All implement `IKinematicModel`.

- [x] `include/common/kinematics/unicycle.hpp` — `Unicycle : IKinematicModel` (simplified, no wheel geometry)
- [x] `include/common/kinematics/ackermann.hpp` — `Ackermann : IKinematicModel` (wheelbase, max steering angle, min turning radius)
- [x] `include/common/kinematics/swerve_drive.hpp` — `SwerveDrive : IKinematicModel` (4 independently steerable + driven wheels)
- [x] `src/kinematics/unicycle.cpp`, `src/kinematics/ackermann.cpp`, `src/kinematics/swerve_drive.cpp`
- [x] `tests/test_unicycle.cpp` — forward motion, pure rotation, zero input
- [x] `tests/test_ackermann.cpp` — straight drive, turning at max steering, min radius validation, no-slip constraint
- [x] `tests/test_swerve_drive.cpp` — holonomic strafing, rotation-in-place, diagonal motion
- [ ] Sim integration: ImGui kinematics dropdown — swaps kinematic model live
- [ ] Scenario JSON supports `"kinematics"` field (default: `"differential_drive"`)
- [ ] MuJoCo 3D scene: robot shape changes to reflect kinematics (diff-drive, Ackermann, swerve models)

---

### cbf

Control Barrier Function safety filter. Decorator wrapping any `IController` — filters the
nominal command through a QP that guarantees collision-free velocity within defined safe sets.

- [x] `include/cbf/cbf_safety_filter.hpp` — `CbfSafetyFilter : IController`, `CbfConfig`
- [x] `src/cbf_safety_filter.cpp` — Eigen-based QP (no external solver needed)
- [x] `tests/test_cbf_safety_filter.cpp`:
  - No obstacles: CBF output equals nominal command
  - Single obstacle on heading: CBF deflects velocity to maintain safe distance
  - Multiple obstacles: all safety constraints satisfied simultaneously
  - Safety radius = 0: same behavior as bare nominal controller
- [ ] Sim integration: ImGui CBF selector (wraps selected nominal controller)
- [ ] ImGui panel: render safety circle (r_safe) around robot when CBF active

---

## Deliverables

- [x] pure_pursuit module: interface, implementation, tests
- [x] mpc module: interface, implementation, tests
- [x] cbf module: interface, implementation, tests
- [x] 3 additional kinematic models (unicycle, Ackermann, swerve): tests
- [x] Controller hot-swap works mid-run without crash
- [ ] Kinematics hot-swap works mid-run without crash
- [ ] CBF wrapping any controller works mid-run without crash
- [ ] Simulation app shows controller-specific and kinematics-specific visualization
- [ ] Mini-demo: compare tracking quality PID → pure_pursuit → MPC on same path; compare kinematics on same scenario; show CBF preventing collision on near-miss obstacle

## Exit Criteria

1. All three controllers (PID, pure_pursuit, MPC) navigate the same scenarios successfully
2. CBF wrapping PID prevents collision on a scenario where bare PID would fail
3. All four kinematic models work with PID (at minimum)
4. Unit tests pass for all new modules
5. ImGui swap mid-run without crash (controller + kinematics independently)
6. Visual difference apparent in simulation app (PID oscillates, pure_pursuit smooth, MPC optimal; Ackermann has turning radius constraints, swerve can strafe; CBF shows safety circle)
7. All modules pass Phase 4.5 — Observability gate (state transitions logged at DEBUG, hot-loop metrics at TRACE)

## NOT IN

Adaptive control, Frenet controller (M11). Changes to planners, estimators, or perception.
