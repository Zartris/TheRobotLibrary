# M3 — Control Upgrades

**Status:** Not Started  
**Dependencies:** M2 (stable interfaces)  
**Scope:** Add pure_pursuit, MPC, and additional kinematic models. Hot-swappable in sim.

---

## Goal

Three controllers available — PID (from M1), pure_pursuit, MPC — all implementing `IController`. Four kinematic models available — differential-drive (from M1), unicycle, Ackermann, swerve — all implementing `IKinematicModel`. User picks the combination at runtime via REST API. Demonstrates the swappable architecture for both control and kinematics.

---

## Modules

### pure_pursuit

Geometric path follower for differential-drive. Adaptive lookahead distance (speed-dependent). Outputs curvature → (v, ω).

- [ ] `include/pure_pursuit/pure_pursuit_controller.hpp` — `PurePursuitController : IController`
- [ ] `src/pure_pursuit_controller.cpp`
- [ ] `tests/test_pure_pursuit_controller.cpp`:
  - Straight path → forward velocity, ~zero omega
  - Curved path → correct curvature-based omega
  - Lookahead beyond path end → targets last waypoint
  - Speed-dependent lookahead scales correctly
- [ ] Sim integration: selectable via `PUT /api/robot/controller {"type": "pure_pursuit"}`
- [ ] Frontend: render lookahead point + lookahead circle

### mpc

Receding-horizon NMPC via **acados** (with CASAdi for OCP specification). Nonlinear kinematic model — works with any `IKinematicModel`.

acados is the recommended solver because:
- Uses CASAdi internally for symbolic problem definition (great for learning MPC formulation)
- Generates standalone C code optimized for real-time NMPC (portable, no runtime dependency on Python)
- Handles nonlinear kinematics natively (Ackermann, swerve) — unlike linearized QP with OSQP
- State-of-the-art for embedded/real-time robotics MPC (HPIPM backend)

Workflow: define OCP in Python (CASAdi + acados Python API) → acados generates C solver code → C++ wrapper calls generated solver.

- [ ] `tools/codegen/generate_mpc_solver.py` — CASAdi + acados OCP definition, generates C code
- [ ] `include/mpc/mpc_controller.hpp` — `MPCController : IController`
- [ ] `src/mpc_controller.cpp` — wraps acados-generated C solver
- [ ] `src/generated/` — acados-generated C code (committed, regenerated via `tools/codegen/`)
- [ ] `tests/test_mpc_controller.cpp`:
  - Tracks straight reference → commands converge to reference velocity
  - Tracks curved reference → smooth velocity/omega profile
  - Respects input constraints (max vel, max omega)
  - Obstacle avoidance constraints (optional soft constraints)
- [ ] Sim integration: selectable via `PUT /api/robot/controller {"type": "mpc"}`
- [ ] Frontend: render prediction horizon (future poses)

### Kinematic Models

Expand the kinematic model set in `common/kinematics/`. All implement `IKinematicModel`.

- [ ] `include/common/kinematics/unicycle.hpp` — `Unicycle : IKinematicModel` (simplified, no wheel geometry)
- [ ] `include/common/kinematics/ackermann.hpp` — `Ackermann : IKinematicModel` (wheelbase, max steering angle, min turning radius)
- [ ] `include/common/kinematics/swerve_drive.hpp` — `SwerveDrive : IKinematicModel` (4 independently steerable + driven wheels)
- [ ] `src/kinematics/unicycle.cpp`, `src/kinematics/ackermann.cpp`, `src/kinematics/swerve_drive.cpp`
- [ ] `tests/test_unicycle.cpp` — forward motion, pure rotation, zero input
- [ ] `tests/test_ackermann.cpp` — straight drive, turning at max steering, min radius validation, no-slip constraint
- [ ] `tests/test_swerve_drive.cpp` — holonomic strafing, rotation-in-place, diagonal motion
- [ ] Sim integration: `PUT /api/robot/kinematics {"type": "ackermann"}` — swaps kinematic model live
- [ ] Scenario JSON supports `"kinematics"` field (default: `"differential_drive"`)
- [ ] Frontend: robot shape changes to reflect kinematics (triangle for diff-drive, car outline for Ackermann, diamond for swerve)

---

## Deliverables

- [ ] pure_pursuit module: interface, implementation, tests, sim integration
- [ ] mpc module: interface, implementation, tests, sim integration
- [ ] 3 additional kinematic models (unicycle, Ackermann, swerve): tests, sim integration
- [ ] Controller hot-swap works mid-run without crash
- [ ] Kinematics hot-swap works mid-run without crash
- [ ] Frontend shows controller-specific and kinematics-specific visualization
- [ ] Mini-demo: compare tracking quality PID → pure_pursuit → MPC on same path; compare kinematics on same scenario

## Exit Criteria

1. All three controllers navigate the same scenarios successfully
2. All four kinematic models work with PID (at minimum)
3. Unit tests pass for all new modules
4. REST swap mid-run without crash (controller + kinematics independently)
5. Visual difference apparent in frontend (PID oscillates, pure_pursuit smooth, MPC optimal; Ackermann has turning radius constraints, swerve can strafe)

## NOT IN

Changes to planners, estimators, or perception.
