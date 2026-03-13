# Session Chat Summary — March 13, 2026

## 1. Conversation Overview

**Primary Objectives:**
- Create comprehensive development milestones for TheRobotLibrary's `repo-plans/` directory using brainstorm→plan→execute→review workflow ✅ DONE
- Fix review findings from superpower-review agent: Critical ✅ DONE, Important ✅ DONE, Minor ✅ DONE
- Incorporate 4 user changes into milestone/module plans: ✅ ALL DONE
  1. Swappable robot kinematics (swerve-drive, differential-drive, ackermann, unicycle) — `IKinematicModel` in `common/kinematics/`
  2. Common map representation — `OccupancyGrid` + `Map` types in `common/`
  3. Camera-based SLAM — full camera simulation section in M6, projective landmark + raycaster options
  4. MPC framework — **acados** chosen (CasADi for OCP spec); rationale documented in M3

**Session Context:**
Project has complete scaffold (all CMakeLists.txt, READMEs, theory.md docs) but ZERO source code. Milestone planning was completed through brainstorm→plan→execute→review cycle. Review found 16 issues; all fixed. All 4 new architectural decisions have been incorporated into the milestone/module plans. Plans are fully up to date — ready to begin implementation.

**User Intent Evolution:**
Original → simplest end-to-end first (M1), then upgrade. Added and integrated: swappable robot kinematics, shared map type, camera SLAM, and acados for MPC.

---

## 2. Technical Foundation

- **Project root:** `/home/zartris/code/cpp/TheRobotLibrary/workspace/` — C++20 robotics library
- **Build system:** CMake 3.20+, FetchContent, 38+ CMakeLists.txt already complete
- **Dependencies (deps.cmake):** Eigen 3.4, Catch2 v3.5.2, nlohmann_json v3.11.3, Crow v1.2, OSQP v0.6.3, cpp-httplib v0.15.3, IXWebSocket v11.4.4, SDL2, ImGui v1.90.4, g2o (commented out)
- **Swappable interface architecture:** 5 abstract interfaces in `common/interfaces/`:
  - `IController`: `compute(Pose2D current, Pose2D target, double dt) → Twist`
  - `IGlobalPlanner`: `plan(Pose2D start, Pose2D goal, OccupancyGrid grid) → optional<Path>`
  - `ILocalPlanner`: `compute(Pose2D, Twist, Path, LaserScan, OccupancyGrid, dt) → Twist`
  - `IStateEstimator`: `predict(Twist, dt)`, `update(SensorData)`, `getPose()`, `getCovariance()`
  - `IVelocityProfiler`: `profile(Path, max_vel, max_accel) → TimedPath`
- **REST API for module hot-swapping:** PUT /api/robot/controller, /global_planner, etc.
- **RobotPipeline struct:** holds `unique_ptr<IFoo>` for each layer, swappable at runtime
- **Docker dev env:** Dockerfile.dev + docker-compose.dev.yml
- **Robot kinematics:** `IKinematicModel` interface in `common/kinematics/`; impls: `DifferentialDrive`, `AckermannModel`, `UniCycleModel`, `SwerveDrive`
- **Common map:** `OccupancyGrid` and `Map` types live in `common/`, shared by all modules
- **Camera simulation:** projective landmark camera or 2.5D raycaster depth image (decide during M6)
- **MPC solver:** acados (real-time NMPC); OCP defined via CasADi + acados Python API → generated C code

---

## 3. Codebase Status

### Milestone Docs (all in `repo-plans/milestones/`)

| File | Status | Notes |
|------|--------|-------|
| `M0-dev-infrastructure.md` | ✅ | Externally modified — AI agent instructions, devcontainer, CI/CD, clang-format/tidy, pre-commit |
| `M1-minimum-viable-robot.md` | ✅ | Critical fix #1 applied (sub-phase ordering); 12 sub-phases M1-A through M1-L |
| `M2-hardening-testing.md` | ✅ | ≥80% coverage, interface freeze |
| `M3-control-upgrades.md` | ✅ | pure_pursuit + MPC; **user viewing this file** |
| `M4-perception-upgrades.md` | ✅ | obstacle_detection, RANSAC, inflation, dynamic obstacles |
| `M5-state-estimation-upgrades.md` | ✅ | particle filter (MCL) |
| `M6-slam.md` | ✅ | EKF-SLAM + lidar SLAM; user wants camera-based SLAM added |
| `M7-advanced-planning.md` | ✅ | Externally modified — dijkstra, rrt, spline_fitting, teb, time_optimal |
| `M8-multi-robot.md` | ✅ | N-robot sim + ORCA → priority → CBS → DMPC → MADER |
| `M9-web-frontend.md` | ✅ | TypeScript/React Canvas 2D |
| `M10-polish-showcase.md` | ✅ | docs site, examples, packaging |

### Per-Module Task Files (all in `repo-plans/modules/`)

11 files for M1: `common.md`, `ray_casting.md`, `occupancy_grid.md`, `lidar_processing.md`, `ekf.md`, `pid.md`, `astar.md`, `dwa.md`, `velocity_profiling.md`, `simulation.md`, `native_frontend.md`

### Other Files

- `repo-plans/README.md` ✅ — Updated with full roadmap (critical fix #2 for dependency graph applied)
- `repo-plans/module-task-template.md` ✅
- `workspace/architecture.md` ✅ — Critical fix #3 (REST endpoints) applied
- `.github/superpower/brainstorm/2026-03-12-development-roadmap-design.md` ✅

---

## 4. Review Findings — Final Status

### Critical (ALL 3 FIXED ✅)
1. M1 sub-phase ordering — dependency groups corrected
2. README dependency graph — M8 branches from M3+M7 fixed
3. architecture.md REST endpoints — module-swap endpoints added

### Important (ALL 7 FIXED ✅)
4. `ISlamEstimator` interface note added to M1-A / M2
5. M7's M3 dependency changed to soft dependency
6. `ILocalPlanner` updated to accept `PerceptionContext` struct
7. M8 split into sub-milestones
8. `POST /api/sim/step` endpoint added
9. Velocity profiler role/type mismatch clarified
10. Workflow note added about module files created per-milestone

### Minor (ALL 6 FIXED ✅)
11. M10 deps clarified
12. M9 dep inconsistency resolved
13. Web deployment model documented
14. Template Phase 5 reference corrected
15. Perception interface clarified
16. architecture.md scope note added

---

## 5. New User Requirements — All Integrated ✅

### 1. Robot Kinematics Abstraction
- `IKinematicModel` interface in `common/kinematics/` — `step()`, `getControlLimits()`, `toTwist()`, `fromTwist()`
- Impls: `DifferentialDrive`, `AckermannModel`, `UniCycleModel`, `SwerveDrive`
- Reference: https://control.ros.org/rolling/doc/ros2_controllers/doc/mobile_robot_kinematics.html
- Integrated into `repo-plans/modules/common.md` (M1-A tasks) and M3 (MPC uses `IKinematicModel`)

### 2. Common Map Representation
- `OccupancyGrid` and `Map` types in `common/`, shared by global planner, local planner, SLAM, perception
- Integrated into `repo-plans/modules/common.md`

### 3. Camera-Based SLAM
- Full section added to `repo-plans/milestones/M6-slam.md`
- Camera simulation options: projective landmark camera (simplest) or 2.5D raycaster depth image (most realistic)
- Tasks: `CameraIntrinsics`, `CameraImage`, `camera_sensor.hpp`, noise model, scenario JSON, WebSocket stream
- Visual SLAM: feature-based (ORB/BRIEF-style), loop closure, landmark map

### 4. MPC Framework — acados chosen
- **acados** selected for real-time NMPC (HPIPM/qpOASES solvers, purpose-built for embedded)
- **CasADi** used only for OCP specification → generates C code → acados solver
- Workflow: Python OCP definition → `acados` generates C solver → C++ wrapper in `src/mpc_controller.cpp`
- Rationale and theory documented in `M3-control-upgrades.md`

---

## 6. Active Work State

**Current Status:** All planning complete. All milestone docs, module task files, review fixes, and architectural integrations are done.

**Ready for:** M0 execution (dev infrastructure) → M1 implementation (minimum viable robot).

**No pending decisions** — all architectural choices have been made and documented.

---

## 7. Prior Session History (Phases 1–4)

### Phase 1: Full Project Scaffold
- Complete project structure under `workspace/`
- 33 documentation files: README.md, architecture.md, all sub-module READMEs and theory.md
- Directory structure: robotics/(common, control, perception, state_estimation, motion_planning), simulation/, frontends/
- Docker dev environment setup

### Phase 2: CMakeLists.txt Scaffolding
- `workspace/cmake/deps.cmake` with FetchContent for all dependencies
- 38 CMakeLists.txt files across entire project (all verified present)
- Architecture.md updated with "Building" section

### Phase 3: Milestone Planning
- User philosophy: "Pick the easy implementation of each module to make a stable simulation first"
- Brainstorm agent → 12-milestone "Deep Domain Waves" structure
- User redirected to "end-to-end first" approach
- Plan agent produced ~1000-line detailed implementation plan
- Created all milestone docs (M0-M10), 11 per-module task files, updated README.md
- Review agent: "Approve with changes" with 16 findings (3 critical, 7 important, 6 minor)

### Phase 4: Review Fix Session
- All 16 review findings fixed (critical + important + minor)
- M0 and M7 were externally modified between sessions
- All 39 expected files verified present via `find` command

---

## 8. Next Steps

1. **Start M0** — dev infrastructure: devcontainer, clang-format/tidy, pre-commit, CI/CD skeleton
2. **Start M1** — minimum viable robot, following M1-A through M1-L sub-phases in order
3. Begin with `repo-plans/modules/common.md` tasks (M1-A) as the foundation for all other modules
