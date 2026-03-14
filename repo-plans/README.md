# TheRobotLibrary — Roadmap

This folder tracks the big picture: milestones define **what** gets built and in what order; module task files track **per-module** progress within each milestone.

## Philosophy

Get the simplest version of every layer running end-to-end first (M1), then upgrade one module at a time. Each upgrade is independently testable because the full robot loop already works. The simulator lets users pick **any combination** of modules and test them live.

## Milestone Sequence

| # | Name | Focus | Status |
|---|------|-------|--------|
| **M0** | [Dev Infrastructure](milestones/M0-dev-infrastructure.md) | AI agent configs, devcontainer, CI/CD, linting, fleet_management scaffold | Not Started |
| **M1** | [Minimum Viable Robot](milestones/M1-minimum-viable-robot.md) | common + lidar + grid + A* + DWA + PID + EKF + sim + frontend | Not Started |
| **M2** | [Hardening & Testing](milestones/M2-hardening-testing.md) | ≥80% coverage, 3+ integration scenarios, interface freeze | Not Started |
| **M3** | [Control Upgrades](milestones/M3-control-upgrades.md) | pure_pursuit, MPC (acados), CBF safety filter, additional kinematics | Not Started |
| **M4** | [Perception Upgrades](milestones/M4-perception-upgrades.md) | obstacle_detection, RANSAC, inflation, dynamic obstacles | Not Started |
| **M5** | [State Estimation Upgrades](milestones/M5-state-estimation-upgrades.md) | Particle filter (MCL) as EKF alternative | Not Started |
| **M6** | [Visual Perception Building Blocks](milestones/M6-visual-perception.md) | `common/camera.hpp`, `feature_extraction`, `visual_odometry`, 2.5D sim renderer | Not Started |
| **M6.5** | [SLAM](milestones/M6.5-slam.md) | EKF-SLAM + lidar SLAM + camera-based visual SLAM | Not Started |
| **M7** | [Advanced Planning](milestones/M7-advanced-planning.md) | Dijkstra, RRT*, spline fitting, TEB, time-optimal | Not Started |
| **M8** | [Multi-Robot](milestones/M8-multi-robot.md) | N-robot sim + ORCA → priority → CBS → DMPC → MADER | Not Started |
| **M9** | [Web Frontend](milestones/M9-web-frontend.md) | TypeScript/React Canvas 2D app, deployable static site | Not Started |
| **M10** | [Polish & Showcase](milestones/M10-polish-showcase.md) | Docs site, examples, packaging, demo GIFs | Not Started |
| **M11** | [Advanced Control](milestones/M11-advanced-control.md) | Adaptive PID, RLS parameter estimator, Frenet controller | Not Started |
| **M12** | [Fleet Management](milestones/M12-fleet-management.md) | VDA 5050, task allocation, fleet monitor, battery management | Not Started |
| **M13** | [Classical & Optimal Control](milestones/M13-classical-optimal-control.md) | LQR (DARE-based state feedback), Stanley path tracker | Not Started |
| **M14** | [Advanced State Estimation II](milestones/M14-advanced-state-estimation-2.md) | UKF (Merwe sigma points), pose graph optimizer (GN/LM, SE2) | Not Started |
| **M15** | [Visual-Inertial Odometry](milestones/M15-visual-inertial-odometry.md) | IMU filter + pre-integration, loosely-coupled VIO (EKF/UKF) | Not Started |
| **M16** | [Planning Upgrades II](milestones/M16-planning-upgrades-2.md) | PRM (multi-query sampling), polynomial trajectories (min-jerk/snap, Bézier) | Not Started |
| **M17** | [Camera Perception II](milestones/M17-camera-perception-2.md) | Place recognition (descriptor DB, loop closure), stereo depth (block matching) | Not Started |
| **M18** | [Advanced Nonlinear Control](milestones/M18-advanced-nonlinear-control.md) | MPPI (stochastic MPC), feedback linearization (diff-drive chained form) | Not Started |
| **M19** | [Depth Perception & 3D Understanding](milestones/M19-depth-perception-3d.md) | RGB-D camera processing, 3D DBSCAN + PCA object detection | Not Started |
| **M20** | [Planning Upgrades III](milestones/M20-planning-upgrades-3.md) | Potential field planner, Informed RRT* | Not Started |
| **M21** | [Estimation & Test Foundations](milestones/M21-estimation-test-foundations.md) | Factor graph optimizer (GN + Eigen sparse), noise models (header-only) | Not Started |
| **M22** | [Optimal Output Feedback & Automotive Perception](milestones/M22-optimal-feedback-automotive.md) | LQG (LQR + Kalman observer), lane detection (Hough + polynomial fit) | Not Started |
| **M23** | [Lattice Planning & Semantic Vision](milestones/M23-lattice-semantic.md) | Lattice planner (motion primitives + A*), semantic segmentation (stub + plugin) | Not Started |
| **M24** | [Fleet Operations](milestones/M24-fleet-operations.md) | Charging station manager (registry, priority queue, completion prediction) | Not Started |

## Dependency Graph

```
M0 (infra)
 └→ M1 (minimum viable robot + common/transforms)
      └→ M2 (hardening & testing)
           ├→ M3 (control upgrades)          → M11 (advanced control)      → M13 (classical & optimal control)
           │    └→ M13 ──────────────────────────────────────────────────────→ M18 (advanced nonlinear control)
           │    └→ M13 ──────────────────────────────────────────────────────→ M22 (optimal output feedback)
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

After M2, milestones M3–M5 and M9 can be developed **in parallel** since they touch independent domains. M6 (visual perception) requires M4 (occupancy grid + ray casting for the 2.5D camera renderer). M6.5 (SLAM) requires both M5 (state estimation interfaces) and M6 (feature extraction + visual odometry + camera types). M7 can start after M4 (needs costmaps). M8 needs M3 + M7. M11 requires M3 (stable control interfaces). M12 requires M8 (N-robot sim infrastructure). M13 requires M3 (IController interface); M11 recommended. M14 requires M5 (IStateEstimator interface). M15 requires M6 and M14 (both). M16 requires M7 (IGlobalPlanner interface) and M4 (OccupancyGrid). M17 requires M6 (feature_extraction, common/camera.hpp). M10 can begin partially after M8; full completion requires all milestones.

**M18–M24 additional dependencies:**
M18 requires M3 (IController); M13 recommended. M19 requires M6 (common/camera.hpp types) and M4 (3D detection extends obstacle_detection). M20 requires M7 (planning interfaces) and M4 (OccupancyGrid). M21 requires M14 for factor_graph (GN optimizer patterns); noise_models has no external dependencies. M22 requires M13 (LQRController) for lqg and M6+M4 for lane_detection. M23 requires M7 (IGlobalPlanner) for lattice_planner and M6 (RgbImage types) for semantic_segmentation. M24 requires M12 (fleet infrastructure). M18 and M22 can run in parallel after M13; M19 and M20 can run in parallel after M6+M4; M21 is independent of M18–M20 and M22–M23; M24 is independent of M18–M23.

## Folder Structure

```
repo-plans/
├── README.md                     ← This file (roadmap overview)
├── todos.md                      ← Quick scratchpad for ideas
├── module-task-template.md       ← Standard per-module workflow
├── milestones/                   ← One doc per milestone
│   ├── M0-dev-infrastructure.md
│   ├── M1-minimum-viable-robot.md
│   ├── ...
│   └── done/                     ← Completed milestones moved here
├── modules/                      ← Per-module task tracking
│   ├── common.md                 ← (created when work begins)
│   ├── pid.md
│   ├── astar.md
│   ├── ...
│   └── done/                     ← Completed module files moved here
└── plans/                        ← Misc planning artifacts
```

## Workflow

1. **Starting a milestone:** Read the milestone doc, create module task files from `module-task-template.md` for each module in scope. Milestone docs contain preliminary task breakdowns that get refined into module task files when work begins.
2. **Working on a module:** Check off tasks in `modules/<name>.md` as you go.
3. **Completing a module:** Move `modules/<name>.md` → `modules/done/<name>.md`.
4. **Completing a milestone:** Verify all exit criteria, move `milestones/M<N>-*.md` → `milestones/done/`.

> **Note:** Module task files for M2+ modules are created when starting that milestone, not upfront. Only M1 module files exist initially.

