# M9 — Full 3D Navigation Pipeline

**Status:** Not Started
**Dependencies:** M2 (frozen interfaces), M3 (all 2D kinematic models)
**Scope:** Upgrade the core pipeline from 2D to full 3D: 3D mapping (voxel grid, TSDF/ESDF, octree), 15-state quaternion EKF, 3D A* and DWA, 3D PID, all kinematics extended to 3D, quadrotor model. New `mapping/` domain.

---

## Goal

Upgrade the core robot pipeline from 2D to full 3D navigation. A robot (ground or aerial) navigates in 3D space: estimating 6-DoF pose via quaternion EKF, planning paths through 3D voxel/ESDF/octree maps, and executing via 3D-aware controllers and kinematic models. All visible in the MuJoCo simulation with 3D map visualization and new scenarios (multi-level, quadrotor arena, outdoor terrain).

This milestone runs parallel to M4-M7 (independent domain) and establishes the 3D foundation that M15 (VIO), M19 (depth perception), and future milestones build upon.

---

## Sub-Phases

M9 is split into ordered sub-phases, each independently testable:

### M9-A: 3D Foundation
- Domain restructure (create `mapping/`, move `occupancy_grid`)
- 3D types validation (confirm `types_3d/` completeness)
- All 3D interfaces (`IMap3D`, `IKinematicModel3D`, `IStateEstimator3D`, `IController3D`, `IGlobalPlanner3D`, `ILocalPlanner3D`)
- `Path3D`, `PerceptionContext3D`, `TrackedObstacle3D` types
- Exit: all interfaces compile

### M9-B: 3D Mapping
- `mapping/voxel_grid/` (3D occupancy grid + EDT)
- `mapping/octree/` (built from scratch)
- `mapping/voxel_mapping/` (voxblox wrapper — validate FetchContent first)
- All implement `IMap3D`
- Exit: all three map backends pass unit tests, interchangeable via `IMap3D`

### M9-C: 3D State Estimation
- `state_estimation/ekf3d/` (15-state quaternion EKF)
- Zero-to-hero EKF theory docs
- Exit: EKF3D converges with simulated IMU + position updates

### M9-D: 3D Planning & Control
- `motion_planning/global_planning/astar3d/`
- `motion_planning/local_planning/dwa3d/`
- `control/pid3d/`
- Exit: quadrotor plans and follows 3D path in unit tests

### M9-E: 3D Kinematics
- `IKinematicModel3D` interface
- `QuadrotorModel` (native 3D)
- `TerrainFollower` utility
- `DifferentialDrive3D`, `Unicycle3D`, `Ackermann3D`, `Swerve3D`
- Exit: all kinematic models pass unit tests

### M9-F: Simulation Integration
- Bridge 3D mode (`Pose3D`/`Twist3D`/point clouds)
- 3 MJCF scenarios
- ImGui 3D panels
- Integration tests (quadrotor 3D nav, ground robot ramp, map swap, module swap)
- Exit: full 3D pipeline runs end-to-end in sim

---

## Domain Restructure: Create `mapping/`

### Rationale

The standard robotics pipeline is **Sense → Map → Localize → Plan → Act**. Mapping is its own domain, distinct from perception (which extracts features from raw sensor data). The current placement of `occupancy_grid` under `perception/` is incorrect.

### Changes

Create `workspace/robotics/mapping/` as a new top-level domain:

```
workspace/robotics/mapping/
  CMakeLists.txt
  occupancy_grid/     <- moved from perception/occupancy_grid/
  voxel_grid/         <- NEW (3D occupancy grid, built from scratch)
  voxel_mapping/      <- NEW (voxblox TSDF/ESDF wrapper)
  octree/             <- NEW (built from scratch)
```

### Coordination Note

The `occupancy_grid` move from `perception/` to `mapping/` must be completed in M9-A before any parallel milestone (M4, M7) modifies `occupancy_grid`. If M4 is in progress, coordinate the move to avoid merge conflicts. Alternatively, M9-A can be landed as a standalone PR before other M9 sub-phases begin.

---

## Modules

### M9-A: 3D Foundation

#### 3D Interfaces (common/interfaces/)

- [ ] `include/common/interfaces/i_map_3d.hpp` — `IMap3D` (isOccupied, getDistance, isInBounds, resolution, bounds, updateFromPointCloud, raycast)
- [ ] `include/common/interfaces/i_state_estimator_3d.hpp` — `IStateEstimator3D` (predict from IMU, updatePosition, updatePose, getPose3D, getCovariance, reset)
- [ ] `include/common/interfaces/i_controller_3d.hpp` — `IController3D` (compute Twist3D from Pose3D error)
- [ ] `include/common/interfaces/i_global_planner_3d.hpp` — `IGlobalPlanner3D` (plan on IMap3D)
- [ ] `include/common/interfaces/i_local_planner_3d.hpp` — `ILocalPlanner3D` + `PerceptionContext3D` + `TrackedObstacle3D`
- [ ] `include/common/interfaces/i_kinematic_model_3d.hpp` — `IKinematicModel3D` + `ControlLimits3D`
- [ ] `include/common/types_3d/pose3d.hpp` — add `Path3D` typedef
- [ ] Domain restructure: create `workspace/robotics/mapping/`, move `occupancy_grid` from `perception/`
- [ ] Update `workspace/architecture.md` — add `mapping` domain

### M9-B: 3D Mapping

#### mapping/voxel_grid — 3D occupancy grid (built from scratch)

- [ ] `include/voxel_grid/voxel_grid.hpp` — `VoxelGrid : IMap3D`
- [ ] `src/voxel_grid.cpp` — flat vector storage, log-odds updates, EDT for getDistance()
- [ ] `tests/test_voxel_grid.cpp`:
  - Empty grid → all unknown
  - Point cloud update → cells along rays free, endpoints occupied
  - getDistance() returns correct EDT distance
  - Coordinate conversion round-trip
  - raycast into occupied cell → correct range
- [ ] `docs/theory.md` — 3D discretization, log-odds, EDT, memory layout, comparison with octree

#### mapping/voxel_mapping — TSDF/ESDF via voxblox (third-party)

- [ ] Validate voxblox FetchContent integration (or implement fallback)
- [ ] `include/voxel_mapping/tsdf_map.hpp` — `TsdfMap : IMap3D`
- [ ] `include/voxel_mapping/esdf_map.hpp` — `EsdfMap : IMap3D`
- [ ] `src/tsdf_map.cpp`, `src/esdf_map.cpp` — voxblox adapter layer
- [ ] `tests/test_tsdf_map.cpp`:
  - Integrate point cloud → TSDF values near surface
  - Query distance → correct ESDF value
  - IMap3D interface works interchangeably with VoxelGrid
- [ ] `docs/theory.md` — TSDF theory (projective vs raycasting, truncation, weighting), ESDF wavefront propagation, marching cubes, nvblox comparison

#### mapping/octree — spatial octree (built from scratch)

- [ ] `include/octree/octree.hpp` — `Octree : IMap3D`, `OctreeNode`
- [ ] `src/octree.cpp` — recursive subdivision, adaptive resolution, pruning
- [ ] `tests/test_octree.cpp`:
  - Insert points → correct occupancy at queried positions
  - Adaptive resolution: dense region → deeper nodes
  - Pruning: uniform region → coarsened
  - Memory usage < equivalent voxel grid for sparse environment
  - IMap3D interface works interchangeably
- [ ] `docs/theory.md` — construction, Morton codes, pruning, OctoMap comparison, when octree vs voxel grid

### M9-C: 3D State Estimation

#### state_estimation/ekf3d — 15-state quaternion EKF (built from scratch)

- [ ] `include/ekf3d/ekf3d.hpp` — `EKF3D : IStateEstimator3D`
- [ ] `src/ekf3d.cpp` — error-state EKF, quaternion kinematics, 15×15 covariance
- [ ] `tests/test_ekf3d.cpp`:
  - Predict with zero IMU → pose unchanged, covariance grows
  - Predict with known acceleration for 1s → expected position
  - Quaternion propagation: constant angular velocity → expected rotation
  - Update with perfect position → covariance shrinks
  - Bias estimation: constant bias converges after N updates
  - Predict→update cycle converges to true 3D pose with noisy inputs
  - Error-state reset: quaternion remains normalized
- [ ] `docs/theory.md` — zero-to-hero: EKF refresher, 2D→3D, rotation representations, quaternion kinematics, error-state formulation, IMU model, Jacobians, tuning guide, worked example

### M9-D: 3D Planning & Control

#### motion_planning/global_planning/astar3d — 3D A*

- [ ] `include/astar3d/astar_planner_3d.hpp` — `AStarPlanner3D : IGlobalPlanner3D`
- [ ] `src/astar_planner_3d.cpp` — 26-connected, 3D octile heuristic, IMap3D collision check
- [ ] `tests/test_astar_planner_3d.cpp`:
  - Straight line in empty 3D space → direct path
  - Obstacle wall → path goes around/over
  - No path possible → nullopt
  - Start == goal → single point path
  - Works with VoxelGrid, EsdfMap, and Octree via IMap3D
- [ ] `docs/theory.md` — 2D→3D generalization, 26-connectivity, heuristics, memory

#### motion_planning/local_planning/dwa3d — 3D DWA

- [ ] `include/dwa3d/dwa_planner_3d.hpp` — `DWAPlanner3D : ILocalPlanner3D`
- [ ] `src/dwa_planner_3d.cpp` — 6-DoF velocity sampling, IKinematicModel3D rollout, IMap3D scoring
- [ ] `tests/test_dwa_planner_3d.cpp`:
  - Open space with goal ahead → forward velocity
  - Obstacle ahead → avoidance velocity
  - Quadrotor: full 6-DoF sampling produces vertical avoidance
  - Ground robot: constrained to near-2D motion
  - Respects dynamic window (acceleration limits from ControlLimits3D)
- [ ] `docs/theory.md` — dynamic window 6-DoF, reachable velocity space, scoring, kinematic constraints

#### control/pid3d — 3D PID controller

- [ ] `include/pid3d/pid3d_controller.hpp` — `PID3DController : IController3D`
- [ ] `src/pid3d_controller.cpp` — 6 PID loops, quaternion orientation error
- [ ] `tests/test_pid3d_controller.cpp`:
  - Position error → proportional Twist3D
  - Orientation error → correct angular velocity (shortest path)
  - Anti-windup: integral doesn't explode
  - Reset clears all 6 axes
- [ ] `docs/theory.md` — PID in 3D, quaternion error, decoupled axis control

### M9-E: 3D Kinematics

#### common/kinematics/ — 3D kinematic models

- [ ] `include/common/kinematics/i_kinematic_model_3d.hpp` — `IKinematicModel3D`, `ControlLimits3D`
- [ ] `include/common/kinematics/terrain_follower.hpp` — `TerrainFollower` (projects 2D model onto 3D surface)
- [ ] `include/common/kinematics/quadrotor_model.hpp` — `QuadrotorModel : IKinematicModel3D`
- [ ] `include/common/kinematics/differential_drive_3d.hpp` — `DifferentialDrive3D : IKinematicModel3D`
- [ ] `include/common/kinematics/unicycle_3d.hpp` — `Unicycle3D : IKinematicModel3D`
- [ ] `include/common/kinematics/ackermann_3d.hpp` — `Ackermann3D : IKinematicModel3D`
- [ ] `include/common/kinematics/swerve_3d.hpp` — `Swerve3D : IKinematicModel3D`
- [ ] `src/kinematics/terrain_follower.cpp`, `quadrotor_model.cpp`, `differential_drive_3d.cpp`, `unicycle_3d.cpp`, `ackermann_3d.cpp`, `swerve_3d.cpp`
- [ ] `tests/test_quadrotor_model.cpp` — hover, ascend, forward flight, thrust/torque limits
- [ ] `tests/test_terrain_follower.cpp` — flat ground matches 2D, slope → correct z/pitch, step edge handling
- [ ] `tests/test_kinematic_models_3d.cpp` — all ground models: flat→same as 2D, slope→correct terrain following, twist round-trip

### M9-F: Simulation Integration

- [ ] `StateAdapter` exposes `Pose3D` / `Twist3D` directly
- [ ] `SensorAdapter` produces 3D point clouds
- [ ] `ModulePipeline` extended with 3D mode
- [ ] `scenarios/multi_level.xml` — ramps connecting two levels
- [ ] `scenarios/quadrotor_arena.xml` — floating obstacles
- [ ] `scenarios/outdoor_terrain.xml` — uneven ground
- [ ] ImGui: 3D map visualization selector
- [ ] ImGui: 3D path rendering, EKF3D covariance ellipsoid
- [ ] ImGui: kinematic model selector (quadrotor + ground 3D models)
- [ ] `tests/test_integration_3d.cpp`:
  - Quadrotor navigates 3D obstacle course → reaches goal
  - Ground robot navigates up ramp → reaches upper level
  - Map backend swap mid-run → no crash
  - Module swap regression → no crash

---

## Deliverables

- [ ] New `mapping/` domain with `occupancy_grid` (moved), `voxel_grid`, `voxel_mapping`, `octree`
- [ ] All 3D interfaces (`IMap3D`, `IStateEstimator3D`, `IController3D`, `IGlobalPlanner3D`, `ILocalPlanner3D`, `IKinematicModel3D`)
- [ ] EKF3D (15-state quaternion, error-state formulation)
- [ ] AStarPlanner3D (26-connected, 3D heuristic)
- [ ] DWAPlanner3D (6-DoF velocity sampling)
- [ ] PID3DController (quaternion orientation error)
- [ ] QuadrotorModel + TerrainFollower + all ground 3D kinematic models
- [ ] 3 MJCF scenarios + simulation integration + integration tests
- [ ] Zero-to-hero theory docs for every module
- [ ] All modules pass Phase 4.5 observability gate

## Exit Criteria

1. Quadrotor autonomously navigates 3D obstacle course (6-DoF)
2. Ground robot navigates up ramp to upper level (terrain following)
3. All three map backends work interchangeably via IMap3D
4. EKF3D converges with simulated IMU + position updates
5. All 2D kinematic models have working 3D counterparts
6. Map backend swap mid-run doesn't crash
7. All modules pass Phase 4.5 observability gate
8. Zero-to-hero theory docs for every module
9. All unit + integration tests pass, CI green

## NOT IN

GPU acceleration (nvblox in docs only), full rotor dynamics / aerodynamics, multi-robot 3D, SLAM 3D (→ M6.5+), visual perception 3D (→ M19), dense mesh reconstruction (documented only), real sensor drivers.
