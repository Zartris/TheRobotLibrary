# M9 — Full 3D Navigation Pipeline — Design Spec

**Date:** 2026-03-23
**Status:** Approved (brainstorm)
**Dependencies:** M2 (frozen interfaces, common/types_3d/, common/transforms/), M3 (all 2D kinematic models)

---

## Goal

Upgrade the core robot pipeline from 2D to full 3D navigation. A robot (ground or aerial) navigates in 3D space: estimating 6-DoF pose via quaternion EKF, planning paths through 3D voxel/ESDF/octree maps, and executing via 3D-aware controllers and kinematic models. All visible in the MuJoCo simulation with 3D map visualization and new scenarios (multi-level, quadrotor arena, outdoor terrain).

This milestone runs parallel to M4-M7 (independent domain) and establishes the 3D foundation that M15 (VIO), M19 (depth perception), and future milestones build upon.

---

## Domain Restructure: `mapping/`

### Rationale

The standard robotics pipeline is **Sense → Map → Localize → Plan → Act**. Mapping is its own domain, distinct from perception (which extracts features from raw sensor data). The current placement of `occupancy_grid` under `perception/` is incorrect.

### Changes

Create `workspace/robotics/mapping/` as a new top-level domain:

```
workspace/robotics/mapping/
  CMakeLists.txt
  occupancy_grid/     ← moved from perception/occupancy_grid/
  voxel_grid/         ← NEW (3D occupancy grid, built from scratch)
  voxel_mapping/      ← NEW (voxblox TSDF/ESDF wrapper)
  octree/             ← NEW (built from scratch)
```

### Milestone Updates Required

All milestones referencing `perception/occupancy_grid` must be updated to `mapping/occupancy_grid`:
- M1, M2, M4, M6.5, M7, and any module plan files
- Roadmap README dependency graph updated to include M9

---

## 3D Interfaces

New 3D interfaces in `common/interfaces/`, parallel to existing 2D (not replacing them):

```
common/interfaces/
  i_controller_3d.hpp       ← NEW
  i_global_planner_3d.hpp   ← NEW
  i_local_planner_3d.hpp    ← NEW
  i_state_estimator_3d.hpp  ← NEW
  i_map_3d.hpp              ← NEW
```

### Key 3D Types (existing in common/types_3d/)

| 2D (existing) | 3D (existing or new) |
|---|---|
| `Pose2D` | `Pose3D` (exists in types_3d/) |
| `Twist` (linear, angular) | `Twist3D` (exists in types_3d/) |
| `Path` (vector of Pose2D) | `Path3D` (new — vector of Pose3D) |
| `OccupancyGrid` | `IMap3D` (new — abstract interface) |

### IMap3D — Common 3D Map Interface

```cpp
class IMap3D {
public:
    virtual ~IMap3D() = default;
    virtual bool isOccupied(double x, double y, double z) const = 0;
    virtual double getDistance(double x, double y, double z) const = 0;
    virtual bool isInBounds(double x, double y, double z) const = 0;
    virtual double resolution() const = 0;
    virtual Eigen::AlignedBox3d bounds() const = 0;
};
```

All three map backends (voxel grid, ESDF, octree) implement `IMap3D`. Planners depend only on `IMap3D`.

### IKinematicModel3D

```cpp
class IKinematicModel3D {
public:
    virtual ~IKinematicModel3D() = default;
    virtual Pose3D step(const Pose3D& state, const Twist3D& control, double dt) = 0;
    virtual Twist3D getControlLimits3D() const = 0;
    virtual Twist3D toTwist3D(const Eigen::VectorXd& control) const = 0;
    virtual Eigen::VectorXd fromTwist3D(const Twist3D& twist) const = 0;
};
```

---

## Mapping Modules

### mapping/voxel_grid/ — Built from scratch

A simple 3D occupancy grid. The 3D equivalent of our 2D `OccupancyGrid`.

- `VoxelGrid : IMap3D` — flat `std::vector<int8_t>` indexed by (x,y,z)
- Log-odds updates from 3D point clouds
- `updateFromPointCloud(points, sensorOrigin)`
- `isOccupied()`, `getDistance()` (brute-force nearest-occupied)
- Theory docs: 3D discretization, log-odds sensor model in 3D, memory layout (flat vs nested), comparison with octree memory tradeoffs

### mapping/voxel_mapping/ — voxblox (third-party, FetchContent)

Thin wrapper around voxblox. Our code is the adapter layer; the real value is the theory docs.

- `TsdfMap : IMap3D` — wraps voxblox's TSDF layer, `integratePointCloud()`
- `EsdfMap : IMap3D` — wraps voxblox's ESDF layer, `getDistance()` returns the Euclidean distance field
- FetchContent in `deps.cmake`
- Theory docs (zero-to-hero):
  - TSDF theory: projective vs raycasting integration, truncation distance, weighting schemes
  - ESDF computation: quasi-Euclidean wavefront propagation
  - Marching cubes mesh extraction
  - Comparison with nvblox (GPU-accelerated production alternative)
  - When to use TSDF vs raw voxel grid vs octree

### mapping/octree/ — Built from scratch

Recursive spatial subdivision tree. Memory-efficient for sparse environments.

- `Octree : IMap3D` — insert points, query occupancy, adaptive resolution
- `OctreeNode` — 8 children, state (free/occupied/unknown), depth
- Subdivision on occupied regions, coarsening on free regions
- Theory docs (zero-to-hero):
  - Octree construction and traversal
  - Morton codes (Z-order curves) for efficient indexing
  - Pruning strategies
  - Comparison with OctoMap's probabilistic octree
  - When octree beats voxel grid (large sparse outdoor) vs when it doesn't (dense indoor)

---

## State Estimation

### state_estimation/ekf3d/ — 15-state quaternion EKF, built from scratch

State vector: `[position(3), quaternion(4), velocity(3), gyro_bias(3), accel_bias(3)]`

- **Prediction:** IMU-driven. Integrates accelerometer (gravity-compensated) for velocity/position, gyroscope for quaternion propagation via `q_dot = 0.5 * q ⊗ ω`.
- **Update sources:** Position measurement (GPS-like), full pose measurement (external tracking), odometry twist. Each is a separate `update()` overload.
- **Bias estimation:** Gyro and accelerometer biases modeled as random walks. Filter estimates and corrects for sensor drift.
- **Error-state formulation:** Error quaternion is a 3-vector (small-angle), so covariance is 15×15 (not 16×16). Quaternion re-normalized after each update.
- **Interface:** `EKF3D : IStateEstimator3D`

### Zero-to-hero docs structure:

1. Why EKF? (intuition — linearizing nonlinear systems)
2. EKF math refresher (predict/update equations, Jacobians)
3. From 2D to 3D — what changes and why
4. Rotation representations (Euler vs quaternion vs rotation matrix — tradeoffs)
5. Quaternion kinematics derivation (why `q_dot = 0.5 * q ⊗ ω`)
6. Error-state formulation (why 15 states not 16, the reset step)
7. IMU sensor model (noise, bias, Allan variance)
8. Jacobian derivation (full F and H matrices, worked through)
9. Tuning guide (Q/R matrices, what happens when you get them wrong)
10. Worked example with numbers

---

## Planning

### motion_planning/global_planning/astar3d/ — 3D A* on IMap3D

- `AStarPlanner3D : IGlobalPlanner3D` — `plan(Pose3D, Pose3D, IMap3D&) → optional<Path3D>`
- 26-connected grid search (vs 8-connected in 2D)
- 3D octile heuristic (diagonal distance in 3D)
- Collision checking via `IMap3D::isOccupied()` or `IMap3D::getDistance() > robot_radius`
- Robot radius inflation for collision safety
- Theory docs: 2D→3D generalization, 26-connectivity, 3D heuristic admissibility, memory concerns (3D search spaces explode), when to use A*3D vs sampling-based planners

### motion_planning/local_planning/dwa3d/ — 3D Dynamic Window Approach

- `DWAPlanner3D : ILocalPlanner3D` — samples `(vx, vy, vz, wx, wy, wz)` within dynamic window
- Trajectory rollout uses `IKinematicModel3D::step()` — respects actual robot dynamics
- Scoring: goal heading + obstacle clearance (via `IMap3D::getDistance()`) + velocity preference
- Ground robots: vy/vz/wx/wy constrained to near-zero, collapses to terrain-aware 2D DWA
- Quadrotors: full 6-DoF sampling
- Theory docs: dynamic window in 6-DoF, reachable velocity space, trajectory scoring in 3D, kinematic constraints shape the search

---

## Control & Kinematics

### control/pid3d/ — 3D PID Controller

- `PID3DController : IController3D` — 6 PID loops (x, y, z, roll, pitch, yaw)
- Quaternion-based orientation error (shortest-path rotation, avoids gimbal lock)
- Computes `Twist3D` from `Pose3D` error
- Theory docs: PID in 3D, quaternion error computation, decoupled vs coupled axis control

### Kinematic Models — All extended to 3D

M9 depends on M3 so all 2D kinematic models are available before building 3D versions.

| 2D Model (M1/M3) | 3D Version (M9) | Notes |
|---|---|---|
| `DifferentialDrive` | `DifferentialDrive3D` | Terrain-following, z from surface |
| `Unicycle` (M3) | `Unicycle3D` | Terrain-following |
| `AckermannModel` (M3) | `Ackermann3D` | Terrain-following, banking on slopes |
| `SwerveModel` (M3) | `Swerve3D` | Terrain-following |
| _(new)_ | `QuadrotorModel` | Native 3D, thrust + 3 torques |

All 3D ground models share `TerrainFollower` — a utility that projects any 2D kinematic model onto a 3D surface using terrain height/normal queries.

```
common/kinematics/
  i_kinematic_model.hpp         ← existing 2D (unchanged)
  i_kinematic_model_3d.hpp      ← NEW
  differential_drive.hpp        ← existing 2D (unchanged)
  differential_drive_3d.hpp     ← NEW
  quadrotor_model.hpp           ← NEW (native 3D)
  terrain_follower.hpp          ← NEW (shared ground→3D projection utility)
  unicycle_3d.hpp               ← NEW
  ackermann_3d.hpp              ← NEW
  swerve_3d.hpp                 ← NEW
```

---

## Simulation Integration

### Bridge Updates

- `StateAdapter` exposes `Pose3D` / `Twist3D` directly (no 2D projection)
- `SensorAdapter` produces 3D point clouds alongside 2D laser scans
- `ModulePipeline` extended with 3D mode using 3D interfaces
- `pipelineTick()` feeds point clouds into active `IMap3D` backend

### New MJCF Scenarios

| Scenario | Purpose |
|---|---|
| `multi_level.xml` | Ramps connecting two levels — tests 3D planning + terrain following |
| `quadrotor_arena.xml` | Open 3D space with floating obstacles — tests full 6-DoF navigation |
| `outdoor_terrain.xml` | Uneven ground with slopes — tests ground robot 3D kinematics |

### ImGui Panels

- 3D map visualization selector (voxel grid / ESDF heatmap / octree wireframe)
- 3D path rendering in MuJoCo scene
- EKF3D covariance ellipsoid (3D version of 2D ellipse)
- Kinematic model selector includes quadrotor + all 3D ground models

### Integration Tests

- Quadrotor navigates through floating obstacles to goal (full 3D)
- Ground robot navigates up a ramp to upper level (terrain following)
- Map backend swap mid-run (voxel grid → ESDF → octree) doesn't crash
- Module swap regression (same as M2 pattern)

---

## Documentation Philosophy

Every module gets a `docs/theory.md` that takes a reader from zero knowledge to full understanding. Not API docs — these teach the underlying math and algorithms comprehensively.

For particularly complex modules (EKF3D, TSDF/voxblox, octree), docs should cover:
- Intuition and motivation
- Mathematical foundations (derivations, not just results)
- Algorithm step-by-step with worked examples
- Implementation decisions and tradeoffs
- Comparison with alternatives
- References to seminal papers

voxblox wrapper docs additionally cover nvblox as the GPU-accelerated production alternative.

---

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

---

## NOT IN

- GPU acceleration (nvblox mentioned in docs only)
- Full rotor dynamics / aerodynamics (simplified quadrotor model)
- Multi-robot 3D
- SLAM 3D (→ M6.5 + future)
- Visual perception 3D (→ M19)
- Dense mesh reconstruction (marching cubes documented but not implemented in sim)
- Real sensor drivers / hardware interface
