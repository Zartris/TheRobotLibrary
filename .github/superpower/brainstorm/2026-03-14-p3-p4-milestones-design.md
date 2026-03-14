# Design: P3 & P4 Priority Milestones (M13–M17)

**Date:** 2026-03-14  
**Status:** Approved  
**Topics:** New milestones M13–M17 covering unresolved P2 modules (LQR, UKF, IMU processing) plus all P3 and P4 modules from the gap analysis.

---

## Summary

Five new milestones (M13–M17), each with exactly two modules, covering ten modules total:

| Milestone | Name | Modules |
|-----------|------|---------|
| M13 | Classical & Optimal Control | `control/lqr` · `control/stanley` |
| M14 | Advanced State Estimation II | `state_estimation/ukf` · `state_estimation/pose_graph` |
| M15 | Visual-Inertial Odometry | `perception/imu_processing` · `state_estimation/visual_inertial_odometry` |
| M16 | Planning Upgrades II | `motion_planning/global_planning/prm` · `motion_planning/trajectory_planning/polynomial` |
| M17 | Camera Perception II | `perception/place_recognition` · `perception/stereo_depth` |

---

## Design Questions — Answers & Rationale

### Q1: Milestone grouping — P2 + P3 as one or two milestones?

**Decision:** Five milestones (M13–M17), each with two modules grouped by domain cohesion and dependency chains, not by priority label.

Mixing P2 unresolved items *into* the milestones they enable (rather than isolating them in a single "P2 cleanup" milestone) keeps each milestone independently teachable. Each milestone trains one concept cluster:

- M13 = optimal/geometric control
- M14 = nonlinear estimation + loop closure backend
- M15 = visual-inertial fusion (imu_processing is M15 entry point, VIO is its culmination)
- M16 = probabilistic sampling and smooth trajectory generation
- M17 = passive camera depth + appearance-based place memory

Two-module milestones mirror the project philosophy: *simplest correct implementation first*, independently testable, reviewable in a focused session.

---

### Q2: Dependency ordering relative to M12

All five new milestones are independent of M12 (fleet management) — they extend the single-robot capability stack. They can be developed in parallel with M12 or after it.

**VIO chain:** `ukf (M14) → imu_processing + visual_inertial_odometry (M15)`  
M15 must follow M14 because UKF is the preferred estimator for the VIO EKF/UKF state propagation. VIO also requires M6 (`visual_odometry`, `common/camera.hpp`) which is an earlier milestone.

**Pose graph timing:** `pose_graph (M14)` becomes available before M6.5 SLAM is complete in most development schedules. M6.5 ships with its own simplified inline chain for loop closure; once M14 is done, both `lidar_slam` and `visual_slam` can optionally swap their loop closure backend to use the reusable `pose_graph`. No hard dependency blocks M6.5.

**Place recognition → pose graph feedback:** `place_recognition (M17)` produces loop closure detections (candidate frame pairs). These can be fed as edges into `pose_graph (M14)` for optimization — creating an optional post-M17 upgrade path for both SLAM variants.

---

### Q3: LQR / Stanley grouping

**Decision:** Together in M13.

LQR (P2) was noted in todos.md as "Add to M3 or create M3.5." Stanley (P4) was noted as a natural companion to `pure_pursuit` (M3). Combining them avoids:

- Retrofitting M3 (avoid scope creep in a defined milestone)
- Creating two single-module mini-milestones (M3.5 + some P4 milestone)

Together they form a clean "classical & optimal control" chapter: LQR teaches cost-function design and DARE solving; Stanley teaches geometric path tracking and CTE normalization. Both are independently implementable within M13.

LQR does **not** extend M11 (Advanced Control). M11 has a clear adaptive-control focus (adaptive PID, RLS, Frenet). LQR's Q/R weighting intuition is closer to MPC (M3) than to online adaptation — M13 sits downstream of M3 with M11 as a soft recommendation.

---

### Q4: PRM vs extending M7

**Decision:** New milestone M16, not an M7 extension.

M7 is self-contained with five modules (dijkstra, rrt, spline_fitting, teb, time_optimal). Extending it retroactively would add scope to a well-bounded milestone and reduce its teachability as a standalone unit.

M16 naturally extends M7 by adding:
- PRM as the multi-query companion to RRT* (single-query)
- Polynomial trajectory generation as the higher-order sibling to `spline_fitting`

M16 depends on M7 for the `IGlobalPlanner` interface and M4's `OccupancyGrid` for collision checking.

---

### Q5: Stereo depth + place recognition

**Decision:** Together in M17 "Camera Perception II."

Both modules are camera-based perception that build on M6 infrastructure:
- `stereo_depth` uses `common/camera.hpp` types (`CameraIntrinsics`, `StereoCalibration`)
- `place_recognition` consumes feature descriptors from `feature_extraction` (M6)

They are independent of each other within M17 (no cross-dependency), so both can be implemented and tested in parallel. Their M6 dependency ties them to the same entry prerequisite, making M17 a clean "camera perception round 2" chapter.

---

## Proposed Milestone Structure

---

### M13 — Classical & Optimal Control

**Status:** Not Started  
**Dependencies:** M3 (stable `IController` interface + kinematic models). M11 recommended but not required.  
**Scope:** Fills the gap between geometric (pure_pursuit) and model-based (MPC) control with two classical algorithms: LQR state-feedback and Stanley cross-track path following.

#### Modules

##### `control/lqr`

Discrete-time infinite-horizon LQR. Solves the DARE (Discrete Algebraic Riccati Equation) using Eigen's `GeneralizedSelfAdjointEigenSolver` or iterative solver. Teaches cost-function design and optimal state feedback — the conceptual sibling to MPC.

- [ ] `include/lqr/lqr_controller.hpp` — `LQRController : IController`
- [ ] `include/lqr/lqr_config.hpp` — `LQRConfig` (Q, R, A, B matrices; discretization dt)
- [ ] `src/lqr_controller.cpp` — DARE solver + `u = -Kx` feedback
- [ ] Support linearised kinematic model at operating point (diff-drive linearization helper)
- [ ] `tests/test_lqr_controller.cpp`:
  - Regulator drives state to zero from perturbation within N steps
  - Higher Q (R fixed) → faster convergence; higher R (Q fixed) → smaller control inputs
  - Provided A/B matrices for double integrator → analytic K matches theoretical solution
  - Stability: eigenvalues of `(A - BK)` lie strictly inside unit circle
- [ ] Phase 4.5: `ILogger`, state transitions at `DEBUG`, DARE solve time at `TRACE`
- [ ] Sim: selectable via `PUT /api/robot/controller {"type":"lqr"}`
- [ ] Frontend: render Q/R gain panel + current control effort

##### `control/stanley`

Stanley controller from Stanford's DARPA challenge. Combines heading error and speed-normalized cross-track error into a single steering command. Ackermann-friendly but also applicable to differential-drive via Ackermann approximation.

- [ ] `include/stanley/stanley_controller.hpp` — `StanleyController : IController`
- [ ] `include/stanley/stanley_config.hpp` — `StanleyConfig` (gain k, velocity softening ε, max steering angle)
- [ ] `src/stanley_controller.cpp` — `δ = ψ_e + arctan(k·e_cte / (v + ε))`
- [ ] `tests/test_stanley_controller.cpp`:
  - Straight path: CTE decays exponentially from initial offset (verify gain k effect)
  - Curved path: tracks within tolerance on circular arc
  - Heading-only error (CTE = 0): pure heading correction, no CTE term
  - Zero velocity: softening ε prevents division by zero; output bounded
  - Reversing: heading error flips sign correctly
- [ ] Phase 4.5: `ILogger`, CTE + heading error logged at `DEBUG`, per-step σ at `TRACE`
- [ ] Sim: selectable via `PUT /api/robot/controller {"type":"stanley"}`
- [ ] Frontend: cross-track error overlay (same style as `frenet` from M11)

#### Exit Criteria

1. LQR drives linearized double-integrator to zero; eigenvalues of `(A-BK)` inside unit circle
2. LQR Q/R weighting behaviour verified quantitatively in tests
3. Stanley tracks straight and curved paths within CTE tolerance
4. Both controllers hot-swappable via REST mid-run without crash
5. All unit tests pass, CI green
6. All modules pass Phase 4.5 — Observability gate (state transitions at `DEBUG`, metrics at `TRACE`)

#### NOT IN

Full nonlinear DARE iteration (use existing Eigen facility), LQG (P8) observer, feedback linearization (P6), hardware Ackermann kinematics.

---

### M14 — Advanced State Estimation II

**Status:** Not Started  
**Dependencies:** M2 (hardening; stable common types), M5 (state estimation interfaces and `IStateEstimator`).  
**Scope:** UKF as a drop-in nonlinear upgrade from EKF, plus a reusable Gauss-Newton / LM pose graph optimizer as the SLAM loop closure backend. No cross-dependency between the two modules within M14.

#### Modules

##### `state_estimation/ukf`

Unscented Kalman Filter with Merwe scaled sigma points. Accepts process and measurement functions as `std::function` — same generic style as EKF. Natural upgrade path for strongly nonlinear systems (SE3 poses, IMU integration).

- [ ] `include/ukf/ukf.hpp` — `UKF : IStateEstimator`, templated on state dimension
- [ ] `include/ukf/ukf_config.hpp` — `UKFConfig` (α, β, κ sigma-point parameters, Q, R)
- [ ] `src/ukf.cpp` — sigma-point propagation, unscented transform for predict + update
- [ ] Generic process function: `std::function<VectorXd(const VectorXd&, const VectorXd&)>`
- [ ] Generic measurement function: `std::function<VectorXd(const VectorXd&)>`
- [ ] `tests/test_ukf.cpp`:
  - Linear system: UKF matches KF solution exactly (unscented transform is exact for linear)
  - Nonlinear pendulum: UKF converges faster than EKF from same initial condition
  - Sigma-point count: always `2n+1` points; weights sum to 1
  - Degenerate covariance: `P` stays positive semi-definite throughout
- [ ] Phase 4.5: `ILogger`, sigma-point spread and residual norm at `DEBUG`, propagation time at `TRACE`
- [ ] Sim: registerable as alternative state estimator alongside EKF
- [ ] Frontend: render state covariance ellipse (same panel as EKF)

##### `state_estimation/pose_graph`

Lightweight Gauss-Newton / Levenberg-Marquardt pose graph optimizer. SE2 nodes (robot poses) + relative-transform edges (with covariance). Provides the reusable loop closure backend that both `lidar_slam` and `visual_slam` (M6.5) can optionally adopt after M14 completion.

- [ ] `include/pose_graph/pose_graph.hpp` — `PoseGraph`, `PoseNode` (SE2), `PoseEdge` (relative transform + Ω = Σ⁻¹)
- [ ] `include/pose_graph/pose_graph_optimizer.hpp` — `PoseGraphOptimizer` (GN / LM)
- [ ] `src/pose_graph.cpp` + `src/pose_graph_optimizer.cpp`
- [ ] APIs: `addNode()`, `addEdge()`, `optimize(maxIter)` → `OptimizationResult`
- [ ] Fix gauge freedom: anchor node 0 during optimization
- [ ] `tests/test_pose_graph.cpp`:
  - 4-node square loop (known relative transforms + one loop closure edge) → optimize → node positions within tolerance of ground truth
  - Chain of 10 nodes with no loop closure → optimizer converges without drift (noise = 0)
  - Adding loop closure edge → trajectory correction > 20% improvement in cumulative error
  - Large graph (100 nodes, 5 loop closures) → solver finishes in < 500 ms (Eigen SparseLU)
  - Jacobian numerical check: analytic Jacobian matches finite-difference approximation
- [ ] Phase 4.5: `ILogger`, iteration count + residual norm at `DEBUG`, solve time per iteration at `TRACE`

#### Exit Criteria

1. UKF matches KF on linear system, outperforms EKF on nonlinear benchmark (measured convergence steps)
2. Pose graph 4-node square loop converges to < 0.05 m position error
3. Adding loop closure edge → cumulative trajectory drift reduced measurably
4. 100-node graph optimization completes in < 500 ms
5. All unit tests pass, CI green
6. All modules pass Phase 4.5 — Observability gate

#### NOT IN

SE3 pose graph (→ future M18+), g2o / GTSAM wrappers, IMU factor nodes, landmark nodes (factor_graph P5 covers the general case).

---

### M15 — Visual-Inertial Odometry

**Status:** Not Started  
**Dependencies:** M6 (`visual_odometry`, `common/camera.hpp`), M14 (UKF preferred for VIO state propagation). M5 (IStateEstimator) as structural prerequisite.  
**Scope:** Complete "visual-inertial chapter." IMU processing (filter + pre-integrator) is the direct prerequisite of loosely-coupled VIO. Both modules are developed in sequence within M15.

#### Within-Milestone Dependency

`perception/imu_processing` → `state_estimation/visual_inertial_odometry`

#### Modules

##### `perception/imu_processing`

Complementary filter for orientation, bias estimation via exponential moving average, and IMU pre-integration between keyframes. All types are self-contained in `ImuTypes`.

- [ ] `include/imu_processing/imu_types.hpp` — `ImuMeasurement` (accel, gyro, dt), `ImuBias` (accel_bias, gyro_bias), `ImuState` (orientation, velocity, position)
- [ ] `include/imu_processing/imu_filter.hpp` — `ImuFilter` (complementary filter, bias estimation)
- [ ] `include/imu_processing/imu_preintegrator.hpp` — `ImuPreintegrator` (accumulates ΔR, Δv, Δp between keyframes, Jacobians w.r.t. initial bias)
- [ ] `src/imu_filter.cpp` + `src/imu_preintegrator.cpp`
- [ ] `tests/test_imu_processing.cpp`:
  - Static IMU (zero motion): complementary filter → orientation converges; bias estimate converges within 50 steps
  - Known constant rotation: gyro integration matches analytical result
  - Pre-integration over N steps: `ΔR, Δv, Δp` match numerical integration of same sequence (< 1e-6 error)
  - Bias Jacobian: analytic matches finite-difference for `∂Δp/∂b_a`
- [ ] Phase 4.5: `ILogger`, bias estimates at `DEBUG`, integration step time at `TRACE`

##### `state_estimation/visual_inertial_odometry`

Loosely-coupled VIO: EKF/UKF fusing IMU pre-integrated motion priors + VO pose updates. Accepts `ImuPreintegrationResult` (from `imu_processing`) and `OdometryResult` (from `visual_odometry`) as inputs — does not link against those modules directly; inputs are provided by the caller through `common/` types.

- [ ] `include/visual_inertial_odometry/visual_inertial_odometry.hpp` — `VisualInertialOdometry : IStateEstimator`
- [ ] `include/visual_inertial_odometry/vio_state.hpp` — `VIOState` (SE3 pose, velocity, `ImuBias`)
- [ ] `include/visual_inertial_odometry/vio_config.hpp` — `VIOConfig` (noise params, update frequency)
- [ ] `src/visual_inertial_odometry.cpp` — predict via IMU pre-integration, update via VO relative pose
- [ ] Loosely-coupled architecture: VO updates at keyframe rate; IMU propagates between frames
- [ ] `tests/test_visual_inertial_odometry.cpp`:
  - IMU-only propagation (no VO updates): drift matches hand-calculated dead-reckoning bound
  - VO update corrects IMU drift: trajectory error decreases vs IMU-only baseline
  - Simulated IMU + camera sequence (hand-crafted): end-to-end drift < 1% of path length
  - Bias observability: after N VO updates, bias estimate converges toward true injected bias
- [ ] Phase 4.5: `ILogger`, predict/update cycle at `DEBUG`, cycle time + residual at `TRACE`
- [ ] Sim: SLAM pipeline slot — `ImuPreintegrator` + `VisualOdometry` + `VisualInertialOdometry`
- [ ] Frontend: VIO trajectory trace alongside raw VO trace; bias drift panel

#### Exit Criteria

1. Static IMU: orientation converges; bias estimate stabilizes
2. Pre-integration error < 1e-6 vs numerical integration reference
3. VIO trajectory drift < pure VO drift on same simulated sequence
4. `VIOState` (pose + velocity + bias) outputs valid, non-diverging data
5. All unit tests pass, CI green
6. All modules pass Phase 4.5 — Observability gate

#### NOT IN

Tightly-coupled VIO (feature reprojection into estimator state), marginalization, IMU initialization from still stand (→ future), online extrinsic calibration.

---

### M16 — Planning Upgrades II

**Status:** Not Started  
**Dependencies:** M7 (`IGlobalPlanner` interface, planning infrastructure), M4 (`OccupancyGrid` for collision checking). M2 (stable common types).  
**Scope:** Probabilistic Roadmap planner (multi-query, sampling-based) plus minimum-jerk / minimum-snap polynomial trajectory generation with Bézier curve variant.

#### Modules

##### `motion_planning/global_planning/prm`

Multi-query PRM. Roadmap is built once (offline or at startup) and reused across many planning queries. Local planner: straight-line collision check against `OccupancyGrid`. Query phase: Dijkstra / A* on the roadmap graph.

- [ ] `include/prm/prm_planner.hpp` — `PRMPlanner : IGlobalPlanner`
- [ ] `include/prm/prm_config.hpp` — `PRMConfig` (max_nodes, k_nearest, max_edge_length, num_samples)
- [ ] `src/prm_planner.cpp` — random sampling in C-space, k-nearest graph construction, Dijkstra query
- [ ] `tests/test_prm_planner.cpp`:
  - Obstacle-free: after N samples, roadmap is connected; query finds short path
  - Narrow corridor: path found with success rate ≥ 90% over 100 runs (probabilistic test)
  - Multi-query: roadmap built once, two distinct queries answered correctly; roadmap not rebuilt
  - No path: all samples blocked → query returns `std::nullopt`; no crash
  - `k_nearest` parameter: larger k → denser roadmap → shorter query paths (statistical test)
- [ ] Phase 4.5: `ILogger`, roadmap build stats (nodes, edges) at `DEBUG`, query solve time at `TRACE`
- [ ] Sim: selectable via `PUT /api/robot/global_planner {"type":"prm"}`
- [ ] Frontend: render roadmap graph (grey edges) + planned path (highlighted)

##### `motion_planning/trajectory_planning/polynomial`

Minimum-jerk (1D: minimize ∫jerk²) and minimum-snap (minimize ∫snap²) polynomial segments. Waypoint-constrained generation via QP on polynomial coefficients. Bézier curve variant using de Casteljau algorithm with convex hull safety guarantee.

- [ ] `include/polynomial/polynomial_trajectory.hpp` — `PolynomialTrajectory`, `PolynomialSegment`, `BezierCurve`
- [ ] `include/polynomial/polynomial_config.hpp` — `PolynomialConfig` (order, derivative order to minimize, segment durations)
- [ ] `src/polynomial_trajectory.cpp` — coefficient QP (Eigen `LDLT`), `BezierCurve` via de Casteljau
- [ ] API: `generate(waypoints, durations)` → `PolynomialTrajectory`; `sample(t)` → position/velocity/acceleration
- [ ] `tests/test_polynomial_trajectory.cpp`:
  - 3-waypoint minimum-jerk: endpoint positions match; endpoint derivatives (vel, accel) match specified boundary conditions
  - Minimum-snap: 4th derivative minimized (verify via numerical integration of snap vs. spline baseline)
  - Bézier curve: all points lie strictly within convex hull of control points
  - `sample(t)` continuity: position/velocity continuous at segment junctions; acceleration C0 for min-jerk
  - Single segment: analytic polynomial coefficients match closed-form solution
- [ ] Phase 4.5: `ILogger`, QP solve time + condition number at `DEBUG`, sample computation time at `TRACE`
- [ ] Sim: trajectory planner slot — replaces `velocity_profiling` for drone/arm scenarios
- [ ] Frontend: render polynomial trajectory with curvature color coding

#### Exit Criteria

1. PRM finds path in cluttered environment within configured sample budget
2. PRM roadmap multi-query verified: second query reuses graph without rebuild
3. Polynomial trajectory through 3+ waypoints satisfies endpoint derivatives
4. Bézier curve satisfies convex hull property (verified by test)
5. All unit tests pass, CI green
6. All modules pass Phase 4.5 — Observability gate

#### NOT IN

Informed PRM (→ future), 3D C-space, minimum-time polynomial (→ M7 `time_optimal`), NURBS (→ M7 `spline_fitting`), QP solver library (use Eigen direct solver for small systems only).

---

### M17 — Camera Perception II

**Status:** Not Started  
**Dependencies:** M6 (`feature_extraction` descriptors, `common/camera.hpp` types). No dependency on M14 or M15 (though `place_recognition` output can be fed into `pose_graph` from M14 post-completion).  
**Scope:** Descriptor-based loop closure detection (works with both lidar and visual descriptors) plus stereo camera disparity and depth estimation. Both modules are independent within M17.

#### Modules

##### `perception/place_recognition`

Descriptor-based place database. Supports both lidar scan descriptors (e.g., a scan-context-style histogram) and visual feature descriptor aggregates. Query returns candidate frame IDs with similarity scores above a threshold.

- [ ] `include/place_recognition/place_recognizer.hpp` — `PlaceRecognizer`
- [ ] `include/place_recognition/place_database.hpp` — `PlaceDatabase` (indexed by frame ID)
- [ ] `include/place_recognition/place_types.hpp` — `PlaceDescriptor` (generic `std::vector<float>`), `PlaceCandidate` (frame_id + score)
- [ ] `src/place_recognizer.cpp` — L2 / cosine similarity search; nearest-neighbour with score threshold
- [ ] `tests/test_place_recognizer.cpp`:
  - 10-frame database: query with same descriptor → correct frame ID returned (score ≥ threshold)
  - Query with perturbed descriptor (5% noise): top-1 still correct
  - False positive rate: random descriptor → no match above threshold (100 random queries, < 2 false positives)
  - Lidar descriptor: scan histogram passed as `PlaceDescriptor`; visual descriptor: aggregated BRIEF as `PlaceDescriptor` — same API handles both
  - Large database (1000 frames): query completes in < 10 ms (linear scan acceptable at this scale)
- [ ] Phase 4.5: `ILogger`, database size + query time at `DEBUG`, candidate scores at `TRACE`
- [ ] Integration note: output `PlaceCandidate` list can be routed to `pose_graph (M14)` as loop closure edges

##### `perception/stereo_depth`

Block-matching stereo disparity estimation and disparity-to-depth conversion. `StereoRectifier` applies calibration to undistort and row-align stereo pairs (epipolar lines become horizontal rows).

- [ ] `include/stereo_depth/stereo_matcher.hpp` — `StereoMatcher` (block matching; optional SGM flag)
- [ ] `include/stereo_depth/stereo_rectifier.hpp` — `StereoRectifier` (apply `StereoCalibration` from `common/camera.hpp`)
- [ ] `include/stereo_depth/stereo_types.hpp` — `DisparityMap`, `DepthMap`, `StereoCalibration` (baseline, K_left, K_right, R, t)
- [ ] `src/stereo_matcher.cpp` + `src/stereo_rectifier.cpp`
- [ ] `tests/test_stereo_depth.cpp`:
  - Synthetic stereo pair (known disparity, integer shifts): block matching recovers disparity within ±1 px
  - Disparity-to-depth: `Z = f·b/d` — computed depth matches known ground truth within 1% for d ≥ 2 px
  - Rectified pair: after `StereoRectifier`, corresponding points share the same row (epipolar constraint verified on 10 point pairs)
  - Invalid pixels: disparity = 0 (occluded) → depth = `std::numeric_limits<float>::infinity()` or NaN (documented choice)
  - Edge case: block size larger than image → returns error via `std::expected`
- [ ] Phase 4.5: `ILogger`, disparity search range + match quality at `DEBUG`, per-row solve time at `TRACE`
- [ ] Sim note: stereo integration requires two camera viewpoints offset by baseline. M17 scope covers offline/synthetic testing; live sim stereo rendering deferred to post-M17 sim upgrade.
- [ ] Frontend: depth map false-colour overlay; disparity histogram panel

#### Exit Criteria

1. Place recognizer: 10-frame database → correct revisit detected; false positive rate < 2/100
2. Works with both lidar scan and visual feature descriptor formats (same `PlaceDescriptor` type)
3. Stereo depth: synthetic pair with known disparity → depth error < 1%
4. StereoRectifier: epipolar constraint satisfied on all test point pairs after rectification
5. All unit tests pass, CI green
6. All modules pass Phase 4.5 — Observability gate

#### NOT IN

Semi-global matching (full SGM graph cuts — block matching is the M17 target; SGM is a flag for future extension), RGB-D camera processing (→ P6 `depth_camera`), dense 3D reconstruction, Bag-of-Words vocabulary training (→ `visual_slam` uses its own vocabulary internally in M6.5).

---

## Module → Milestone Mapping

| Module | Path | Milestone | Priority |
|--------|------|-----------|---------|
| `lqr` | `control/lqr` | M13 | P2 (unresolved) |
| `stanley` | `control/stanley` | M13 | P4 |
| `ukf` | `state_estimation/ukf` | M14 | P2 (unresolved) |
| `pose_graph` | `state_estimation/pose_graph` | M14 | P3 |
| `imu_processing` | `perception/imu_processing` | M15 | P2 (unresolved) |
| `visual_inertial_odometry` | `state_estimation/visual_inertial_odometry` | M15 | P3 |
| `prm` | `motion_planning/global_planning/prm` | M16 | P3 |
| `polynomial` | `motion_planning/trajectory_planning/polynomial` | M16 | P4 |
| `place_recognition` | `perception/place_recognition` | M17 | P4 |
| `stereo_depth` | `perception/stereo_depth` | M17 | P4 |

---

## Dependency Graph Additions

### New edges

```
M3  (control upgrades)         → M13 (classical & optimal control)
M11 (advanced control)         → M13 (recommended; shared Q/R gain intuition)
M5  (state estimation)         → M14 (advanced state estimation II)
M6.5 (SLAM)                   ←── M14 (optional: swap loop closure backend to pose_graph)
M6  (visual perception)        → M15 (VIO)
M14 (ukf)                      → M15 (VIO)
M7  (advanced planning)        → M16 (planning upgrades II)
M4  (perception)               → M16 (OccupancyGrid for PRM collision checking)
M6  (visual perception)        → M17 (camera perception II)
M14 (pose_graph)               ←── M17 (optional: place_recognition feeds loop closure edges)
```

### Updated full graph (new nodes in **bold**)

```
M0 (infra)
 └→ M1 (minimum viable robot)
      └→ M2 (hardening & testing)
           ├→ M3 (control upgrades)       → M11 (advanced control)      → M13 (classical & optimal control)
           ├→ M4 (perception upgrades)
           │    └→ M6 (visual perception)
           │         ├→ M6.5 (SLAM) ←── M5 ──── M14 (state estimation II) → M15 (VIO)
           │         ├→ M15 (VIO) ←── M14
           │         └→ M17 (camera perception II)
           ├→ M5 (state estimation upgrades)
           │    └→ M14 (state estimation II) → M15 (VIO)
           ├→ M7 (advanced planning) ←── M4
           │    └→ M16 (planning upgrades II) ←── M4
           ├→ M8 (multi-robot) ←── M3, M7
           │    └→ M12 (fleet management)
           ├→ M9 (web frontend)
           └→ M10 (polish & showcase) ←── M8, M9
```

### Parallelism after M2

After M2, the new milestones fan out as follows:
- M13 is unblocked once M3 is complete
- M14 is unblocked once M5 is complete  
- M15 is unblocked once M6 and M14 are both complete
- M16 is unblocked once M7 is complete
- M17 is unblocked once M6 is complete

M14 and M17 can start in parallel (different domains). M16 and M17 can start in parallel. M15 is the most downstream new milestone — it requires both M6 and M14.

---

## Concerns & Tradeoffs

### 1. LQR in M13 vs backfilling M11

LQR was proposed in todos.md as "M3 or M3.5." Creating M13 avoids retrofitting a defined milestone and avoids making M11 overloaded (it already has three modules). The numbering gap (M11 then M13) is acceptable since milestones do not need to be sequential by completion order.

**Alternative considered:** M3.5 "Control Optimal" with only LQR. Rejected because single-module milestones are less teachable and Stanley (P4) is a natural pairing.

---

### 2. UKF in M14 vs backfilling M5

M5 is defined as "Particle Filter (MCL)." UKF is a natural companion at the same tier. However, M5 is already self-contained around particle-based estimation to contrast with EKF. Adding UKF to M5 would dilute the filter-comparison narrative of M5.

In M14, UKF pairs with `pose_graph` (both are Eigen-based mathematical algorithms) and creates the clean `M14 → M15` dependency chain. If needed, UKF can be presented as a retrospective upgrade to M5 in docs without being in the M5 definition.

---

### 3. Stanley + Ackermann kinematics in simulation

Stanley is Ackermann-friendly. The current sim uses a differential-drive kinematic model. Sim integration (Phase 5) may require one of:
- A bicycle-model kinematic layer in the sim (Ackermann approximation)
- A new Ackermann robot type alongside the diff-drive robot

This is a **simulation concern, not a library concern.** The `StanleyController` library implementation is kinematic-model-agnostic (it outputs a steering angle δ). Sim integration may be deferred to a post-M13 sim upgrade; the Phase 5 task in the module file should flag this.

---

### 4. Pose graph scope (SE2 only)

The M14 pose graph targets SE2 (2D SLAM) for implementation simplicity and direct integration with the existing 2D simulation. An SE3 extension (for 3D SLAM or VIO pose graph) is deferred to a future milestone, either as M18 or via the P5 `factor_graph` module which is a generalization. The SE2 design should be written to minimize SE2-specific assumptions (use `Sophus`-style conventions even if SE2 specific for now) to ease future SE3 extension.

---

### 5. VIO complexity — consider splitting M15

VIO is the most complex single module in this batch. `imu_processing` can be implemented independently and tested standalone; `visual_inertial_odometry` is the complex integrator.

If implementation velocity during M15 suggests it is too large for a single tracking unit, M15 can be split into:
- M15a: `imu_processing` only (small, well-bounded)
- M15b: `visual_inertial_odometry` (complex, depends on M15a + M6 + M14)

As currently designed, M15 is the intended "visual-inertial chapter." This should be reassessed when M14 is complete and M15 begins.

---

### 6. PRM vs RRT* performance narrative

PRM is multi-query (amortizes roadmap build cost across queries); RRT* is single-query optimal. M16 docs (theory.md) should clearly explain this tradeoff. PRM is appropriate when the environment is near-static and many similar queries are expected; RRT* excels for one-shot queries in dynamic environments. Both PRM and RRT* should be benchmarked on the same scenario in the M16 sim demo.

---

### 7. Stereo depth simulation support

M6 adds a single monocular 2.5D renderer. `stereo_depth` (M17) requires two camera views offset by a baseline. Live sim integration of stereo depth requires either:
- Two sim camera instances with fixed baseline offset (relatively simple extension to the sim's camera renderer)
- Synthetic stereo test pairs generated procedurally (sufficient for all unit tests and CI)

The M17 module design works fully offline with synthetic pairs. Live sim rendering is noted in the Phase 5 task as requiring a sim stereo-camera upgrade, which can be scoped separately.

---

### 8. Place recognition algorithm choice

The design targets L2 / cosine similarity over `PlaceDescriptor` (a `std::vector<float>`). This is intentionally simple:
- No vocabulary tree or Fisher vector encoding (→ too complex for P4 scope)
- No GPU FLANN index (Eigen-only, linear scan for ≤ 1000 frames)

For larger deployments (10k+ frame databases) a k-d tree or inverted index would be needed. The API is designed so this is an internal implementation detail — external callers only call `query(descriptor)`. The `NOT IN` section explicitly calls out vocabulary training.

---

## README.md Milestone Table Update

The following rows should be appended to the milestone table in `repo-plans/README.md`:

```markdown
| **M13** | [Classical & Optimal Control](milestones/M13-classical-optimal-control.md) | LQR + Stanley controller | Not Started |
| **M14** | [Advanced State Estimation II](milestones/M14-advanced-state-estimation.md) | UKF + pose graph optimizer | Not Started |
| **M15** | [Visual-Inertial Odometry](milestones/M15-visual-inertial-odometry.md) | IMU processing + loosely-coupled VIO | Not Started |
| **M16** | [Planning Upgrades II](milestones/M16-planning-upgrades-2.md) | PRM global planner + polynomial trajectories | Not Started |
| **M17** | [Camera Perception II](milestones/M17-camera-perception-2.md) | Place recognition + stereo depth | Not Started |
```

The dependency graph note should be extended with:

> After M3/M11: M13 (classical & optimal control). After M5: M14 (UKF + pose graph). After M6 + M14: M15 (VIO). After M7 + M4: M16 (PRM + polynomial). After M6: M17 (camera perception II). M6.5 (SLAM) can optionally adopt pose_graph (M14) for its loop closure backend once M14 is complete.

---

## Next Steps

1. **Append M13–M17** to `repo-plans/README.md` milestone table
2. **Create milestone docs** at `repo-plans/milestones/M13-*.md` through `M17-*.md` (use this design as source of truth)
3. **Update todos.md** — mark P2 items LQR, UKF, IMU processing as resolved (placed in M13/M14/M15)
4. **Create module task files** when work on each milestone begins: `repo-plans/modules/lqr.md`, `ukf.md`, etc.
5. **Pre-scaffold module directories** (optional ahead of milestone start): `workspace/robotics/control/lqr/`, `workspace/robotics/control/stanley/`, etc.
6. **Transition to implementation planning** (superpower-plan) starting with M13, which has the most mature dependencies (M3 is closest to early milestones in sequence).
