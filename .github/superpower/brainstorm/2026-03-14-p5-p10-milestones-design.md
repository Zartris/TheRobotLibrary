# Design: P5–P10 Priority Milestones (M18–M24)

**Date:** 2026-03-14  
**Status:** Approved  
**Topics:** New milestones M18–M24 covering P5 through P10 modules from the gap analysis. Continues from [2026-03-14-p3-p4-milestones-design.md](2026-03-14-p3-p4-milestones-design.md) (M13–M17).

---

## Summary

Seven new milestones (M18–M24): six two-module milestones and one solo fleet milestone, covering 13 modules total.

| Milestone | Name | Modules |
|-----------|------|---------|
| M18 | Advanced Nonlinear Control | `control/mppi` · `control/feedback_linearization` |
| M19 | Depth Perception & 3D Understanding | `perception/depth_camera` · `perception/object_detection_3d` |
| M20 | Planning Upgrades III | `motion_planning/local_planning/potential_field` · `motion_planning/global_planning/informed_rrt_star` |
| M21 | Estimation & Test Foundations | `state_estimation/factor_graph` · `common/noise_models` |
| M22 | Optimal Output Feedback & Automotive Perception | `control/lqg` · `perception/lane_detection` |
| M23 | Lattice Planning & Semantic Vision | `motion_planning/global_planning/lattice_planner` · `perception/semantic_segmentation` |
| M24 | Fleet Operations | `fleet_management/charging_station` |

---

## Design Questions — Answers & Rationale

### Q1: Grouping — how many milestones?

**Decision:** Seven milestones (M18–M24) — a deviation from the "6 milestones" recommendation.

The 13 modules across P5–P10 do not divide evenly into groups of two without forcing unnatural pairings. The key driver is `fleet_management/charging_station` (P10): it belongs to a completely different domain from every other P5–P10 module, and its only natural partner modules are already in M12. Making it a focused solo milestone (M24) is cleaner than pairing it with an unrelated module.

All other milestones (M18–M23) are two-module milestones grouped by domain cohesion:

- **M18**: Two advanced control techniques (same domain, share `IController` interface)
- **M19**: Two 3D perception modules (same domain, deepen lidar/camera perception into the 3D space)
- **M20**: Two planning upgrades — one local, one global (share `OccupancyGrid` + planning infrastructure)
- **M21**: Two infrastructure/utility modules (general estimation backend + reproducible noise generation)
- **M22**: Two P8 modules — control synthesis + perception (both P8 priority, independent domains but same milestone rhythm)
- **M23**: Two P9 modules — structured-env planning + semantic vision stub
- **M24**: One fleet module (solo, fleet domain only)

---

### Q2: Milestone dependency ordering

**Decision:** M18 → recommended after M13; M19 → after M6 + M4; M20 → after M7; M21 → after M14 (recommended); M22 → after M13; M23 → after M7 + M6; M24 → after M12.

**Parallel after M13:** M18 and M22 both depend on M13 (LQR/`IController`) and are independent of each other — they can proceed concurrently.  
**Parallel after M6+M4:** M19 and M20 both require perception infrastructure (M4 for OccupancyGrid/lidar; M6 for camera types) and can proceed concurrently.  
**M21 vs M22:** M21 depends on M14 (after pose_graph patterns); M22 depends on M13 (after LQR). Independent — can run in parallel once respective prerequisites are met.  
**M23:** Requires both M7 (planning interfaces) and M6 (image types for semantic seg); independent of M18–M22.  
**M24:** Requires M12 only; fully independent of M18–M23 — can proceed at any time after M12.

Recommended sequencing for a single developer: M18 → M19 → M20 → M21 → M22 → M23 → M24 (follows priority order within each prerequisite tier). This ordering teaches: nonlinear control → 3D perception → planning upgrades → estimation foundations → output feedback → lattice/semantic → fleet.

---

### Q3: `noise_models` placement

**Decision:** Override the Q3 recommendation in favor of Q5. Place `common/noise_models` in **M21** paired with `state_estimation/factor_graph`, not at P10 paired with `charging_station`.

**Rationale for the Q3 recommended answer** (P10, paired with charging_station): Both are quality-of-life / polish items at the end of the roadmap; tests work without formalized noise models.

**Rationale for the Q5 recommended answer** (M21, paired with factor_graph): Both are infrastructure utilities that improve *every other module's* tests. `factor_graph` in particular needs reproducible noise injected into simulated measurements to test convergence — having `noise_models` in the same milestone avoids a gap where the module exists but its primary test aid does not.

**Resolution:** Domain cohesion and developer experience win over priority grouping. `noise_models` + `factor_graph` in M21 creates a clean "Foundations II" chapter that pairs two general-purpose infrastructure items. `charging_station` becomes a clean solo M24 milestone (fleet domain). The Q3/Q5 conflict is noted explicitly: the two recommendations were mutually exclusive; Q5 was chosen.

---

### Q4: `semantic_segmentation` DL dependency

**Decision:** Stub-only scope in M23 — no real inference; define a plugin interface for a future DL backend.

The alternatives evaluated:

| Option | Verdict |
|--------|---------|
| Hard-require DL backend (ONNX Runtime / TensorRT) | **Rejected** — Introduces a GPU/DL build dependency that breaks the library's build-anywhere philosophy. Doubles CMake complexity. |
| Defer the milestone entirely | **Rejected** — The interface design (SemanticMap, SemanticClass enum, segmenter abstraction) has value even without inference. Deferral leaves a known gap unaddressed. |
| Stub-only + plugin interface | **Accepted** — Correctly scoped. `ISemanticSegmenter` + `StubSemanticSegmenter` define the interface; a `std::function<SemanticMap(RgbImage)>` injection point documents where a future ONNX/TensorRT backend plugs in. M23 is fully completable with zero DL framework dependency. |

See the M23 "Concerns" section at the end of this document for a full discussion.

---

### Q5: `factor_graph` placement relative to estimation chapter size

**Decision:** Pair `factor_graph` with `common/noise_models` in M21 (contra grouping multiple estimation modules together). See Q3 answer for rationale.

The concern about the estimation chapter growing too large (M14 + M15 already cover UKF, pose_graph, IMU processing, VIO) is valid. Adding a full general factor graph optimiser to that chapter would create a four-module estimation block. M21 as a separate "Foundations II" milestone keeps M14 and M15 self-contained. `factor_graph` in M21 can optionally become the backend for `pose_graph` (M14) and both SLAM variants (M6.5) — this upgrade path is documented in M21 as a post-completion integration note.

---

### Q6: MPPI placement

**Decision:** `control/mppi` paired with `control/feedback_linearization` in M18 — "Advanced Nonlinear Control." Follows the recommendation.

Both are advanced nonlinear control techniques that extend the M3/M13 control chapter. They are independent within the milestone (no cross-dependency). MPPI is stochastic MPC (randomised trajectory sampling + importance weighting); feedback linearization is deterministic exact linearization (diffeomorphism). Together they represent the two dominant approaches to handling system nonlinearity in control: randomised optimisation vs. exact algebraic transformation. Grouping MPPI with LQG (a stochastic-controller argument) would mix optimal-output-feedback concerns with rollout-sampling concerns, reducing the teachability of M22 as the "LQG completes the output feedback picture" chapter.

---

## Proposed Milestone Structure

---

### M18 — Advanced Nonlinear Control

**Status:** Not Started  
**Dependencies:** M3 (stable `IController` interface + kinematic models), M13 (control conventions; recommended, not required).  
**Scope:** Two advanced nonlinear control algorithms extending the M3/M13 control chapter. MPPI provides stochastic trajectory optimisation via Monte Carlo rollouts; feedback linearization provides exact input-output linearization for differential-drive systems. Both are independently implementable within M18.

#### Goal

M18 pushes control beyond deterministic-optimal (LQR/MPC) and geometric (pure_pursuit/Stanley) into the two dominant approaches for nonlinear systems: randomized optimization (MPPI) and algebraic state transformation (feedback linearization). MPPI is increasingly used in aggressive autonomy (legged locomotion, racing); feedback linearization is the textbook gateway to Lie-group control theory. Both sit naturally after M13's LQR/Stanley and before any advanced estimation work.

#### Modules

##### `control/mppi`

Model Predictive Path Integral controller. Samples N control trajectories as Gaussian perturbations of a nominal sequence, propagates each through the robot's motion model, scores by a configurable cost function, and computes the importance-weighted update to the nominal control.

- [ ] `include/mppi/mppi_controller.hpp` — `MPPIController : IController`
- [ ] `include/mppi/mppi_config.hpp` — `MPPIConfig` (N rollouts, horizon H, temperature λ, noise covariance Σ, dt)
- [ ] `include/mppi/mppi_cost.hpp` — `MPPICostFn = std::function<double(const Eigen::VectorXd&, const Eigen::VectorXd&)>` (state, control → cost scalar)
- [ ] `src/mppi_controller.cpp` — Monte Carlo rollout loop, importance weight normalization, control update: `u* = Σ (w_k · ε_k)` where `w_k = exp(−J_k / λ)`
- [ ] Configurable motion model via `std::function<VectorXd(VectorXd, VectorXd)>` — allows testing on diff-drive and double-integrator
- [ ] `tests/test_mppi_controller.cpp`:
  - Double integrator: MPPI drives state to target; final error < tolerance
  - Obstacle avoidance: high-cost obstacle region → sampled distribution clusters around valid trajectories (importance weights on obstacle rollouts → near zero)
  - Weight normalization: sum of `w_k` over N rollouts = 1 for any λ > 0
  - Temperature λ: high λ → uniform weights (exploration); low λ → peaked on best rollout (exploitation)
  - N = 1 (degenerate case): no crash; produces finite control output
- [ ] Phase 4.5: `ILogger`, rollout cost distribution (min/max/mean) at `DEBUG`, per-iteration solve time at `TRACE`
- [ ] Sim: selectable via `PUT /api/robot/controller {"type":"mppi"}`
- [ ] Frontend: visualise rollout fan (N sampled trajectories, grey) + selected trajectory (highlighted)

##### `control/feedback_linearization`

Exact input-output linearization for a differential-drive robot in chained form. Transforms the nonlinear system `(x, y, θ, v)` to a pair of decoupled linear integrators by introducing a virtual output point ahead of the robot's axle, then applies a `LinearController` (PD or LQR) to the linearized system. Includes a numerical Lie derivative checker for test verification.

- [ ] `include/feedback_linearization/feedback_linearization_controller.hpp` — `FeedbackLinearizationController : IController`
- [ ] `include/feedback_linearization/feedback_linearization_config.hpp` — `FeedbackLinearizationConfig` (lookahead `l`, inner controller gains, velocity limits)
- [ ] `src/feedback_linearization_controller.cpp` — diffeomorphism: virtual output `ξ = (x + l·cos θ, y + l·sin θ)`; linearizing control law → `(v, ω)`; inverse kinematics back to robot frame
- [ ] `src/lie_derivative_check.cpp` — numerical check: verify `L_f h(x)` via finite-difference matches analytic expression for test verification
- [ ] `tests/test_feedback_linearization_controller.cpp`:
  - Straight-line tracking: virtual output tracks reference line; `(x, y, θ)` converges within tolerance
  - Circular arc: nonlinear system tracks circular reference; CTE bounded
  - Lookahead `l`: larger `l` → smoother but lagged response; smaller `l` → tighter but noisier
  - Lie derivative check: numerical `L_f h(x)` matches analytic expression (< 1e-5 relative error)
  - Singularity: `l = 0` → documented error via `std::expected`; no division by zero in production code
- [ ] Phase 4.5: `ILogger`, virtual output error at `DEBUG`, Lie derivative residual at `TRACE`
- [ ] Sim: selectable via `PUT /api/robot/controller {"type":"feedback_linearization"}`
- [ ] Frontend: render virtual output point trajectory overlay

#### Exit Criteria

1. MPPI drives double integrator to target within tolerance; importance weights sum to 1
2. MPPI obstacle avoidance: obstacle-colliding rollouts receive near-zero weight
3. Feedback linearization: straight-line and circular-arc tracking within CTE tolerance
4. Lie derivative numerical check passes (< 1e-5 relative error vs. analytic)
5. Both controllers hot-swappable via REST mid-run without crash
6. All unit tests pass, CI green
7. All modules pass Phase 4.5 — Observability gate (cost distribution at `DEBUG`, timing at `TRACE`)

#### NOT IN

GPU-accelerated rollouts (CUDA MPPI), gradient-based trajectory optimisation (iLQR/DDP), full SE3 diffeomorphism (→ future), Lie group Kalman filters, feedback linearization for non-diff-drive kinematics (Ackermann, holonomic).

---

### M19 — Depth Perception & 3D Understanding

**Status:** Not Started  
**Dependencies:** M6 (`common/camera.hpp` intrinsics + image types), M4 (`obstacle_detection` 2D clustering as conceptual predecessor; `OccupancyGrid` types).  
**Scope:** Adds RGB-D camera processing and 3D object detection from point clouds. Both modules deepen the perception stack into the true 3D domain. `depth_camera` handles a new sensor type (structured light / ToF); `object_detection_3d` elevates M4's 2D DBSCAN to full 3D oriented bounding boxes.

#### Goal

M4's `obstacle_detection` detects 2D clusters in a lidar scan. M19 extends this in two directions: `depth_camera` adds a new sensor pathway (RGB-D → point cloud) that feeds the same downstream processing; `object_detection_3d` recognizes full 3D extent and orientation of objects, enabling height-aware classification needed for real environments (pedestrians, cars, robots of different sizes). Both modules are independently implementable.

#### Modules

##### `perception/depth_camera`

RGB-D camera processing pipeline. Converts raw depth images (uint16 mm or float32 m) to dense point clouds using camera intrinsics, fills holes via averaging, removes statistical outliers, and provides an `RgbdCamera` wrapper that bundles aligned RGB + depth.

- [ ] `include/depth_camera/depth_image_processor.hpp` — `DepthImageProcessor`
- [ ] `include/depth_camera/rgbd_camera.hpp` — `RgbdCamera` (intrinsics from `common/camera.hpp` `CameraIntrinsics`, depth scale factor, min/max valid depth range)
- [ ] `include/depth_camera/depth_types.hpp` — `DepthImage` (float32 row-major grid, NaN = invalid), `RgbdFrame` (rgb image + depth image + timestamp), `PointCloud` (`Eigen::MatrixX3f`, one row per valid point)
- [ ] `src/depth_image_processor.cpp` — depth → point cloud: `X = (u − cx)·d/fx`, `Y = (v − cy)·d/fy`, `Z = d`; hole filling: NaN → average of valid 8-neighbours (max 2 passes); outlier removal: statistical (mean + k·σ distance filter)
- [ ] Frustum filtering: discard points outside configured min/max depth and field-of-view cone
- [ ] `tests/test_depth_camera.cpp`:
  - Flat plane at known depth `Z = 1.0 m`: all valid pixels → point cloud; mean Z within 0.1% of 1.0 m
  - `X = (u − cx)·d/fx` — three known pixels verify correct 3D position (corner + centre)
  - Depth image with 20% random NaN holes: after hole filling, < 2% NaN remaining; filled values within 5% of ground truth
  - Statistical outlier removal: synthetic cloud with 5 outlier points 10σ away → outliers removed, inliers intact
  - Frustum filter: points beyond max depth → excluded; points within frustum → retained
  - Empty depth image (all NaN): `toPointCloud()` → empty point cloud; no crash
- [ ] Phase 4.5: `ILogger`, valid pixel count + hole-fill pass count at `DEBUG`, projection time per frame at `TRACE`
- [ ] Sim note: depth camera simulation requires sim renderer extension (render depth buffer). M19 scope covers offline/synthetic testing; live sim RGB-D rendering deferred to a post-M19 sim upgrade.
- [ ] Frontend: false-colour depth overlay panel (hot = close, cool = far)

##### `perception/object_detection_3d`

3D object detector from point clouds. Runs 3D DBSCAN to find clusters, fits an oriented bounding box to each cluster using PCA (principal components → box axes and dimensions), and applies a size-based classification stub.

- [ ] `include/object_detection_3d/object_detector_3d.hpp` — `ObjectDetector3D`
- [ ] `include/object_detection_3d/object_detection_3d_config.hpp` — `ObjectDetection3DConfig` (DBSCAN: `eps`, `min_points`; box: `min_cluster_size`, `max_cluster_size`)
- [ ] `include/object_detection_3d/object_3d_types.hpp` — `Object3D` (centroid `Eigen::Vector3f`, dimensions `Eigen::Vector3f` in sorted order, orientation `Eigen::Matrix3f` rotation, `ObjectClass` enum: `PERSON / CAR / UNKNOWN`), `Detection3DList`
- [ ] `src/object_detector_3d.cpp` — 3D DBSCAN: KD-tree (Eigen KD-tree or hand-rolled) for ε-neighbour queries; PCA box fitting: covariance matrix → `SelfAdjointEigenSolver` → eigenvectors = box axes; classification: `PERSON` if height > 1.0 m AND footprint < 0.5×0.5 m, `CAR` if footprint > 1.5×1.5 m, else `UNKNOWN`
- [ ] `tests/test_object_detector_3d.cpp`:
  - Synthetic point cloud with 3 isolated clusters (known centroid, separation > 2ε): DBSCAN returns exactly 3 clusters
  - Flat disk cluster (known PCA): box major axis matches disk normal (< 5° angular error)
  - Tall thin cluster 0.4×0.4×1.8 m: classified as `PERSON`
  - Wide flat cluster 2.0×1.5×0.5 m: classified as `CAR`
  - Single-point cluster (below `min_points`): filtered out; not returned as object
  - Empty point cloud: returns empty `Detection3DList`; no crash
- [ ] Phase 4.5: `ILogger`, cluster count + per-cluster point count at `DEBUG`, DBSCAN + PCA time at `TRACE`
- [ ] Sim: integrate with lidar point cloud pipeline; 3D objects renderable as oriented boxes in frontend
- [ ] Frontend: render oriented bounding boxes with class label overlays in 3D scene view

#### Exit Criteria

1. Depth → point cloud: pixel positions correctly back-projected (verified on 3 known pixels)
2. Hole filling: < 2% NaN remaining after filling a 20% hole depth image
3. DBSCAN finds correct cluster count on synthetic 3-cluster point cloud
4. PCA box major axis within 5° of ground truth orientation
5. Classification stub: PERSON / CAR rules produce correct labels on synthetic clusters
6. All unit tests pass, CI green
7. All modules pass Phase 4.5 — Observability gate

#### NOT IN

Dense 3D reconstruction (TSDF / surfel maps), semantic 3D labelling (→ M23 `semantic_segmentation`), real-time GPU-accelerated DBSCAN, IMU-aligned point clouds, RGB-D SLAM (requires M6.5 + future extension).

---

### M20 — Planning Upgrades III

**Status:** Not Started  
**Dependencies:** M7 (`IGlobalPlanner` interface, `RRTStar` implementation, planning infrastructure), M4 (`OccupancyGrid` for collision checking). M16 (PRM/polynomial) recommended for context but not required.  
**Scope:** Two complementary planning additions: a classical reactive local planner (potential fields) and an asymptotically optimal global planner (Informed RRT*). Both extend M7's planning infrastructure; both are independently implementable within M20.

#### Goal

M7 established the global planning chapter with RRT* and local planning with DWA/TEB. M20 adds the two missing entries: potential fields as the simplest reactive local planner (every robotics curriculum's starting point; teaches attractive/repulsive gradient field design and local minimum phenomena); Informed RRT* as the sample-efficiency upgrade to RRT* (ellipsoidal heuristic region drastically narrows the search space after a first feasible path is found). Together they complete the "classical → probabilistic → informed" planning curriculum arc.

#### Modules

##### `motion_planning/local_planning/potential_field`

Artificial potential field planner. Combines an attractive parabolic/conic field toward the goal with repulsive inverse-square fields from obstacles. Local minima escape is triggered when the robot's progress stalls (velocity below threshold for N consecutive steps), applying a random perturbation.

- [ ] `include/potential_field/potential_field_planner.hpp` — `PotentialFieldPlanner : ILocalPlanner`
- [ ] `include/potential_field/potential_field_config.hpp` — `PotentialFieldConfig` (attractive gain `k_att`, repulsive gain `k_rep`, influence radius `d_0`, step size, stall threshold, escape steps)
- [ ] `src/potential_field_planner.cpp` — attractive force: parabolic `k_att · (goal − pos)` within `d_switch`, conic `k_att · d_switch · (goal − pos)/|goal − pos|` beyond; repulsive force: `k_rep · (1/d − 1/d_0) · (1/d²) · ∇d` for `d < d_0`; resultant gradient descent step
- [ ] Obstacle distance queries via `OccupancyGrid` nearest-obstacle lookup
- [ ] `tests/test_potential_field_planner.cpp`:
  - Open space, no obstacles: robot reaches goal within N steps; final position within 0.1 m of goal
  - Single obstacle between start and goal: robot deflects around obstacle; does not collide
  - Designed local minimum (symmetric obstacle arrangement): escape triggered within M steps; robot eventually reaches goal
  - Goal coincides with repulsive field: documented edge case (repulsion wins → goal unreachable with this configuration); returns `std::nullopt`
  - `d_0 = 0` (no repulsion): pure attractive field; straight-line motion
- [ ] Phase 4.5: `ILogger`, net force magnitude + stall detection at `DEBUG`, per-step potential gradient at `TRACE`
- [ ] Sim: selectable via `PUT /api/robot/local_planner {"type":"potential_field"}`
- [ ] Frontend: render potential field gradient vectors (arrow overlay, optional toggle)

##### `motion_planning/global_planning/informed_rrt_star`

Informed RRT*. Runs standard RRT* until the first feasible path is found, then switches to sampling exclusively within the prolate hyperspheroid (PHS) defined by the current best path cost `c_best` and the start-goal Euclidean distance `c_min`. Sampling within the PHS is achieved by transforming uniform samples in a unit ball via the rotation and scale of the ellipsoid.

- [ ] `include/informed_rrt_star/informed_rrt_star_planner.hpp` — `InformedRRTStar : IGlobalPlanner`
- [ ] `include/informed_rrt_star/informed_rrt_star_config.hpp` — `InformedRRTStarConfig` (max_iterations, step_size, rewire_radius, initial_rrt_budget — iterations before switching to informed phase)
- [ ] `src/informed_rrt_star_planner.cpp` — standard RRT* phase until first feasible path; PHS transformation: `x_sample = C · diag(r1,...,rn) · x_ball + x_centre`, where `r1 = c_best/2`, `r2 = ... = rn = sqrt(c_best² − c_min²)/2`, `C = rotation from start→goal axis`; switch to informed sampling once `c_best < ∞`
- [ ] `tests/test_informed_rrt_star.cpp`:
  - Obstacle-free: finds path; solution length ≤ 1.05 × `c_min` after sufficient iterations
  - Convergence speed comparison: Informed RRT* reaches `c_best ≤ 1.1 × c_min` in ≤ 50% of the iterations that vanilla RRT* requires (statistical over 10 seeds)
  - PHS bound check: all samples after first solution found lie strictly within the ellipsoid (verify by checking `x^T * M^-1 * x ≤ 1` for transform matrix M)
  - C-space obstacle: path correctly avoids obstacle; cost strictly decreases (or stays same) across iterations
  - No feasible path possible: returns `std::nullopt`; no crash
- [ ] Phase 4.5: `ILogger`, `c_best` improvement per iteration at `DEBUG`, PHS volume reduction at `TRACE`
- [ ] Sim: selectable via `PUT /api/robot/global_planner {"type":"informed_rrt_star"}`
- [ ] Frontend: render PHS ellipse overlay; show `c_best` convergence curve in side panel

#### Exit Criteria

1. Potential field planner reaches goal in open space within N steps
2. Potential field local minimum escape triggers within M stall steps and succeeds
3. Informed RRT* finds feasible path in cluttered map
4. Informed RRT* converges to near-optimal cost faster than vanilla RRT* (measured across 10 seeds)
5. All post-first-solution samples lie within the prolate hyperspheroid (geometric check)
6. All unit tests pass, CI green
7. All modules pass Phase 4.5 — Observability gate

#### NOT IN

Potential field with navigation function (guaranteed global convergence — too complex for M20), 3D APF, Informed PRM (→ future), bidirectional Informed RRT*, motion-primitive informed sampling (→ M23 `lattice_planner`).

---

### M21 — Estimation & Test Foundations

**Status:** Not Started  
**Dependencies:** M14 (`pose_graph` GN/LM optimizer as structural predecessor; `common` types). `common/noise_models` has zero dependencies.  
**Scope:** Two infrastructure/utility modules. `factor_graph` is the general graph estimator that subsumes `pose_graph` (M14); `noise_models` provides reproducible seeded noise generation for testing across all modules. Both are "Foundations II" — they improve rigor throughout the library without being prerequisites for most other modules.

#### Goal

M14's `pose_graph` is a specialized SE2-pose-only optimizer. `factor_graph` generalizes this to support heterogeneous variable types (poses, landmarks, IMU biases) and arbitrary factor types (odometry, observation, prior) — the pattern underlying g2o, GTSAM, and Ceres. Once available, SLAM variants (M6.5), VIO (M15), and future perception modules can optionally adopt it as a consistent backend. `noise_models` solves a persistent test-quality problem: all modules currently inject noise ad hoc with non-seeded RNGs, making tests non-reproducible. Promoting noise injection to a first-class `common/` utility is a small effort with system-wide payoff. Pairing these two modules reflects their shared character: both are infrastructure that makes every other module more rigorous.

**Note — Q3/Q5 conflict resolution:** The Q3 recommendation placed `noise_models` with `charging_station`; the Q5 recommendation placed it with `factor_graph`. These are mutually exclusive. M21 follows Q5: domain cohesion (two infrastructure utilities) produces a more teachable milestone than priority grouping. `charging_station` becomes the solo M24 fleet milestone.

#### Modules

##### `state_estimation/factor_graph`

General factor graph optimizer using a Gauss-Newton iterative solver with sparse Cholesky (Eigen `SimplicialLDLT`). Supports heterogeneous variable nodes and arbitrarily typed factor nodes. Designed as a lightweight Eigen-only replacement for g2o/Ceres/GTSAM at small graph sizes.

- [ ] `include/factor_graph/factor_graph.hpp` — `FactorGraph`; `addVariable<N>(id)` → `Variable<N>&`; `addFactor(factor)` → void; `optimize(maxIter, tol)` → `OptimizationResult`
- [ ] `include/factor_graph/variable.hpp` — `Variable<int N>` (id, `Eigen::VectorXd` value, fixed flag); `PoseVariable` (SE2: x, y, θ), `LandmarkVariable` (x, y), `BiasVariable` (accel_bias, gyro_bias)
- [ ] `include/factor_graph/factor.hpp` — `IFactor` interface: `residual()`, `jacobian()`, `informationMatrix()`; `OdometryFactor` (binary pose-pose), `ObservationFactor` (pose-landmark bearing-range), `PriorFactor` (unary, any variable type)
- [ ] `src/factor_graph.cpp` — GN iteration: assemble sparse Jacobian `J` + residual `r`; normal equations `J^T Ω J Δx = −J^T Ω r`; Eigen `SimplicialLDLT` solve; update variables; stop on `‖Δx‖ < tol`
- [ ] Gauge freedom: first non-fixed variable held fixed (prior factor at zero) OR explicit `setFixed(id)` call
- [ ] `tests/test_factor_graph.cpp`:
  - 2-node SE2 chain + `OdometryFactor`: optimizer converges; both poses match ground truth within 1e-6
  - 4-node SE2 loop (3 odometry factors + 1 loop closure): closed-loop poses converge; cumulative error < 0.05 m
  - Pose + landmark + `ObservationFactor`: landmark position converges from corrupted initial guess within 10 iterations
  - GN convergence: `‖Δx‖` decreasing monotonically on noise-free problem
  - Jacobian check: `OdometryFactor` analytic Jacobian matches finite-difference (< 1e-5 max element error)
  - Large graph (100 SE2 nodes, 10 loop closures): `optimize()` completes in < 1 s (Eigen sparse)
- [ ] Phase 4.5: `ILogger`, iteration count + final residual norm at `DEBUG`, per-iteration solve time at `TRACE`
- [ ] Integration note: `PoseGraph (M14)` can be re-implemented as a thin wrapper over `FactorGraph` post-M21; not a requirement of M21 itself

##### `common/noise_models`

Seeded, reproducible noise injection primitives. All types are templated on the scalar type `T` and accept an explicit RNG seed for deterministic test sequences. `OutlierInjector<T>` wraps any base noise model and probabilistically injects large-magnitude outliers.

- [ ] `include/common/noise_models.hpp` — `GaussianNoise<T>` (`μ=0`, configurable `σ`, `std::mt19937` seeded RNG); `UniformNoise<T>` (configurable `[lo, hi]`); `OutlierInjector<T>` (base noise model + outlier probability `p` + outlier range `[lo_out, hi_out]`)
- [ ] `src/noise_models.cpp` — `GaussianNoise::sample()` → `std::normal_distribution<T>`; `UniformNoise::sample()` → `std::uniform_real_distribution<T>`; `OutlierInjector::sample()` → either base model or uniform outlier (Bernoulli trial)
- [ ] Batch API: `sampleN(n)` → `std::vector<T>`; `addTo(Eigen::VectorXd&)` → in-place noise injection
- [ ] `tests/test_noise_models.cpp`:
  - Gaussian mean: 10000 samples → `|mean| < 0.01·σ`; variance: `|var − σ²| < 0.05·σ²`
  - Uniform coverage: bucket histogram of 1000 samples → all buckets within 20% of uniform expectation
  - Seeded reproducibility: two `GaussianNoise<double>` with same seed → identical `sampleN(100)` sequence
  - Different seeds → statistically distinguishable sequences (Kolmogorov-Smirnov p > 0.05 is NOT expected; sequences differ at first differing sample)
  - `OutlierInjector`: 1000 samples with `p = 0.1` → outlier count in `[70, 130]` (< 3σ of Binomial(1000, 0.1))
  - `addTo(Eigen::VectorXd&)`: adds correct noise dimension-wise; does not modify vector size
- [ ] Phase 4.5: `ILogger`, seed value at `DEBUG` on construction; not logged per-sample (too hot)

#### Exit Criteria

1. Factor graph 4-node SE2 loop closure converges to < 0.05 m position error
2. Analytic Jacobians of all factor types match finite-difference (< 1e-5 max element error)
3. 100-node sparse factor graph optimizes in < 1 s on CI hardware
4. Gaussian noise mean/variance converges to configured parameters over 10000 samples
5. Same seed → identical noise sequence (reproducibility verified in tests)
6. All unit tests pass, CI green
7. All modules pass Phase 4.5 — Observability gate

#### NOT IN

SE3 variable types (→ future), robust cost functions (Huber, Cauchy — → future), marginalization / Schur complement, GTSAM/g2o file format import, online incremental updates (iSAM2 style), pink/brown correlated noise models.

---

### M22 — Optimal Output Feedback & Automotive Perception

**Status:** Not Started  
**Dependencies:** M13 (`LQRController`, `IController` interface) for `lqg`; M6 (`common/camera.hpp`, image types) for `lane_detection`. M1 (EKF) implicitly used inside LQG.  
**Scope:** Two P8 modules from separate domains. `lqg` completes the optimal control sequence by adding a Kalman state observer to LQR, enabling output-only feedback. `lane_detection` adds structured-environment perception for automotive scenarios. Both are independently implementable within M22.

#### Goal

LQR (M13) is a state-feedback controller — it requires full state observation. LQG adds the observer half: an internal EKF (Kalman filter) that estimates the full state from output measurements only. This "separation principle" combination — design LQR and Kalman filter independently and combine — is one of the key theoretical results in modern control. M22 teaches this pattern. `lane_detection` provides a complementary perception capability for the same structured-environment / automotive scenarios where LQG output feedback is most relevant (lane-following with output-only measurements).

#### Modules

##### `control/lqg`

Linear Quadratic Gaussian controller. Wraps an `LQRController` (from M13) for state feedback and an internal `EKF` instance as the state observer. Accepts only output measurements `y = Cx + v` (not full state `x`). Demonstrates the separation principle: optimal LQR feedback gain and optimal Kalman estimate are designed independently and combined without loss of optimality.

- [ ] `include/lqg/lqg_controller.hpp` — `LQGController : IController`
- [ ] `include/lqg/lqg_config.hpp` — `LQGConfig` (system: `A, B, C`; LQR weights: `Q, R`; noise: process `Q_w`, measurement `R_v`; discretization `dt`)
- [ ] `src/lqg_controller.cpp` — initialize `LQRController` with `(A, B, Q, R)` → gain `K`; initialize internal `EKF` with `(A, B, C, Q_w, R_v)` → Kalman gain; `update(y)` → EKF predict + update; `computeControl(x̂)` → `LQR u = −K x̂`
- [ ] `tests/test_lqg_controller.cpp`:
  - Double integrator, output = position only (C = [1 0]): LQG drives position and velocity to zero; velocity estimate converges even though it is not measured
  - Separation principle: LQR gain `K` from `(A, B, Q, R)` matches `LQRController::computeGain()` independently; Kalman gain `L` from `(A, C, Q_w, R_v)` matches standalone EKF construction
  - AWGN measurement noise: steady-state estimation covariance matches theoretical DARE solution for Kalman filter
  - LQR + perfect state observer (C = I): LQG reduces to pure LQR (verify numerically)
  - Wrong noise covariance (R_v underestimated by 10×): controller still stabilizes, but estimation covariance differs — documents behavior
- [ ] Phase 4.5: `ILogger`, state estimate covariance trace + control effort at `DEBUG`, EKF residual norm at `TRACE`
- [ ] Sim: selectable via `PUT /api/robot/controller {"type":"lqg"}`
- [ ] Frontend: render estimated state covariance ellipse; control effort vs. time; estimation residual panel

##### `perception/lane_detection`

Lane marking extraction from binary edge images (e.g., Canny output). Applies Hough transform to detect line candidates, fits a polynomial to each lane cluster via RANSAC, and assigns left/right lane identity based on slope and image position.

- [ ] `include/lane_detection/lane_detector.hpp` — `LaneDetector`
- [ ] `include/lane_detection/lane_detection_config.hpp` — `LaneDetectionConfig` (Hough: rho resolution, theta resolution, threshold, min_line_length, max_line_gap; polynomial degree; RANSAC iterations)
- [ ] `include/lane_detection/lane_types.hpp` — `LanePolynomial` (coefficients `std::vector<double>`, degree, validity flag), `LaneResult` (left: `LanePolynomial`, right: `LanePolynomial`)
- [ ] `src/lane_detector.cpp` — Hough transform (Eigen-based accumulator or hand-rolled; no OpenCV dependency); left/right split: lines with negative slope + left half-image → left lane, positive slope + right half → right lane; polynomial fit via RANSAC least-squares on Hough line sample points
- [ ] `tests/test_lane_detection.cpp`:
  - Synthetic edge image (two known straight lanes): detected polynomials have coefficients within 5% of ground truth
  - Curved lanes (known quadratic): degree-2 polynomial fit within tolerance; curvature extracted correctly
  - Only left lane present: `right.validity = false`; left lane detected correctly; no crash
  - Pure noise edge image (random pixels): both lanes report `validity = false`; no false-positive lines
  - Very narrow lane gap (lanes close together): both lanes still separated correctly if Hough threshold is met
- [ ] Phase 4.5: `ILogger`, detected line count + polynomial fit residual at `DEBUG`, Hough accumulator build time at `TRACE`
- [ ] Sim note: lane_detection operates on pre-computed binary edge images; sim edge rendering deferred to post-M22 extension
- [ ] Frontend: render left/right lane polynomial curves overlaid on camera image

#### Exit Criteria

1. LQG drives double integrator to zero using position-only measurements; velocity estimate converges
2. Separation principle verified: independently-computed K and L match joint LQG construction
3. Lane detection: straight synthetic lanes detected with coefficient error < 5%
4. Curved lane: polynomial fit within tolerance; curvature correctly extracted
5. Missing lane: reports `validity = false` cleanly
6. Both modules hot-swappable in sim via REST
7. All unit tests pass, CI green
8. All modules pass Phase 4.5 — Observability gate

#### NOT IN

LQR servo tracking (integral action — → future), output-constrained LQG, H∞ robust control (→ future), lane change maneuver planning, lane-keeping PID wrapper (use the LQG as the controller), deep-learning lane detection (→ M23 semantic segmentation interface), OpenCV dependency.

---

### M23 — Lattice Planning & Semantic Vision

**Status:** Not Started  
**Dependencies:** M7 (`IGlobalPlanner` interface, planning infrastructure) for `lattice_planner`; M6 (`common/camera.hpp` image types) for `semantic_segmentation`.  
**Scope:** Two P9 modules. Lattice planner uses pre-computed motion primitives over a discrete state lattice for structure-environment planning. Semantic segmentation defines a clean stub interface for pixel-level scene understanding, with an explicit plugin point for a future DL backend. No DL framework dependency in M23.

#### Goal

Lattice planning completes the "structured-environment planning" chapter: where RRT/PRM search continuous C-space, the lattice planner exploits the regularity of structured environments (roads, corridors) by restricting motion to a pre-computed set of kinematically-feasible primitives over a discretized state lattice. This makes planning faster and produces smoother, more natural-looking trajectories for Ackermann vehicles. Semantic segmentation addresses the known P9 DL-dependency challenge via a deliberate stub-first design: M23 ships `ISemanticSegmenter` + `StubSemanticSegmenter` with a documented plugin interface for future ONNX/TensorRT. The interface design remains durable regardless of which DL backend is eventually adopted.

#### Modules

##### `motion_planning/global_planning/lattice_planner`

State lattice planner over a discrete `(x, y, θ)` grid with pre-computed kinematically-feasible motion primitives. Planning is a graph search (Dijkstra or A* with 2D Euclidean heuristic) over nodes connected by primitives.

- [ ] `include/lattice_planner/lattice_planner.hpp` — `LatticePlanner : IGlobalPlanner`
- [ ] `include/lattice_planner/lattice_planner_config.hpp` — `LatticePlannerConfig` (lattice_resolution, num_headings — discretized heading count, num_primitives_per_heading, max_primitive_length, use_astar flag)
- [ ] `include/lattice_planner/lattice_types.hpp` — `LatticeNode` (grid_x, grid_y, heading_idx), `MotionPrimitive` (sequence of `Pose2D` waypoints, cost), `LatticeGraph` (adjacency list)
- [ ] `src/lattice_planner.cpp` — primitive generation: for each heading, generate `num_primitives_per_heading` arcs (straight + left/right curves of varying radius) that connect to neighboring lattice nodes; graph construction: check each primitive for `OccupancyGrid` collision; A* / Dijkstra search on `LatticeGraph`
- [ ] `tests/test_lattice_planner.cpp`:
  - Straight corridor: planner finds straight path using zero-curvature primitives
  - 90° turn required: planner selects turning primitive; output path is kinematically feasible (heading continuity at each node)
  - Blocked corridor (one primitive set blocked by obstacle): planner routes around via alternate primitives
  - No path exists (all primitives to goal blocked): returns `std::nullopt`; no crash
  - Lattice resolution: coarser grid → faster planning (measured); longer output path (verify within 20% of fine-grid result)
  - Heading continuity: consecutive primitive start/end headings align within heading discretization tolerance
- [ ] Phase 4.5: `ILogger`, graph node/edge counts + A* expansion count at `DEBUG`, primitive generation time at `TRACE`
- [ ] Sim: selectable via `PUT /api/robot/global_planner {"type":"lattice"}`
- [ ] Frontend: render lattice graph (grey edges) + planned path (highlighted) + heading arrows at each node

##### `perception/semantic_segmentation`

Pixel-level scene classification with a stub implementation and explicit DL plugin interface. The module ships a complete, tested `ISemanticSegmenter` interface and `StubSemanticSegmenter`, plus a documented `SegmenterPlugin` (a `std::function<SemanticMap(const RgbImage&)>` injection point) that future ONNX/TensorRT backends will implement. No DL inference in M23 scope.

- [ ] `include/semantic_segmentation/semantic_segmenter.hpp` — `ISemanticSegmenter` (pure virtual: `std::expected<SemanticMap, std::string> segment(const RgbImage&)`); `StubSemanticSegmenter : ISemanticSegmenter` (returns all-`ROAD` map for valid image; used in tests and integration scaffolding)
- [ ] `include/semantic_segmentation/semantic_types.hpp` — `SemanticClass` enum (`ROAD, SIDEWALK, OBSTACLE, PERSON, VEHICLE, BUILDING, SKY, UNKNOWN`); `SemanticMap` (2D grid `Eigen::MatrixXi` or `std::vector<std::vector<SemanticClass>>`, same dimensions as input image, each cell = `SemanticClass` label)
- [ ] `include/semantic_segmentation/segmenter_plugin.hpp` — `SegmenterPlugin = std::function<std::expected<SemanticMap, std::string>(const RgbImage&)>`; `PluginSemanticSegmenter : ISemanticSegmenter` (wraps a `SegmenterPlugin`; enables runtime DL backend injection without recompilation)
- [ ] `src/stub_semantic_segmenter.cpp`
- [ ] `src/plugin_semantic_segmenter.cpp`
- [ ] `tests/test_semantic_segmentation.cpp`:
  - Stub: output `SemanticMap` has same dimensions as input image; all labels are valid `SemanticClass` values
  - Stub: empty image (0×0) → returns `std::expected` error; no crash
  - Plugin: custom lambda returning checkerboard `ROAD/OBSTACLE` map → invoked correctly; output matches lambda
  - Plugin: lambda throwing exception → error propagated as `std::expected` error string; no unhandled exception
  - Interface: both `StubSemanticSegmenter` and `PluginSemanticSegmenter` are assignable to `ISemanticSegmenter&`
- [ ] Phase 4.5: `ILogger`, image dimensions + inference mode at `DEBUG`
- [ ] Future-DL documentation: `docs/theory.md` must include a section titled "DL Backend Integration" documenting how to implement a `SegmenterPlugin` wrapping ONNX Runtime, the expected model input format (`HWC` float RGB, normalized), and the output format (class index map)
- [ ] Sim note: semantic overlay requires sim rendering extension; M23 scope covers offline/batch processing only

#### Exit Criteria

1. Lattice planner finds straight path; finds turning path around 90° turn
2. Heading continuity: consecutive primitives align at joining node (within heading discretization)
3. Lattice planner returns `std::nullopt` cleanly when no path exists
4. `ISemanticSegmenter` interface: both `StubSemanticSegmenter` and `PluginSemanticSegmenter` implement it correctly
5. Plugin injection: custom lambda invoked and result returned correctly
6. Invalid input (empty image): returns `std::expected` error; no crash
7. DL backend integration guide documented in `docs/theory.md`
8. All unit tests pass, CI green
9. All modules pass Phase 4.5 — Observability gate

#### NOT IN

Any DL framework (ONNX Runtime, TensorRT, LibTorch, OpenVINO) — all deferred to post-M23 DL backend milestone, real semantic inference (all inference is stub), lattice planner in 3D C-space, clothoid motion primitives (→ future), GPU-accelerated A* on lattice graph.

---

### M24 — Fleet Operations

**Status:** Not Started  
**Dependencies:** M12 (`BatteryManagement`, `FleetMonitor`, `TaskAllocation`, VDA 5050 modules completed; fleet infrastructure available).  
**Scope:** Single-module milestone. `ChargingStationManager` adds station registry, priority charge queue, and completion prediction to the fleet management layer established in M12.

#### Goal

M12 built out the fleet layer: task allocation, fleet monitoring, battery level tracking, and VDA 5050 messaging. One capability is missing: assigning robots to physical charging stations and managing queues when more robots need charging than stations are available. `ChargingStationManager` closes this gap. It is intentionally a focused solo milestone — the fleet domain has no natural P5–P10 partner, and forcing an unrelated pairing (e.g., with `noise_models`) would reduce clarity.

#### Modules

##### `fleet_management/charging_station`

Charging station registry and assignment manager. Tracks which stations are available, maintains a priority queue of robots awaiting charging (ordered by battery urgency), and provides predictive charge completion estimates based on a simple linear charge model.

- [ ] `include/charging_station/charging_station_manager.hpp` — `ChargingStationManager`
- [ ] `include/charging_station/charging_station_types.hpp` — `ChargingStation` (id `StationId`, position `Pose2D`, `max_power_kw`, status enum `IDLE / OCCUPIED / FAULT`); `ChargeRequest` (robot_id, battery_level_pct, urgency_flag); `AssignmentResult` (station_id, estimated_arrival_s, estimated_completion_s)
- [ ] `src/charging_station_manager.cpp` —
  - `registerStation(ChargingStation)` → void
  - `setFault(StationId)` / `clearFault(StationId)` — station health management
  - `requestCharging(ChargeRequest)` → `std::expected<AssignmentResult, std::string>` — assigns idle station if available, else enqueues; priority = urgency_flag first, then ascending battery_level_pct
  - `releaseStation(StationId)` → assigns next in queue if any
  - `predictCompletion(StationId)` → `std::optional<double>` — `t = capacity_kwh × (1 − soc) / power_kw` (linear model)
  - `getStatus()` → summary of all stations + queue depth
- [ ] `tests/test_charging_station_manager.cpp`:
  - 2 stations, 3 robots requesting simultaneously: first 2 assigned immediately; 3rd queued
  - Priority ordering: robot with `urgency = true` jumps ahead of lower-battery robot without urgency flag
  - `releaseStation()`: queued robot assigned automatically; assignment result returned to caller
  - Fault station: `FAULT` station not assigned; robots redirected to healthy stations
  - `predictCompletion()`: linear model matches hand-calculated time for known capacity / power / soc
  - Empty registry: `requestCharging()` → error via `std::expected`
  - No stations available + queue limit exceeded → documented cap behavior (return error)
- [ ] Phase 4.5: `ILogger`, assignment decisions + queue depth at `DEBUG`, prediction parameters at `TRACE`
- [ ] Sim: fleet REST endpoint `POST /api/fleet/charging/request` + `GET /api/fleet/charging/status`
- [ ] Frontend: charging station map overlay; queue depth indicator; charge completion countdown

#### Exit Criteria

1. 3-robot / 2-station scenario: correct assignment + queuing verified
2. Priority ordering: urgency flag pre-empts battery-level sort
3. Release-and-reassign: queued robot automatically assigned when station freed
4. Fault handling: FAULT stations excluded from assignment
5. Completion prediction matches hand-calculated linear model
6. All unit tests pass, CI green
7. All modules pass Phase 4.5 — Observability gate

#### NOT IN

Dynamic power allocation (variable charging rate), multi-robot convoy routing to stations, predictive charging scheduling (→ future), physical charger hardware interface, energy pricing model.

---

## Module → Milestone Mapping

| Module | Path | Milestone | Priority |
|--------|------|-----------|---------|
| `mppi` | `control/mppi` | M18 | P5 |
| `feedback_linearization` | `control/feedback_linearization` | M18 | P6 |
| `depth_camera` | `perception/depth_camera` | M19 | P6 |
| `object_detection_3d` | `perception/object_detection_3d` | M19 | P7 |
| `potential_field` | `motion_planning/local_planning/potential_field` | M20 | P5 |
| `informed_rrt_star` | `motion_planning/global_planning/informed_rrt_star` | M20 | P7 |
| `factor_graph` | `state_estimation/factor_graph` | M21 | P5 |
| `noise_models` | `common/noise_models` | M21 | P10 |
| `lqg` | `control/lqg` | M22 | P8 |
| `lane_detection` | `perception/lane_detection` | M22 | P8 |
| `lattice_planner` | `motion_planning/global_planning/lattice_planner` | M23 | P9 |
| `semantic_segmentation` | `perception/semantic_segmentation` | M23 | P9 |
| `charging_station` | `fleet_management/charging_station` | M24 | P10 |

---

## Updated Dependency Graph Additions

The following entries extend the existing graph in `repo-plans/README.md`.

```
M13 (Classical & Optimal Control)
 ├→ M18 (Advanced Nonlinear Control)   [mppi: IController; feedback_lin: IController + LQR patterns]
 └→ M22 (Optimal Output Feedback)      [lqg: LQRController + IController]

M6 (Visual Perception)
 ├→ M19 (Depth Perception & 3D)        [depth_camera: common/camera.hpp types]
 └→ M22 (Automotive Perception)        [lane_detection: image types]
       └→ M23 (Semantic Vision)         [semantic_segmentation: RgbImage type]

M4 (Perception Upgrades)
 ├→ M19 (3D Detection)                 [object_detection_3d: conceptual extension of obstacle_detection]
 └→ M20 (Planning Upgrades III)        [potential_field: OccupancyGrid; informed_rrt_star: OccupancyGrid]

M7 (Advanced Planning)
 ├→ M20 (Planning Upgrades III)        [informed_rrt_star: RRTStar + IGlobalPlanner]
 └→ M23 (Lattice Planning)             [lattice_planner: IGlobalPlanner]

M14 (Advanced State Estimation II)
 └→ M21 (Estimation Foundations)       [factor_graph: generalizes pose_graph GN optimizer patterns]

M12 (Fleet Management)
 └→ M24 (Fleet Operations)             [charging_station: fleet infrastructure]
```

### Parallel development opportunities (M18–M24)

After respective prerequisites are complete:

| Group | Milestones | Can run in parallel |
|-------|-----------|---------------------|
| Post-M13 | M18, M22 | Yes — both need M13; no cross-dependency |
| Post-M6+M4 | M19, M20 | Yes — both need M6/M4 infrastructure; no cross-dependency |
| Post-M14 vs post-M13 | M21, M22 | Yes — M21 needs M14; M22 needs M13; independent |
| Post-M7+M6 | M20, M23 | Partial — M20 needs M7; M23 needs M7+M6; M20 can start before M23 if M7 precedes M6 |
| Post-M12 | M24 | Fully independent of M18–M23 |

---

## Concerns: P9 Semantic Segmentation & the DL Dependency

### Problem

`perception/semantic_segmentation` requires pixel-level inference (e.g., per-pixel class label from a CNN). No viable path exists to implement this in pure C++20 with Eigen only at production quality — all practical implementations run through a DL inference runtime: ONNX Runtime, TensorRT, LibTorch (libtorch), or OpenVINO.

Adding **any** of these as a FetchContent dependency introduces:
- A 100–500 MB binary artifact (ONNX Runtime / TensorRT runtime binaries)
- GPU drivers / CUDA toolkit dependency (TensorRT, LibTorch CUDA)
- Non-trivial CMake complexity to detect and link the runtime
- Potentially breaking CI on CPU-only runners

### Resolution in M23

M23 adopts the **stub-only + plugin interface** approach:

1. `ISemanticSegmenter` defines the stable interface that *all* code consumes.
2. `StubSemanticSegmenter` provides a deterministic, always-valid output for tests and integration scaffolding.
3. `SegmenterPlugin = std::function<...>` is the documented injection point — no compile-time DL dependency.
4. `docs/theory.md` includes a "DL Backend Integration" section explaining exactly how to implement a `SegmenterPlugin` wrapping ONNX Runtime (model I/O format, normalization, output decode).

### Post-M23 path

A future milestone (e.g., "M25 — Deep Perception Backend") would:
- Make ONNX Runtime a conditional FetchContent dependency `if(ROBOTLIB_ENABLE_DL_BACKEND)`
- Implement `OnnxSemanticSegmenter : ISemanticSegmenter` behind the flag
- Ship a test with a tiny quantized semantic model (< 5 MB) for CI
- Gate the entire module with `if(ROBOTLIB_ENABLE_DL_BACKEND)` in CMakeLists

This design means **M23 is unconditionally completable** and the DL backend decision is decoupled from the interface design decision. The interface, once established in M23, remains stable across both the stub and any future DL implementation.

### Risk

If the chosen future DL runtime's inference API turns out to require a fundamentally different input representation than `RgbImage` (e.g., it expects a CUDA tensor, not a CPU `Eigen::MatrixXf`), the `SegmenterPlugin` injection point provides an adapter boundary — the plugin implementor handles the conversion, and `ISemanticSegmenter` is not changed. This is the primary reason the plugin type is `std::function<...>` rather than a virtual method on a base class: the std::function closure can capture any pre-processing or GPU upload logic without modifying the interface.
