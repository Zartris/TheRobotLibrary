Description and intent of the file:
This file is listing down tasks and features that we want before making it into a concrete milestone. There is two  list. The quick todo list is for us to quickly jot down ideas and tasks, while the fleshed out todo list is for us to have a more detailed view of what we want to do and how we want to do it.

---
Quick todo list:
---
Fleshed out todo list:

---
## Missing Modules Gap Analysis
*Reviewed 2026-03-13. Priority 1 = most critical, 10 = edge case / specialized.*

---

### ✅ [P1] visual_slam — Full Visual SLAM pipeline
- **Domain:** `state_estimation/visual_slam`
- **Resolution (2026-03-13):** `workspace/robotics/state_estimation/visual_slam/` pre-scaffolded; renamed old M6-slam → M6.5-slam; architecture note added (does not link `feature_extraction`/`visual_odometry` — pipeline assembled by caller via `common/` types); `repo-plans/modules/visual_slam.md` created; M6.5 updated with prerequisite note and replacement task bullets.

### ✅ [P1] common/transforms — Coordinate frame & rigid-body math
- **Domain:** `common/transforms`
- **Resolution (2026-03-13):** `workspace/robotics/common/transforms/` pre-scaffolded (header-only INTERFACE, SE2/SE3/SO3 + tests); `add_subdirectory(transforms)` wired in `common/CMakeLists.txt`; tasks added to `repo-plans/modules/common.md` under M1-A; M1 milestone table updated with the new sub-module row.

### [P2] control/lqr — Linear Quadratic Regulator
- **Domain:** `control/lqr`
- **Why missing:** LQR is the canonical optimal control baseline — teaches cost-function design, DARE solving, and state feedback. No milestone mentions it. It is the natural companion to MPC (same Q/R weighting intuition, simpler to implement).
- **Scope:**
  - [ ] Scaffold `workspace/robotics/control/lqr/`
  - [ ] `LQRController : IController` — discrete-time infinite-horizon LQR, solve DARE via Eigen
  - [ ] Support linearised kinematic model at operating point
  - [ ] Tests: regulator drives state to zero from perturbation; respects Q/R weighting
  - [ ] Add to M3 or create M3.5 (Control Optimal) milestone

### ✅ [P2] perception/visual_odometry — Visual front-end for camera-based localization
- **Domain:** `perception/visual_odometry`
- **Resolution (2026-03-13):** `workspace/robotics/perception/visual_odometry/` pre-scaffolded (8-point RANSAC + cheirality check); added to M6 Visual Perception Building Blocks milestone; `repo-plans/modules/visual_odometry.md` created; `CameraIntrinsics` sourced from `common/camera.hpp` (not redefined).

### ✅ [P2] perception/feature_extraction — ORB / keypoint feature pipeline
- **Domain:** `perception/feature_extraction`
- **Resolution (2026-03-13):** `workspace/robotics/perception/feature_extraction/` pre-scaffolded (FAST+BRIEF stub, `DescriptorMatcher` with Hamming distance via `__builtin_popcount`, ratio test); added to M6 Visual Perception Building Blocks milestone; `repo-plans/modules/feature_extraction.md` created.

### [P2] state_estimation/ukf — Unscented Kalman Filter
- **Domain:** `state_estimation/ukf`
- **Why missing:** The UKF is the natural upgrade path from EKF for strongly nonlinear systems (e.g., SE3 poses, IMU integration). M5 only adds a particle filter. UKF fills the gap between EKF and particle filter in accuracy vs. cost.
- **Scope:**
  - [ ] Scaffold `workspace/robotics/state_estimation/ukf/`
  - [ ] `UKF : IStateEstimator` — sigma-point propagation, Merwe scaled sigma points, additive noise
  - [ ] Generic: accepts process/measurement functions as `std::function` (same style as EKF)
  - [ ] Tests: same scenarios as EKF — nonlinear pendulum converges faster than EKF

### [P2] perception/imu_processing — IMU filtering and pre-integration
- **Domain:** `perception/imu_processing`
- **Why missing:** IMU is the most common sensor in robotics, yet no module handles gyro/accel bias estimation, low-pass filtering, or pre-integration. Required for visual-inertial SLAM and high-frequency state estimation.
- **Scope:**
  - [ ] Scaffold `workspace/robotics/perception/imu_processing/`
  - [ ] `ImuFilter` — complementary filter (accel + gyro fusion), bias estimation
  - [ ] `ImuPreintegrator` — IMU pre-integration between keyframes (for VIO)
  - [ ] `ImuTypes` — `ImuMeasurement`, `ImuBias`, `ImuState`
  - [ ] Tests: static IMU → bias converges; pre-integration matches numerical integration

### [P3] state_estimation/pose_graph — Pose graph optimization (loop closure backend)
- **Domain:** `state_estimation/pose_graph`
- **Why missing:** Both lidar SLAM and visual SLAM need a pose graph backend for loop closure. Currently the lidar_slam spec mentions "simple chain + loop closure" inline but there is no reusable pose graph optimizer. M6's NOT-IN list excludes GTSAM/g2o, so a lightweight internal implementation is needed.
- **Scope:**
  - [ ] Scaffold `workspace/robotics/state_estimation/pose_graph/`
  - [ ] `PoseGraph` — nodes (SE2 poses), edges (relative transforms + covariance)
  - [ ] Optimizer: Gauss-Newton or LM linearised graph optimization (Eigen-only)
  - [ ] Loop closure integration: add edge → re-optimize
  - [ ] Tests: 4-node square loop → optimization closes the loop; trajectory error < threshold

### [P3] motion_planning/global_planning/prm — Probabilistic Roadmap
- **Domain:** `motion_planning/global_planning/prm`
- **Why missing:** PRM is the canonical sampling-based planner for high-dimensional or complex configuration spaces — the pair to RRT in every robotics curriculum. M7 has Dijkstra + RRT* but no PRM.
- **Scope:**
  - [ ] Scaffold `workspace/robotics/motion_planning/global_planning/prm/`
  - [ ] `PRM` — random sampling in C-space, local planner (straight-line collision check), k-nearest graph, query phase via Dijkstra/A*
  - [ ] Multi-query reuse: build roadmap once, answer multiple queries
  - [ ] Tests: cluttered environment → path found within N samples; roadmap reused for second query

### [P3] state_estimation/visual_inertial_odometry — VIO (IMU + camera fusion)
- **Domain:** `state_estimation/visual_inertial_odometry`
- **Why missing:** VIO is the gold standard for drift-free odometry on mobile robots without GPS. Requires both `imu_processing` (P2) and `visual_odometry` (P2) as building blocks — natural M6 extension for a visual-inertial variant.
- **Scope:**
  - [ ] Scaffold `workspace/robotics/state_estimation/visual_inertial_odometry/`
  - [ ] Loosely-coupled VIO: EKF/UKF fusing IMU pre-integration + VO pose updates
  - [ ] `VIOState` — pose, velocity, IMU bias
  - [ ] Tests: simulated IMU + camera sequence → trajectory error < pure VO drift

### [P4] control/stanley — Stanley controller
- **Domain:** `control/stanley`
- **Why missing:** Stanley is the path-tracking controller from Stanford's DARPA challenge entry — widely referenced, Ackermann-friendly, trivial to implement. A clear gap next to pure_pursuit and MPC.
- **Scope:**
  - [ ] Scaffold `workspace/robotics/control/stanley/`
  - [ ] `StanleyController : IController` — heading error + cross-track error, speed-normalized gain
  - [ ] Tests: straight path → converges; curved path → tracks within tolerance; reversing supported

### [P4] motion_planning/trajectory_planning/polynomial — Polynomial / Bézier trajectory generation
- **Domain:** `motion_planning/trajectory_planning/polynomial`
- **Why missing:** Spline fitting exists but minimum-snap / minimum-jerk polynomial trajectories (used in drone and arm motion planning) are absent. Natural companion to spline_fitting.
- **Scope:**
  - [ ] Scaffold `workspace/robotics/motion_planning/trajectory_planning/polynomial/`
  - [ ] Minimum-jerk 1D polynomial segments, minimum-snap (4th derivative) for drones
  - [ ] Waypoint-constrained polynomial generation
  - [ ] Bézier curve variant with convex hull property
  - [ ] Tests: 3-waypoint trajectory → smooth, satisfies endpoint derivatives

### [P4] perception/place_recognition — Loop closure / place recognition
- **Domain:** `perception/place_recognition`
- **Why missing:** Loop closure is essential for drift correction in SLAM. Currently folded into `visual_slam` spec as "bag-of-words" but warrants its own module for reuse by both lidar and visual SLAM backends.
- **Scope:**
  - [ ] Scaffold `workspace/robotics/perception/place_recognition/`
  - [ ] `PlaceRecognizer` — descriptor-based place database, query by similarity score
  - [ ] Supports both lidar scan descriptors and visual feature descriptors
  - [ ] Tests: 10-frame database → correct revisit detection; false positive rate < threshold

### [P4] perception/stereo_depth — Stereo disparity / depth estimation
- **Domain:** `perception/stereo_depth`
- **Why missing:** Stereo cameras are the primary depth sensor for visual SLAM (no IR pattern, works outdoors). Needed for scale-resolved visual odometry and RGB-D SLAM. Complements the camera sim infrastructure planned in M6.
- **Scope:**
  - [ ] Scaffold `workspace/robotics/perception/stereo_depth/`
  - [ ] `StereoMatcher` — block matching or semi-global matching (SGM), disparity → depth map
  - [ ] `StereoRectifier` — apply calibration to undistort + rectify stereo pair
  - [ ] Tests: synthetic stereo pair with known disparity → depth error < tolerance

### [P5] control/mppi — Model Predictive Path Integral
- **Domain:** `control/mppi`
- **Why missing:** MPPI is the modern stochastic MPC variant — handles non-convex costs, used in aggressive autonomous driving. Academic hotspot. No mention in any milestone.
- **Scope:**
  - [ ] Scaffold `workspace/robotics/control/mppi/`
  - [ ] `MPPIController : IController` — Monte Carlo rollouts, importance-weighted cost averaging
  - [ ] Configurable: N rollouts, horizon H, temperature λ, noise covariance Σ
  - [ ] Tests: navigate around obstacle → sampled distribution clusters around valid trajectories

### [P5] motion_planning/local_planning/potential_field — Artificial potential field planner
- **Domain:** `motion_planning/local_planning/potential_field`
- **Why missing:** Potential fields are the simplest reactive planner and a key teaching tool (attractive + repulsive field, local minima, oscillation). Not covered by DWA or TEB.
- **Scope:**
  - [ ] Scaffold `workspace/robotics/motion_planning/local_planning/potential_field/`
  - [ ] `PotentialFieldPlanner` — goal attractive field + obstacle repulsive fields
  - [ ] Local minima escape: random perturbation or wavefront fallback
  - [ ] Tests: clear path → reaches goal; obstacle → deflects; local minimum → escape triggered

### [P5] state_estimation/factor_graph — Factor graph SLAM backend (lightweight g2o-style)
- **Domain:** `state_estimation/factor_graph`
- **Why missing:** M6's NOT-IN explicitly excludes GTSAM/g2o but long-term a reusable factor graph (Eigen-only) enables consistent backends for lidar SLAM, visual SLAM, and VIO. Foundational for M13+ if the library grows.
- **Scope:**
  - [ ] Scaffold `workspace/robotics/state_estimation/factor_graph/`
  - [ ] Variable nodes (pose, landmark, bias) + factor types (odometry, observation, prior)
  - [ ] Gauss-Newton optimizer with sparse Cholesky (Eigen)
  - [ ] Tests: 2-node odometry chain → correct; 4-node loop closure → optimized

### [P6] perception/depth_camera — RGB-D / structured light camera model
- **Domain:** `perception/depth_camera`
- **Why missing:** RGB-D cameras (Kinect, RealSense) are ubiquitous but there is no module for handling their output (depth image → point cloud, invalid pixel filtering, hole filling). Required for RGB-D SLAM.
- **Scope:**
  - [ ] Scaffold `workspace/robotics/perception/depth_camera/`
  - [ ] `DepthImageProcessor` — depth → point cloud, hole filling, outlier removal, frustum filtering
  - [ ] `RgbdCamera` type wrapping RGB + aligned depth
  - [ ] Tests: depth image with holes → filled output; known depth → correct point cloud

### [P6] control/feedback_linearization — Nonlinear control via state/input transformation
- **Domain:** `control/feedback_linearization`
- **Why missing:** Feedback linearization (exact linearization via diffeomorphism) is a key nonlinear control technique taught in every advanced robotics course. Pairs well with LQR (apply LQR to linearized system).
- **Scope:**
  - [ ] Scaffold `workspace/robotics/control/feedback_linearization/`
  - [ ] Input-output linearization for differential-drive (chained form)
  - [ ] Tests: nonlinear system tracks linear reference; numerical check of Lie derivatives

### [P7] perception/object_detection_3d — 3D bounding box detection from point clouds
- **Domain:** `perception/object_detection_3d`
- **Why missing:** `obstacle_detection` (M4) does 2D DBSCAN clustering. True 3D object detection (oriented bounding boxes from 3D lidar) is absent and needed for environments with varied-height obstacles.
- **Scope:**
  - [ ] `ObjectDetector3D` — 3D DBSCAN + oriented bounding box fitting (PCA-based)
  - [ ] Object classification placeholder (size-based: person / car / unknown)

### [P7] motion_planning/global_planning/informed_rrt_star — Informed RRT*
- **Domain:** `motion_planning/global_planning/informed_rrt_star`
- **Why missing:** Informed RRT* focuses sampling inside the ellipsoidal heuristic region once a solution is found — orders-of-magnitude faster convergence than vanilla RRT*. Natural incremental upgrade.
- **Scope:**
  - [ ] `InformedRRTStar` — extends RRT* with prolate hyperspheroid sampling after first solution

### [P8] perception/lane_detection — Lane marking extraction (automotive / structured environments)
- **Domain:** `perception/lane_detection`
- **Why missing:** Useful for autonomous vehicle scenarios (Ackermann kinematics + lane following), but very application-specific.
- **Scope:**
  - [ ] `LaneDetector` — Hough transform on binary edge image, lane polynomial fitting, left/right lane assignment

### [P8] control/lqg — Linear Quadratic Gaussian (LQR + Kalman observer)
- **Domain:** `control/lqg`
- **Why missing:** LQG = LQR + Kalman filter — the complete optimal output-feedback controller. Natural extension once LQR (P2) is implemented.
- **Scope:**
  - [ ] `LQGController : IController` — LQR state feedback + internal EKF state observer
  - [ ] Tests: output-only measurements → converges; separation principle holds

### [P9] perception/semantic_segmentation — Pixel-level scene classification
- **Domain:** `perception/semantic_segmentation`
- **Why missing:** Semantic understanding (free space / obstacle / road) from camera. Computationally heavy; out of scope for a C++ CPU library without GPU/DL framework but worth tracking.
- **Note:** Likely requires a DL backend (ONNX Runtime or TensorRT) — dependency decision needed.

### [P9] motion_planning/global_planning/lattice_planner — State lattice planner
- **Domain:** `motion_planning/global_planning/lattice_planner`
- **Why missing:** Lattice planners generate pre-computed motion primitives over a discrete state lattice — very effective for structured environments and Ackermann vehicles. More principled than RRT for on-road driving.
- **Scope:**
  - [ ] `LatticePlanner` — pre-computed motion primitives + graph search (Dijkstra/A*)

### [P10] fleet_management/charging_station — Charging station assignment and queuing
- **Domain:** `fleet_management/charging_station`
- **Why missing:** `battery_management` tracks battery state but there is no module for assigning robots to charging stations, managing queues, or predicting charge completion time.
- **Scope:**
  - [ ] `ChargingStationManager` — station registry, charge queue, priority-based assignment

### [P10] common/noise_models — Sensor noise and perturbation models
- **Domain:** `common/noise_models`
- **Why missing:** Gaussian noise, outliers, and systematic bias are generated ad hoc in tests. A shared noise model library keeps tests reproducible and matches real sensor specs.
- **Scope:**
  - [ ] `GaussianNoise<T>`, `UniformNoise<T>`, `OutlierInjector` — seeded RNG wrappers

---

### ✅ Fleet management scaffold + M12 milestone
- [x] Add fleet management as a module (just the scaffolding), which will include features such as:
  - [x] Standard protocols like VDA 5050.
  - [x] Fleet management features such as task assignment, monitoring, and battery management.
- [x] Add milestone for populating the fleet management module with features and protocols.
- **Resolution:** `workspace/robotics/fleet_management/` scaffolded (vda5050, task_allocation, fleet_monitor, battery_management); added to M0 exit criteria; M12 milestone created (depends on M8). Fleet REST endpoints added to `workspace/architecture.md`. Brainstorm design doc at `.github/superpower/brainstorm/2026-03-13-robot-library-expansions-design.md`.

### ✅ Advanced control algorithms + M11 milestone
- [x] Add more to the control section, such as:
  - [x] Adaptive Control: Learning in Real Time → `control/adaptive/gain_scheduling/` + `control/adaptive/rls/`
  - [x] Control Barrier Functions for Safety → `control/cbf/`
  - [x] Frenet-Serret Frame Controller for Path Following → `control/frenet/`
- **Resolution:** CBF added to M3 (safety filter decorator); M11 milestone created for adaptive + Frenet (depends on M3). All scaffold files created with theory docs.

### ✅ Agent/Developer Observability
- [x] Focus on both human and agent feedback — everything observable through logging and metrics.
  - [x] Add observability criteria to all milestones.
- **Resolution:** Phase 4.5 (Observability) inserted into `module-task-template.md`; `ILogger` / `SpdlogLogger` scaffolded under `common/logging/`; observability exit criterion added to every milestone (M0–M12).
