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
- **✅ Resolution (2026-03-14):** Scaffolded in M13 — Classical & Optimal Control. `workspace/robotics/control/lqr/` created; `repo-plans/modules/lqr.md` created; wired into `control/CMakeLists.txt`.

### ✅ [P2] perception/visual_odometry — Visual front-end for camera-based localization
- **Domain:** `perception/visual_odometry`
- **Resolution (2026-03-13):** `workspace/robotics/perception/visual_odometry/` pre-scaffolded (8-point RANSAC + cheirality check); added to M6 Visual Perception Building Blocks milestone; `repo-plans/modules/visual_odometry.md` created; `CameraIntrinsics` sourced from `common/camera.hpp` (not redefined).

### ✅ [P2] perception/feature_extraction — ORB / keypoint feature pipeline
- **Domain:** `perception/feature_extraction`
- **Resolution (2026-03-13):** `workspace/robotics/perception/feature_extraction/` pre-scaffolded (FAST+BRIEF stub, `DescriptorMatcher` with Hamming distance via `__builtin_popcount`, ratio test); added to M6 Visual Perception Building Blocks milestone; `repo-plans/modules/feature_extraction.md` created.

### ✅ Vehicle Dynamics — Dynamic model, motor, terrain, param ID
- **Domain:** `control/vehicle_dynamics`, `control/motor_model`, `control/param_estimation`, `common/robot/`, `common/environment/`
- **✅ Resolution (2026-03-16):** M3.5 milestone created. `BicycleDynamicModel : IDynamicModel` (linear tire, Newton-Euler 2D), `DcMotorModel` + `ActuatorLimiter`, `StepResponseFitter`/`CircleTestFitter`/`CogEstimator` for offline calibration. Shared types in `common/robot/` (VehicleParams, MotorParams, TireParams) and `common/environment/` (TerrainProperties, SlipDetector). TerrainMap owned by simulation. Theory doc covers full spectrum (Pacejka, Lagrangian, MuJoCo upgrade path).

### [P2] state_estimation/ukf — Unscented Kalman Filter
- **Domain:** `state_estimation/ukf`
- **Why missing:** The UKF is the natural upgrade path from EKF for strongly nonlinear systems (e.g., SE3 poses, IMU integration). M5 only adds a particle filter. UKF fills the gap between EKF and particle filter in accuracy vs. cost.
- **Scope:**
  - [ ] Scaffold `workspace/robotics/state_estimation/ukf/`
  - [ ] `UKF : IStateEstimator` — sigma-point propagation, Merwe scaled sigma points, additive noise
  - [ ] Generic: accepts process/measurement functions as `std::function` (same style as EKF)
  - [ ] Tests: same scenarios as EKF — nonlinear pendulum converges faster than EKF
- **✅ Resolution (2026-03-14):** Scaffolded in M14 — Advanced State Estimation II. `workspace/robotics/state_estimation/ukf/` created; `repo-plans/modules/ukf.md` created; wired into `state_estimation/CMakeLists.txt`.

### [P2] perception/imu_processing — IMU filtering and pre-integration
- **Domain:** `perception/imu_processing`
- **Why missing:** IMU is the most common sensor in robotics, yet no module handles gyro/accel bias estimation, low-pass filtering, or pre-integration. Required for visual-inertial SLAM and high-frequency state estimation.
- **Scope:**
  - [ ] Scaffold `workspace/robotics/perception/imu_processing/`
  - [ ] `ImuFilter` — complementary filter (accel + gyro fusion), bias estimation
  - [ ] `ImuPreintegrator` — IMU pre-integration between keyframes (for VIO)
  - [ ] `ImuTypes` — `ImuMeasurement`, `ImuBias`, `ImuState`
  - [ ] Tests: static IMU → bias converges; pre-integration matches numerical integration
- **✅ Resolution (2026-03-14):** Scaffolded in M15 — Visual-Inertial Odometry. `workspace/robotics/perception/imu_processing/` created; `repo-plans/modules/imu_processing.md` created; wired into `perception/CMakeLists.txt`.

### [P3] state_estimation/pose_graph — Pose graph optimization (loop closure backend)
- **Domain:** `state_estimation/pose_graph`
- **Why missing:** Both lidar SLAM and visual SLAM need a pose graph backend for loop closure. Currently the lidar_slam spec mentions "simple chain + loop closure" inline but there is no reusable pose graph optimizer. M6's NOT-IN list excludes GTSAM/g2o, so a lightweight internal implementation is needed.
- **Scope:**
  - [ ] Scaffold `workspace/robotics/state_estimation/pose_graph/`
  - [ ] `PoseGraph` — nodes (SE2 poses), edges (relative transforms + covariance)
  - [ ] Optimizer: Gauss-Newton or LM linearised graph optimization (Eigen-only)
  - [ ] Loop closure integration: add edge → re-optimize
  - [ ] Tests: 4-node square loop → optimization closes the loop; trajectory error < threshold
- **✅ Resolution (2026-03-14):** Scaffolded in M14 — Advanced State Estimation II. `workspace/robotics/state_estimation/pose_graph/` created; `repo-plans/modules/pose_graph.md` created; wired into `state_estimation/CMakeLists.txt`.

### [P3] motion_planning/global_planning/prm — Probabilistic Roadmap
- **Domain:** `motion_planning/global_planning/prm`
- **Why missing:** PRM is the canonical sampling-based planner for high-dimensional or complex configuration spaces — the pair to RRT in every robotics curriculum. M7 has Dijkstra + RRT* but no PRM.
- **Scope:**
  - [ ] Scaffold `workspace/robotics/motion_planning/global_planning/prm/`
  - [ ] `PRM` — random sampling in C-space, local planner (straight-line collision check), k-nearest graph, query phase via Dijkstra/A*
  - [ ] Multi-query reuse: build roadmap once, answer multiple queries
  - [ ] Tests: cluttered environment → path found within N samples; roadmap reused for second query
- **✅ Resolution (2026-03-14):** Scaffolded in M16 — Planning Upgrades II. `workspace/robotics/motion_planning/global_planning/prm/` created; `repo-plans/modules/prm.md` created; wired into `global_planning/CMakeLists.txt`.

### [P3] state_estimation/visual_inertial_odometry — VIO (IMU + camera fusion)
- **Domain:** `state_estimation/visual_inertial_odometry`
- **Why missing:** VIO is the gold standard for drift-free odometry on mobile robots without GPS. Requires both `imu_processing` (P2) and `visual_odometry` (P2) as building blocks — natural M6 extension for a visual-inertial variant.
- **Scope:**
  - [ ] Scaffold `workspace/robotics/state_estimation/visual_inertial_odometry/`
  - [ ] Loosely-coupled VIO: EKF/UKF fusing IMU pre-integration + VO pose updates
  - [ ] `VIOState` — pose, velocity, IMU bias
  - [ ] Tests: simulated IMU + camera sequence → trajectory error < pure VO drift
- **✅ Resolution (2026-03-14):** Scaffolded in M15 — Visual-Inertial Odometry. `workspace/robotics/state_estimation/visual_inertial_odometry/` created; `repo-plans/modules/visual_inertial_odometry.md` created; wired into `state_estimation/CMakeLists.txt`.

### [P4] control/stanley — Stanley controller
- **Domain:** `control/stanley`
- **Why missing:** Stanley is the path-tracking controller from Stanford's DARPA challenge entry — widely referenced, Ackermann-friendly, trivial to implement. A clear gap next to pure_pursuit and MPC.
- **Scope:**
  - [ ] Scaffold `workspace/robotics/control/stanley/`
  - [ ] `StanleyController : IController` — heading error + cross-track error, speed-normalized gain
  - [ ] Tests: straight path → converges; curved path → tracks within tolerance; reversing supported
- **✅ Resolution (2026-03-14):** Scaffolded in M13 — Classical & Optimal Control. `workspace/robotics/control/stanley/` created; `repo-plans/modules/stanley.md` created; wired into `control/CMakeLists.txt`.

### [P4] motion_planning/trajectory_planning/polynomial — Polynomial / Bézier trajectory generation
- **Domain:** `motion_planning/trajectory_planning/polynomial`
- **Why missing:** Spline fitting exists but minimum-snap / minimum-jerk polynomial trajectories (used in drone and arm motion planning) are absent. Natural companion to spline_fitting.
- **Scope:**
  - [ ] Scaffold `workspace/robotics/motion_planning/trajectory_planning/polynomial/`
  - [ ] Minimum-jerk 1D polynomial segments, minimum-snap (4th derivative) for drones
  - [ ] Waypoint-constrained polynomial generation
  - [ ] Bézier curve variant with convex hull property
  - [ ] Tests: 3-waypoint trajectory → smooth, satisfies endpoint derivatives
- **✅ Resolution (2026-03-14):** Scaffolded in M16 — Planning Upgrades II. `workspace/robotics/motion_planning/trajectory_planning/polynomial/` created; `repo-plans/modules/polynomial.md` created; wired into `trajectory_planning/CMakeLists.txt`.

### [P4] perception/place_recognition — Loop closure / place recognition
- **Domain:** `perception/place_recognition`
- **Why missing:** Loop closure is essential for drift correction in SLAM. Currently folded into `visual_slam` spec as "bag-of-words" but warrants its own module for reuse by both lidar and visual SLAM backends.
- **Scope:**
  - [ ] Scaffold `workspace/robotics/perception/place_recognition/`
  - [ ] `PlaceRecognizer` — descriptor-based place database, query by similarity score
  - [ ] Supports both lidar scan descriptors and visual feature descriptors
  - [ ] Tests: 10-frame database → correct revisit detection; false positive rate < threshold
- **✅ Resolution (2026-03-14):** Scaffolded in M17 — Camera Perception II. `workspace/robotics/perception/place_recognition/` created; `repo-plans/modules/place_recognition.md` created; wired into `perception/CMakeLists.txt`.

### [P4] perception/stereo_depth — Stereo disparity / depth estimation
- **Domain:** `perception/stereo_depth`
- **Why missing:** Stereo cameras are the primary depth sensor for visual SLAM (no IR pattern, works outdoors). Needed for scale-resolved visual odometry and RGB-D SLAM. Complements the camera sim infrastructure planned in M6.
- **Scope:**
  - [ ] Scaffold `workspace/robotics/perception/stereo_depth/`
  - [ ] `StereoMatcher` — block matching or semi-global matching (SGM), disparity → depth map
  - [ ] `StereoRectifier` — apply calibration to undistort + rectify stereo pair
  - [ ] Tests: synthetic stereo pair with known disparity → depth error < tolerance
- **✅ Resolution (2026-03-14):** Scaffolded in M17 — Camera Perception II. `workspace/robotics/perception/stereo_depth/` created; `repo-plans/modules/stereo_depth.md` created; wired into `perception/CMakeLists.txt`.

### ✅ [P5] control/mppi — Model Predictive Path Integral
- **Domain:** `control/mppi`
- **✅ Resolution (2026-03-14):** Scaffolded in M18 — Advanced Nonlinear Control. `workspace/robotics/control/mppi/` created; `repo-plans/modules/mppi.md` created; wired into `control/CMakeLists.txt`.

### ✅ [P5] motion_planning/local_planning/potential_field — Artificial potential field planner
- **Domain:** `motion_planning/local_planning/potential_field`
- **✅ Resolution (2026-03-14):** Scaffolded in M20 — Planning Upgrades III. `workspace/robotics/motion_planning/local_planning/potential_field/` created; `repo-plans/modules/potential_field.md` created; wired into `local_planning/CMakeLists.txt`.

### ✅ [P5] state_estimation/factor_graph — Factor graph SLAM backend (lightweight g2o-style)
- **Domain:** `state_estimation/factor_graph`
- **✅ Resolution (2026-03-14):** Scaffolded in M21 — Estimation & Test Foundations. `workspace/robotics/state_estimation/factor_graph/` created; `repo-plans/modules/factor_graph.md` created; wired into `state_estimation/CMakeLists.txt`.

### ✅ [P6] perception/depth_camera — RGB-D / structured light camera model
- **Domain:** `perception/depth_camera`
- **✅ Resolution (2026-03-14):** Scaffolded in M19 — Depth Perception & 3D Understanding. `workspace/robotics/perception/depth_camera/` created; `repo-plans/modules/depth_camera.md` created; wired into `perception/CMakeLists.txt`.

### ✅ [P6] control/feedback_linearization — Nonlinear control via state/input transformation
- **Domain:** `control/feedback_linearization`
- **✅ Resolution (2026-03-14):** Scaffolded in M18 — Advanced Nonlinear Control. `workspace/robotics/control/feedback_linearization/` created; `repo-plans/modules/feedback_linearization.md` created; wired into `control/CMakeLists.txt`.

### ✅ [P7] perception/object_detection_3d — 3D bounding box detection from point clouds
- **Domain:** `perception/object_detection_3d`
- **✅ Resolution (2026-03-14):** Scaffolded in M19 — Depth Perception & 3D Understanding. `workspace/robotics/perception/object_detection_3d/` created; `repo-plans/modules/object_detection_3d.md` created; wired into `perception/CMakeLists.txt`.

### ✅ [P7] motion_planning/global_planning/informed_rrt_star — Informed RRT*
- **Domain:** `motion_planning/global_planning/informed_rrt_star`
- **✅ Resolution (2026-03-14):** Scaffolded in M20 — Planning Upgrades III. `workspace/robotics/motion_planning/global_planning/informed_rrt_star/` created; `repo-plans/modules/informed_rrt_star.md` created; wired into `global_planning/CMakeLists.txt`.

### ✅ [P8] perception/lane_detection — Lane marking extraction (automotive / structured environments)
- **Domain:** `perception/lane_detection`
- **✅ Resolution (2026-03-14):** Scaffolded in M22 — Optimal Output Feedback & Automotive Perception. `workspace/robotics/perception/lane_detection/` created; `repo-plans/modules/lane_detection.md` created; wired into `perception/CMakeLists.txt`.

### ✅ [P8] control/lqg — Linear Quadratic Gaussian (LQR + Kalman observer)
- **Domain:** `control/lqg`
- **✅ Resolution (2026-03-14):** Scaffolded in M22 — Optimal Output Feedback & Automotive Perception. `workspace/robotics/control/lqg/` created; `repo-plans/modules/lqg.md` created; wired into `control/CMakeLists.txt`.

### ✅ [P9] perception/semantic_segmentation — Pixel-level scene classification
- **Domain:** `perception/semantic_segmentation`
- **✅ Resolution (2026-03-14):** Scaffolded in M23 — Lattice Planning & Semantic Vision. Stub-only scope (no DL inference); `ISemanticSegmenter` + `StubSemanticSegmenter` + `PluginSemanticSegmenter` (std::function DL plugin point). `workspace/robotics/perception/semantic_segmentation/` created; `repo-plans/modules/semantic_segmentation.md` created; wired into `perception/CMakeLists.txt`.

### ✅ [P9] motion_planning/global_planning/lattice_planner — State lattice planner
- **Domain:** `motion_planning/global_planning/lattice_planner`
- **✅ Resolution (2026-03-14):** Scaffolded in M23 — Lattice Planning & Semantic Vision. `workspace/robotics/motion_planning/global_planning/lattice_planner/` created; `repo-plans/modules/lattice_planner.md` created; wired into `global_planning/CMakeLists.txt`.

### ✅ [P10] fleet_management/charging_station — Charging station assignment and queuing
- **Domain:** `fleet_management/charging_station`
- **✅ Resolution (2026-03-14):** Scaffolded in M24 — Fleet Operations. `workspace/robotics/fleet_management/charging_station/` created; `repo-plans/modules/charging_station.md` created; wired into `fleet_management/CMakeLists.txt`.

### ✅ [P10] common/noise_models — Sensor noise and perturbation models
- **Domain:** `common/noise_models`
- **✅ Resolution (2026-03-14):** Scaffolded in M21 — Estimation & Test Foundations. Header-only INTERFACE library. `workspace/robotics/common/noise_models/` created; `repo-plans/modules/noise_models.md` created; wired into `common/CMakeLists.txt`.

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
