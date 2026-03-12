# M6 — SLAM

**Status:** Not Started  
**Dependencies:** M4 (perception pipeline), M5 (state estimation interfaces)  
**Scope:** Simultaneous Localization and Mapping. Three approaches: landmark-based EKF-SLAM, scan-matching lidar SLAM, and camera-based visual SLAM.

---

## Goal

The robot can explore an **unknown** environment — simultaneously building a map and localizing within it. This is the capstone of single-robot autonomy: no pre-known map required.

---

## Modules

### ekf_slam

Augmented-state EKF-SLAM. Joint state vector: robot pose + landmark positions.

- [ ] `include/ekf_slam/ekf_slam.hpp` — `EKFSLAM` (new interface or extends `IStateEstimator`)
- [ ] `src/ekf_slam.cpp`
- [ ] Data association: nearest-neighbor with Mahalanobis distance gating
- [ ] Landmark initialization: first observation creates new landmark
- [ ] `tests/test_ekf_slam.cpp`:
  - Single landmark: predict + observe → landmark position converges
  - Multiple landmarks (10): all converge, cross-correlations correct
  - Data association: correctly matches re-observed landmark
  - Unknown landmark: creates new entry in state vector
  - Scales to ~100 landmarks without divergence

### lidar_slam

Scan-matching SLAM. Frame-to-frame alignment + simple pose-graph backend.

- [ ] `include/lidar_slam/lidar_slam.hpp` — `LidarSLAM`
- [ ] `src/lidar_slam.cpp`
- [ ] Scan matching: ICP (Iterative Closest Point)
- [ ] Pose graph: simple chain of relative transforms + loop closure detection via scan similarity
- [ ] Map output: stitched occupancy grid from aligned scans
- [ ] `tests/test_lidar_slam.cpp`:
  - Two overlapping scans with known transform → ICP recovers transform within tolerance
  - Sequence of 10 scans → trajectory drift bounded
  - Loop closure: detect revisited area, correct drift
  - Output map matches ground truth layout

---

## Sim & Frontend Integration (Lidar SLAM)

- [ ] New sim scenarios: unknown environment (no pre-loaded map, robot starts in blank grid)
- [ ] SLAM modules receive raw scans and build their own map representation
- [ ] Frontend: SLAM landmark map overlay (EKF-SLAM), scan-match trajectory trace (lidar SLAM)
- [ ] Frontend: toggle ground-truth map vs SLAM-built map for comparison
- [ ] Mini-demo: robot explores unknown room, builds map, navigates to goal on self-built map

---

## Camera Simulation & Visual SLAM

This is a major extension requiring a **camera sensor simulation** in addition to the existing lidar.

### Camera Simulation Infrastructure (prerequisite)

Simulating a camera in a 2D/2.5D environment. Options to evaluate during implementation:

1. **Projective landmark camera** — project known 3D landmark positions into a virtual pinhole camera → observed pixel coordinates + descriptors. Simplest; works for feature-based VSLAM.
2. **Depth rendering via ray-casting** — cast dense rays from camera pose → depth image. Extends existing ray_casting module. Suitable for depth-based VO.
3. **Simple 3D raycaster** — extend world model with height information (2.5D walls), render a basic RGB or depth image from first-person perspective. Most realistic but most complex.

- [ ] `include/common/camera.hpp` — `CameraIntrinsics` (fx, fy, cx, cy, width, height), `CameraImage` (grayscale/depth)
- [ ] `include/simulation/camera_sensor.hpp` — simulated camera: projects landmarks → pixel observations (option 1), or renders depth from 2.5D world (option 3)
- [ ] Simulated camera noise: pixel noise, missed detections, false positives
- [ ] Scenario JSON: `"camera": {"fov": 90, "resolution": [320, 240], "max_range": 10}`
- [ ] Camera data in WebSocket state stream (compact: feature list or downsampled depth)

### visual_slam

Feature-based visual SLAM. Monocular or RGB-D depending on camera sim chosen.

- [ ] `include/visual_slam/visual_slam.hpp` — `VisualSLAM : ISlamEstimator`
- [ ] `src/visual_slam.cpp`
- [ ] Feature extraction: ORB features (implement or lightweight wrapper)
- [ ] Feature matching: brute-force or FLANN-like nearest-neighbor
- [ ] Visual odometry: essential matrix → relative pose from matched features
- [ ] Map: 3D landmark triangulation from feature tracks (or 2D projection if depth available)
- [ ] Loop closure: bag-of-words or feature-similarity based revisit detection
- [ ] `tests/test_visual_slam.cpp`:
  - Known camera + known landmarks → correct pixel projection
  - 2-frame VO: features match → recovered relative pose within tolerance
  - 10-frame sequence → trajectory drift bounded
  - Loop closure: detect revisited area, correct drift
  - Landmark map approximately matches ground truth

### Sim & Frontend Integration (Visual SLAM)

- [ ] Frontend: camera view panel (what the robot "sees")
- [ ] Frontend: feature tracks overlay on camera view
- [ ] Frontend: visual SLAM landmark map (3D points projected onto 2D top-down view)
- [ ] Mini-demo: robot explores with camera, builds feature map, closes loop

---

## Deliverables

- [ ] ekf_slam module: implementation + tests
- [ ] lidar_slam module: implementation + tests
- [ ] Camera simulation infrastructure (common types + sim sensor)
- [ ] visual_slam module: implementation + tests
- [ ] Unknown-map scenario(s) in sim
- [ ] Frontend SLAM visualization (all three SLAM variants)
- [ ] Demo: explore unknown environment, build map, navigate

## Exit Criteria

1. Robot explores and builds a map recognizably matching ground truth
2. EKF-SLAM handles ≥ 50 landmarks without divergence
3. Lidar-SLAM trajectory error ≤ 0.5m over 100m traverse
4. Visual SLAM tracks features and builds consistent landmark map
5. All unit tests pass

## NOT IN

NDT, advanced graph optimization libraries (GTSAM/g2o for pose graph), multi-robot SLAM, full 3D SLAM, deep-learning-based features.

## Open Questions

- **Camera sim approach:** Projective landmarks (simplest) vs 2.5D raycaster (most realistic)? Decide during implementation based on effort vs educational value. Can start with projective landmarks and upgrade later.
- **Feature extraction library:** Implement ORB from scratch (educational) vs use OpenCV (practical)? If OpenCV, it becomes a new dependency. Consider a minimal standalone ORB implementation for the "copyable library" philosophy.
