# Module: `visual_slam`

**Milestone:** M6.5 — SLAM
**Status:** Not Started (pre-scaffolded)
**Depends on:** common (including `common/camera.hpp`, `common/transforms`)

**Architecture note:** `visual_slam` does **not** link against `feature_extraction` or `visual_odometry`. The pipeline is assembled by the simulation (or user application): `FeatureExtractor` → `VisualOdometry` → `VisualSlam` through `common/` types. Each module is independently testable and reusable. **Prerequisite:** M6 must be complete before beginning this module.

---

### Phase 1 — Interface Design

- [ ] `include/visual_slam/visual_slam.hpp`:
  - `FeatureObservation` (pixel: `Eigen::Vector2d`, descriptor: 32-byte array)
  - `Landmark` (id, position: `Eigen::Vector3d`, descriptor)
  - `Keyframe` (id, pose: `SE3`, observations)
  - `SlamState` enum (initializing, tracking, lost)
  - `SlamResult` (pose: `SE3`, state, num_landmarks, num_keyframes)
  - `SlamError` enum
  - `VisualSlamConfig` (keyframe_distance_threshold, loop_closure_similarity_threshold, max_landmarks)
  - `VisualSlam` — `update(observations: std::span<FeatureObservation>, odometry: OdometryResult, intrinsics: CameraIntrinsics) → std::expected<SlamResult, SlamError>`
- [ ] `OdometryResult` and `CameraIntrinsics` imported from `<common/camera.hpp>` — not from `visual_odometry` header
- [ ] All public API errors returned via `std::expected<T, SlamError>`

### Phase 2 — Failing Tests (Red)

- [ ] `tests/CMakeLists.txt` with Catch2 target `visual_slam_tests`
- [ ] `tests/test_visual_slam.cpp`:
  - Known camera + known landmarks → correct pixel projection via `CameraIntrinsics::project`
  - Two consecutive frames with synthetic observations → `SlamState` transitions from `initializing` to `tracking`
  - Keyframe selection: close frames do not trigger new keyframe; moved frame does
  - Loop closure detection: re-observed area with matching descriptors triggers correction
  - 10-frame sequence → trajectory drift bounded (< 0.5 m over 2.5 m path)
  - Landmark map size grows monotonically on new observations
- [ ] Verify tests fail before implementation

```bash
cmake --build build --target visual_slam_tests
cd build && ctest -R visual_slam --output-on-failure
```

### Phase 3 — Implementation (Green)

- [ ] `src/visual_slam.cpp`:
  - **Map management:** add/update/query landmarks by descriptor; prune lost landmarks after N keyframes
  - **Keyframe selection:** trigger when translation > threshold or rotation > threshold vs last keyframe
  - **Landmark triangulation:** DLT from two keyframe observations; depth validation
  - **Loop closure:** bag-of-words similarity score over active keyframe descriptors; trigger pose graph correction when score exceeds threshold
  - **Pose graph correction:** propagate accumulated correction from loop-closure edge back along keyframe chain
  - State machine: `initializing` (< 2 keyframes) → `tracking` → `lost` (< 4 inlier observations)
- [ ] Only depends on `common` (Eigen via common, `common/camera.hpp`, `common/transforms`)
- [ ] Use `common::getLogger("visual_slam")`

### Phase 4 — Passing Tests

- [ ] All `visual_slam_tests` pass

```bash
cmake --build build --target visual_slam_tests
cd build && ctest -R visual_slam --output-on-failure
```

### Phase 4.5 — Observability

> **This phase gates module completion.**

- [ ] `ILogger` injected via `common::getLogger("visual_slam")`
- [ ] State transitions at `DEBUG` (SlamState changes, keyframe insertion, loop closure trigger, pose correction applied)
- [ ] Hot-loop metrics at `TRACE` (observations processed, landmarks queried, bag-of-words score)
- [ ] At least one test asserts expected log lines appear
- [ ] Zero `ERROR`-level entries during nominal test runs

```bash
cmake --build build --target visual_slam_tests
cd build && ctest -R visual_slam --output-on-failure 2>&1 | grep "\[DEBUG\]\|\[TRACE\]"
```

### Phase 5 — Simulation Integration

- [ ] Sim assembles pipeline: `CameraRenderer` → `FeatureExtractor` → `VisualOdometry` → `VisualSlam` each tick
- [ ] `VisualSlam` output (pose, landmark map) added to bridge state adapter
- [ ] Unknown-map scenario: robot explores corridor with camera, builds feature map end-to-end
- [ ] Module itself does not link against simulation

### Phase 6 — Visualization

- [ ] Camera view panel showing live `CameraFrame`
- [ ] Feature tracks overlay on camera view
- [ ] Visual SLAM landmark map projected onto 2D top-down view
- [ ] Mini-demo: robot explores with camera, builds feature map, closes loop

### Phase 7 — Docs Polish

- [ ] Update `workspace/robotics/state_estimation/visual_slam/README.md` with usage examples
- [ ] Add visual SLAM theory (bag-of-words, pose graph, DLT triangulation) to `docs/theory.md`
- [ ] Move this file to `repo-plans/modules/done/visual_slam.md`
