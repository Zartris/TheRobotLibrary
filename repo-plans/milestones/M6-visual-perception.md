# M6 — Visual Perception Building Blocks

**Status:** Not Started
**Dependencies:** M4 (occupancy_grid + ray_casting used by 2.5D camera renderer)
**Scope:** Perception infrastructure for visual SLAM: feature extraction, visual odometry, shared camera types, and a 2.5D textured simulation renderer.

---

## Goal

The robot gains a simulated camera. Feature extraction and visual odometry pipelines are implemented and independently validated. The simulation streams live camera frames alongside lidar state. A headless integration test confirms the full VO pipeline produces bounded-drift trajectories on rendered sim frames.

This milestone delivers the building blocks that M6.5 (SLAM) consumes — without building SLAM itself.

---

## Architecture Note

All robotics modules (`feature_extraction`, `visual_odometry`) depend only on `common`. They do **not** link against each other. The pipeline is assembled by the simulation (or caller): `FeatureExtractor` → `VisualOdometry` → (M6.5) `VisualSlam`. Data flows through types in `common/camera.hpp`.

---

## Modules

### common/camera.hpp

Shared camera types consumed by `visual_odometry`, `visual_slam`, and the simulation camera sensor.

- [ ] `workspace/robotics/common/include/common/camera.hpp` — `CameraIntrinsics` (fx, fy, cx, cy, width, height; `project()`, `unproject()`, `K()`), `CameraFrame` (timestamp, width, height, grayscale pixel buffer `std::vector<uint8_t>`)
- [ ] Add `camera.hpp` include to `workspace/robotics/common/CMakeLists.txt` header list (no new target needed — header-only addition to existing `common`)
- [ ] Tests: `project` then `unproject` round-trip; `project` behind camera returns nullopt; `K()` diagonal matches fx/fy/cx/cy

### feature_extraction

FAST keypoints + BRIEF descriptors. No OpenCV.

Pre-scaffolded at `workspace/robotics/perception/feature_extraction/`.

- [ ] **Phase 1 — Interface Design**
  - [ ] `include/feature_extraction/feature_extraction.hpp` — `Keypoint`, `Descriptor` (32-byte array), `Feature`, `Match`, `FeatureError`, `GrayscaleImage`, `FeatureExtractorConfig`, `FeatureExtractor`, `MatcherConfig`, `DescriptorMatcher`
  - [ ] All public API errors returned via `std::expected<T, FeatureError>`
- [ ] **Phase 2 — Failing Tests (Red)**
  - [ ] `tests/feature_extraction_tests.cpp` — `hammingDistance` on identical/all-flipped/single-bit descriptors; `DescriptorMatcher` cross-check; ratio test; empty-input guard
  - [ ] Verify tests fail: `cmake --build build --target feature_extraction_tests && cd build && ctest -R feature_extraction --output-on-failure`
- [ ] **Phase 3 — Implementation (Green)**
  - [ ] `src/feature_extraction.cpp` — `hammingDistance` via `__builtin_popcount` XOR over 8×4-byte chunks (real); `DescriptorMatcher::match` full forward + cross-check + ratio-test + sort (real); `FeatureExtractor::detectAndCompute` FAST + rBRIEF (stub with TODO for full FAST implementation acceptable at this milestone)
  - [ ] Use `common::getLogger("feature_extraction")`
- [ ] **Phase 4 — Passing Tests**
  - [ ] All `feature_extraction_tests` pass
- [ ] **Phase 4.5 — Observability**
  - [ ] State transitions at DEBUG (detect entry, match entry)
  - [ ] Hot-loop metrics at TRACE (per-descriptor distance, iteration count)

### visual_odometry

Essential matrix decomposition. Frame-to-frame pose estimation. No OpenCV.

Pre-scaffolded at `workspace/robotics/perception/visual_odometry/`.

- [ ] **Phase 1 — Interface Design**
  - [ ] `include/visual_odometry/visual_odometry.hpp` — `PointMatch`, `OdometryResult`, `VoError`, `VisualOdometryConfig`, `VisualOdometry`
  - [ ] `CameraIntrinsics` imported from `<common/camera.hpp>` (not redefined here)
- [ ] **Phase 2 — Failing Tests (Red)**
  - [ ] `tests/visual_odometry_tests.cpp` — `project` round-trip; behind-camera nullopt; K matrix; construction; insufficient matches error; pure-translation synthetic scenario (rotation within 5°)
  - [ ] Verify tests fail
- [ ] **Phase 3 — Implementation (Green)**
  - [ ] `src/visual_odometry.cpp` — 8-point algorithm inside RANSAC (Sampson distance); rank-2 enforcement (SVD, zero smallest singular value); F→E via K; SVD decompose E into 4 candidates; cheirality check selects unique valid pose
  - [ ] Use `common::getLogger("visual_odometry")`
- [ ] **Phase 4 — Passing Tests**
  - [ ] All `visual_odometry_tests` pass
- [ ] **Phase 4.5 — Observability**
  - [ ] State transitions at DEBUG (estimatePose entry, inlier count)
  - [ ] RANSAC iteration count at TRACE

---

## Simulation: 2.5D Camera Renderer

Wolfenstein-style per-column raycaster added to `workspace/simulation/`.

- [ ] `workspace/simulation/include/simulation/camera_sensor.hpp` — `CameraConfig` (fov, resolution, height_offset), `CameraRenderer` — produces `CameraFrame` from current robot pose + occupancy grid
- [ ] `workspace/simulation/src/camera_renderer.cpp` — per-column ray-cast; wall distance → column height; texture lookup (procedural: brick = alternating dark/light bands by wall position; concrete = gradient noise; checker = XOR grid); floor/ceiling distance-shaded grey
- [ ] Wall material IDs added to occupancy grid cell metadata (0 = free, 1 = wall-brick, 2 = wall-concrete, 3 = wall-checker)
- [ ] Scenario JSON support: `"camera": {"fov": 90, "resolution": [320, 240], "height_offset": 0.5}`
- [ ] `CameraFrame` populated by bridge SensorAdapter from MuJoCo offscreen render each physics tick
- [ ] `workspace/simulation/tests/test_camera_renderer.cpp` — known room (all walls brick) → CameraFrame has non-uniform pixel values; floor pixels darker than wall pixels at same distance; frame dimensions match config

---

## Integration Test

Headless VO pipeline test. Lives in `workspace/simulation/tests/` (sim links modules; modules do not link sim).

- [ ] `workspace/simulation/tests/test_vo_integration.cpp`
  - Scenario: straight corridor 5 m, textured walls
  - Fixed RNG seed; camera noise σ = 1 px added to feature pixel positions
  - 20 frames at 0.25 m/frame
  - Pipeline: `FeatureExtractor::detectAndCompute` → `DescriptorMatcher::match` (frame-to-frame) → `VisualOdometry::estimatePose`
  - Assert: cumulative trajectory drift < 0.5 m over the 5 m path

---

## Exit Criteria

1. `feature_extraction_tests` all pass (`hammingDistance`, cross-check, ratio test)
2. `visual_odometry_tests` all pass (round-trip, synthetic translation scenario within 5°)
3. Simulation builds with camera renderer; bridge populates `CameraFrame` from MuJoCo offscreen render (verified by `test_vo_integration`)
4. `test_vo_integration` passes: trajectory drift < 0.5 m over 5 m straight corridor
5. All modules pass Phase 4.5 Observability gate

## NOT IN

- Visual SLAM (map, loop closure) — M6.5
- Frontend camera view panel — M6.5
- `place_recognition` as a standalone module — loop closure is a sub-task within `visual_slam`
- OpenCV dependency
- GPU acceleration
- VIO (visual-inertial odometry) — todos.md P3
