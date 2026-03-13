# Visual SLAM Milestones Implementation Plan

> **For agentic workers:** REQUIRED: Use superpowers:subagent-driven-development (if subagents available) or superpowers:executing-plans to implement this plan. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Restructure the milestone roadmap to introduce a new M6 "Visual Perception Building Blocks" milestone, rename the existing M6 (SLAM) to M6.5, add `common/transforms` to M1, and create all supporting module task files.

**Architecture:** All work is documentation only — markdown milestone files, module task files, and README updates. No C++ implementation code. The pre-scaffolded workspace directories already exist; this plan formalises them in the planning layer.

**Tech Stack:** Markdown, repo-plans directory structure, `module-task-template.md` conventions.

**Spec:** `docs/superpowers/specs/2026-03-13-visual-slam-milestones-design.md`

---

## Chunk 1: M1 Updates — Add `common/transforms`

**Files:**
- Modify: `repo-plans/modules/common.md`
- Modify: `repo-plans/milestones/M1-minimum-viable-robot.md`

### Task 1: Add `common/transforms` tasks to `repo-plans/modules/common.md`

- [ ] **Step 1: Open `repo-plans/modules/common.md` and locate Phase 1**

  The file currently lists M1-A interface tasks ending with `differential_drive.hpp`. We need to add a new sub-section for `common/transforms` after the existing Phase 1 block but still under M1-A.

- [ ] **Step 2: Append the following block after the closing `>` note in Phase 1**

  Add this between the `> **Map types live here.**` note and `### Phase 2 — Failing Tests (Red)`:

  ```markdown
  #### Sub-module: `common/transforms` (INTERFACE, header-only)

  > Pre-scaffolded at `workspace/robotics/common/transforms/`. CMake wiring already present.
  > SE2/SE3/SO3 Lie-group types extending the existing `Transform2D` for 3D use cases (M6+).

  - [ ] `include/transforms/transforms.hpp` — `SE2` (x, y, θ; compose, inverse, transformPoint, interpolate, toMatrix/fromMatrix), `SE3` (translation + quaternion; compose, inverse, transformPoint, toSE2, toMatrix/fromMatrix), `SO3` (quaternion; compose, inverse, rotate, toMatrix, fromAxisAngle, fromRPY/toRPY) — all header-only
  - [ ] `tests/transforms_tests.cpp` — SE2 compose + inverse round-trip, SE3 compose + inverse, SO3 fromRPY/toRPY round-trip, SE3::toSE2 projection, all within 1e-9 tolerance
  - [ ] Verify `add_subdirectory(transforms)` is present in `workspace/robotics/common/CMakeLists.txt` (already done — confirm only)
  ```

- [ ] **Step 3: Verify the edit looks correct**

  ```bash
  grep -A 15 "Sub-module.*common/transforms" repo-plans/modules/common.md
  ```
  Expected: the three bullet points appear under the sub-module heading.

- [ ] **Step 4: Also add `common/camera.hpp` section to `common.md` for M6**

  Append the following after the `### Phase 7 — Docs Polish` section (at the end of `common.md`):

  ```markdown
  ---

  ## M6 Addition: `common/camera.hpp`

  **Milestone:** M6 — Visual Perception Building Blocks
  **Tracked here** because `camera.hpp` is a header added to the `common` library target.

  - [ ] `include/common/camera.hpp` — `CameraIntrinsics` (fx, fy, cx, cy, width, height; `project()` → `std::optional<Eigen::Vector2d>`, `unproject()` → `Eigen::Vector3d`, `K()` → `Eigen::Matrix3d`), `CameraFrame` (timestamp `double`, width, height, data `std::vector<uint8_t>` row-major grayscale)
  - [ ] Tests: `project` round-trip; behind-camera returns `std::nullopt`; `K()` diagonal matches intrinsics; `CameraFrame::valid()` on empty data returns false
  ```

- [ ] **Step 5: Commit**

  ```bash
  git add repo-plans/modules/common.md
  git commit -m "plan(M1/M6): add common/transforms and common/camera.hpp tasks to common.md"
  ```

---

### Task 2: Add `common/transforms` to M1 milestone What's IN table

- [ ] **Step 1: Open `repo-plans/milestones/M1-minimum-viable-robot.md`**

  Locate the `| Common | \`common\` |` row in the What's IN table.

- [ ] **Step 2: Update that row to mention transforms**

  Change:
  ```markdown
  | Common | `common` | Pose2D, Twist, Transform2D, OccupancyGrid, LaserScan, Path, geometry utils, kinematics models, map types, swappable interfaces |
  ```
  To:
  ```markdown
  | Common | `common` | Pose2D, Twist, Transform2D, OccupancyGrid, LaserScan, Path, geometry utils, kinematics models, map types, swappable interfaces |
  | Common | `common/transforms` | SE2, SE3, SO3 rigid-body types (header-only INTERFACE; extends Transform2D for 3D use in M6+) |
  ```

- [ ] **Step 3: Verify**

  ```bash
  grep "common/transforms" repo-plans/milestones/M1-minimum-viable-robot.md
  ```
  Expected: one matching line with the new row.

- [ ] **Step 4: Commit**

  ```bash
  git add repo-plans/milestones/M1-minimum-viable-robot.md
  git commit -m "plan(M1): add common/transforms to M1 What's IN table"
  ```

---

## Chunk 2: Create M6-visual-perception.md

**Files:**
- Create: `repo-plans/milestones/M6-visual-perception.md`

### Task 3: Write the new M6 milestone document

- [ ] **Step 1: Create `repo-plans/milestones/M6-visual-perception.md` with this content**

  ```markdown
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
  - [ ] `CameraFrame` streamed over WebSocket at 30 Hz as base64-encoded grayscale payload alongside existing lidar state JSON
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
  3. Simulation builds with camera renderer; streams `CameraFrame` at 30 Hz alongside lidar state (verified by `test_vo_integration`)
  4. `test_vo_integration` passes: trajectory drift < 0.5 m over 5 m straight corridor
  5. All modules pass Phase 4.5 Observability gate

  ## NOT IN

  - Visual SLAM (map, loop closure) — M6.5
  - Frontend camera view panel — M6.5 or M9
  - `place_recognition` as a standalone module — loop closure is a sub-task within `visual_slam`
  - OpenCV dependency
  - GPU acceleration
  - VIO (visual-inertial odometry) — todos.md P3
  ```

- [ ] **Step 2: Verify file exists and header is correct**

  ```bash
  head -5 repo-plans/milestones/M6-visual-perception.md
  ```
  Expected: `# M6 — Visual Perception Building Blocks`

- [ ] **Step 3: Commit**

  ```bash
  git add repo-plans/milestones/M6-visual-perception.md
  git commit -m "plan(M6): create M6 Visual Perception Building Blocks milestone"
  ```

---

## Chunk 3: Rename M6-slam.md → M6.5-slam.md and Apply Edits

**Files:**
- Rename: `repo-plans/milestones/M6-slam.md` → `repo-plans/milestones/M6.5-slam.md`
- Modify: `repo-plans/milestones/M6.5-slam.md`

### Task 4: Rename the file

- [ ] **Step 1: Git-rename the file**

  ```bash
  git mv repo-plans/milestones/M6-slam.md repo-plans/milestones/M6.5-slam.md
  ```

- [ ] **Step 2: Update the header inside the file**

  In `repo-plans/milestones/M6.5-slam.md`, change the first line:
  ```markdown
  # M6 — SLAM
  ```
  to:
  ```markdown
  # M6.5 — SLAM
  ```

  And update the dependencies line:
  ```markdown
  **Dependencies:** M4 (perception pipeline), M5 (state estimation interfaces)
  ```
  to:
  ```markdown
  **Dependencies:** M4 (perception pipeline), M5 (state estimation interfaces), M6 (feature_extraction + visual_odometry — required by visual_slam only; ekf_slam and lidar_slam can start without M6)
  ```

- [ ] **Step 3: Apply targeted edits to the `visual_slam` section**

  a) **Remove the camera infrastructure sub-section** (`### Camera Simulation Infrastructure (prerequisite)` and all its bullets) — this is now owned by M6-visual-perception.md.

  b) **Remove from `visual_slam` task list:**
  - `- [ ] Feature extraction: ORB features (implement or lightweight wrapper)`
  - `- [ ] Feature matching: brute-force or FLANN-like nearest-neighbor`
  - `- [ ] Visual odometry: essential matrix → relative pose from matched features`

  c) **Replace with:**
  ```markdown
  - [ ] `VisualSlam` receives pre-computed feature observations (`std::vector<Feature>` from `feature_extraction`) and pre-computed `OdometryResult` from `visual_odometry` as caller-supplied inputs — not linked internally (architecture rule)
  - [ ] Map management: landmark triangulation from feature tracks, keyframe selection (interval + feature-ratio criteria)
  - [ ] Loop closure: bag-of-words descriptor-similarity database built and queried within `visual_slam`; pose graph correction via Gauss-Newton (Eigen-only)
  ```

  d) **Rename `CameraImage` → `CameraFrame` throughout the file** (it now comes from `common/camera.hpp`).

  e) **Add note at top of `visual_slam` section:**
  ```markdown
  > **Prerequisite:** M6 (`feature_extraction` + `visual_odometry`) must be complete before implementing `visual_slam`. `ekf_slam` and `lidar_slam` have no M6 dependency and can be started immediately.
  ```

  _(Note: step a removes the entire Camera Simulation Infrastructure section, which includes the `include/common/camera.hpp` task. No separate removal step needed.)_

- [ ] **Step 4: Verify rename and header**

  ```bash
  ls repo-plans/milestones/M6*
  head -3 repo-plans/milestones/M6.5-slam.md
  ```
  Expected: only `M6-visual-perception.md` and `M6.5-slam.md` exist; header reads `# M6.5 — SLAM`.

- [ ] **Step 5: Commit**

  ```bash
  git add repo-plans/milestones/M6.5-slam.md
  git commit -m "plan(M6.5): rename M6-slam → M6.5-slam, update visual_slam tasks and dependencies"
  ```

---

## Chunk 4: Update `repo-plans/README.md`

**Files:**
- Modify: `repo-plans/README.md`

### Task 5: Update the milestone table

- [ ] **Step 1: Locate the milestone table in `repo-plans/README.md`**

  Find the `| **M6** |` row.

- [ ] **Step 2: Replace the M6 row and add M6.5**

  Change:
  ```markdown
  | **M6** | [SLAM](milestones/M6-slam.md) | EKF-SLAM + lidar SLAM + camera-based visual SLAM | Not Started |
  ```
  To:
  ```markdown
  | **M6** | [Visual Perception Building Blocks](milestones/M6-visual-perception.md) | feature_extraction, visual_odometry, common/camera.hpp, 2.5D sim renderer | Not Started |
  | **M6.5** | [SLAM](milestones/M6.5-slam.md) | EKF-SLAM + lidar SLAM + visual SLAM (consumes M6 pipeline) | Not Started |
  ```

- [ ] **Step 3: Verify table**

  ```bash
  grep "M6" repo-plans/README.md
  ```
  Expected: both M6 and M6.5 rows present; no reference to old `M6-slam.md`.

### Task 6: Update the dependency graph

- [ ] **Step 1: Replace the dependency graph**

  Find this exact block in `repo-plans/README.md`:
  ```
  ```
  M0 (infra)
   └→ M1 (minimum viable robot)
        └→ M2 (hardening & testing)
             ├→ M3 (control upgrades) → M11 (advanced control)
             ├→ M4 (perception upgrades)
             ├→ M5 (state estimation upgrades)
             │    └→ M6 (SLAM) ←── M4
             ├→ M7 (advanced planning) ←── M4
             ├→ M8 (multi-robot) ←── M3, M7 → M12 (fleet management)
             ├→ M9 (web frontend)
             └→ M10 (polish & showcase) ←── M8, M9
  ```
  ```

  Replace with:
  ```
  ```
  M0 (infra)
   └→ M1 (minimum viable robot + common/transforms)
        └→ M2 (hardening & testing)
             ├→ M3 (control upgrades) → M11 (advanced control)
             ├→ M4 (perception upgrades)
             │    └→ M6 (visual perception)
             │         └→ M6.5 (SLAM)
             ├→ M5 (state estimation upgrades)
             │    └→ M6.5 (SLAM)
             ├→ M7 (advanced planning) ←── M4
             ├→ M8 (multi-robot) ←── M3, M7 → M12 (fleet management)
             ├→ M9 (web frontend)
             └→ M10 (polish & showcase) ←── M8, M9
  ```
  ```

- [ ] **Step 2: Update the paragraph after the dependency graph**

  Find this exact sentence:
  ```
  After M2, milestones M3–M5 and M9 can be developed **in parallel** since they touch independent domains. M7 can start after M4 (needs costmaps). M8 needs M3 + M7. M11 requires M3 (stable control interfaces). M12 requires M8 (N-robot sim infrastructure). M10 can begin partially after M8; full completion requires all milestones.
  ```

  Replace with:
  ```
  After M2, milestones M3–M5 and M9 can be developed **in parallel** since they touch independent domains. M7 can start after M4 (needs costmaps). M8 needs M3 + M7. M11 requires M3 (stable control interfaces). M12 requires M8 (N-robot sim infrastructure). M10 can begin partially after M8; full completion requires all milestones.

  M6 (Visual Perception Building Blocks) requires M4 and can begin after M4 completes. M6.5 (SLAM) requires M4, M5, and M6 (for `visual_slam` specifically); `ekf_slam` and `lidar_slam` within M6.5 can start after M4+M5 without waiting for M6.
  ```

- [ ] **Step 3: Verify**

  ```bash
  grep -n "M6\|M6.5" repo-plans/README.md
  ```
  Expected: M6 and M6.5 appear in both the table and dependency graph; no reference to old `M6-slam.md`.

  ```bash
  grep "M6-slam.md" repo-plans/README.md
  ```
  Expected: no output (old link fully replaced).

- [ ] **Step 4: Commit**

  ```bash
  git add repo-plans/README.md
  git commit -m "plan(roadmap): insert M6 Visual Perception, rename SLAM to M6.5, update dependency graph"
  ```

  > **Note:** After this commit, `repo-plans/README.md` must not reference `M6-slam.md` anywhere. Verify: `grep "M6-slam.md" repo-plans/README.md` → no output.

---

## Chunk 5: Create Module Task Files

**Files:**
- Create: `repo-plans/modules/feature_extraction.md`
- Create: `repo-plans/modules/visual_odometry.md`
- Create: `repo-plans/modules/visual_slam.md`

### Task 7: Create `repo-plans/modules/feature_extraction.md`

- [ ] **Step 1: Create the file**

  ```markdown
  # Module: feature_extraction

  **Milestone:** M6 — Visual Perception Building Blocks
  **Status:** Not Started
  **Depends on:** common

  > Pre-scaffolded at `workspace/robotics/perception/feature_extraction/`.
  > CMake target `robotlib::feature_extraction` already present.

  ---

  ### Phase 1 — Interface Design

  - [ ] `include/feature_extraction/feature_extraction.hpp` — `Keypoint` (x, y, response, angle, octave), `Descriptor` (32-byte array), `Feature`, `Match`, `FeatureError`, `GrayscaleImage`, `FeatureExtractorConfig` (fastThreshold, maxFeatures, nOctaves, scaleFactor, useOrientation), `FeatureExtractor`, `MatcherConfig` (maxHammingDistance, crossCheck, ratioThreshold), `DescriptorMatcher`
  - [ ] All recoverable errors returned via `std::expected<T, FeatureError>` — no exceptions at public API boundaries
  - [ ] `GrayscaleImage::valid()` guards against null/empty input
  - [ ] `DescriptorMatcher::hammingDistance()` declared `static noexcept`

  ### Phase 2 — Failing Tests (Red)

  - [ ] `tests/feature_extraction_tests.cpp`:
    - `hammingDistance` — identical descriptors → 0; all bits flipped → 256; single bit difference → 1
    - `FeatureExtractor` — construction with default config; invalid image returns `FeatureError::InvalidImage`
    - `DescriptorMatcher` — empty inputs return empty matches; cross-check filters asymmetric matches; results sorted by distance ascending
  - [ ] Verify tests fail:

  ```bash
  cmake -B build -S workspace && cmake --build build --target feature_extraction_tests
  cd build && ctest -R feature_extraction --output-on-failure
  ```

  ### Phase 3 — Implementation (Green)

  - [ ] `src/feature_extraction.cpp`:
    - `hammingDistance`: `__builtin_popcount` XOR over 8 × 4-byte chunks — real implementation
    - `DescriptorMatcher::match`: forward pass (best match per query), optional cross-check reverse pass, Lowe ratio test, distance threshold filter, sort by distance — real implementation
    - `FeatureExtractor::detectAndCompute`: FAST corner detection + rBRIEF descriptors (stub with logged TODO acceptable for initial milestone; `hammingDistance` and matcher must be fully real)
  - [ ] `common::getLogger("feature_extraction")` used for all log output
  - [ ] Only links against `common` — no OpenCV, no other robotics modules

  ### Phase 4 — Passing Tests

  - [ ] All `feature_extraction_tests` pass
  - [ ] CI green

  ```bash
  cmake --build build --target feature_extraction_tests
  cd build && ctest -R feature_extraction --output-on-failure
  ```

  ### Phase 4.5 — Observability

  - [ ] State transitions logged at `DEBUG` (detectAndCompute entry with image dims, match entry with set sizes)
  - [ ] Per-match Hamming distance logged at `TRACE`
  - [ ] Zero `ERROR`-level logs during nominal test runs

  ```bash
  cd build && ctest -R feature_extraction --output-on-failure 2>&1 | grep "\[DEBUG\]\|\[TRACE\]"
  ```

  ### Phase 5 — Simulation Integration

  _`feature_extraction` is called by the sim integration test (not wired into the sim loop directly). See M6 integration test task in `M6-visual-perception.md`._

  ### Phase 6 — Frontend Visualization

  _N/A at M6. Feature overlays on camera view deferred to M6.5/M9._

  ### Phase 7 — Docs Polish

  - [ ] Update `workspace/robotics/perception/feature_extraction/README.md` with completed API
  - [ ] Update `docs/theory.md` with FAST + BRIEF implementation notes
  - [ ] Move this file to `repo-plans/modules/done/feature_extraction.md`
  ```

- [ ] **Step 2: Verify**

  ```bash
  head -5 repo-plans/modules/feature_extraction.md
  ```
  Expected: `# Module: feature_extraction`

- [ ] **Step 3: Commit**

  ```bash
  git add repo-plans/modules/feature_extraction.md
  git commit -m "plan(M6): add feature_extraction module task file"
  ```

---

### Task 8: Create `repo-plans/modules/visual_odometry.md`

- [ ] **Step 1: Create the file**

  ```markdown
  # Module: visual_odometry

  **Milestone:** M6 — Visual Perception Building Blocks
  **Status:** Not Started
  **Depends on:** common (includes `common/camera.hpp` for `CameraIntrinsics` + `CameraFrame`)

  > Pre-scaffolded at `workspace/robotics/perception/visual_odometry/`.
  > CMake target `robotlib::visual_odometry` already present.

  ---

  ### Phase 1 — Interface Design

  - [ ] `include/visual_odometry/visual_odometry.hpp`:
    - `CameraIntrinsics` imported from `<common/camera.hpp>` — **do not redefine**
    - `PointMatch` (point1, point2 as `Eigen::Vector2d`)
    - `OdometryResult` (relativePose `Eigen::Isometry3d`, inlierCount, matchCount, isValid)
    - `VoError` (Code: InsufficientMatches, DegenerateConfiguration, NumericalFailure; message)
    - `VisualOdometryConfig` (minMatchesRequired=8, ransacThresholdPx=2.0, ransacConfidence=0.999, maxRansacIterations=1000, minInlierRatio=0.3, useDepth=false)
    - `VisualOdometry` (pimpl; `estimatePose`, `estimatePoseWithDepth`)
  - [ ] All errors via `std::expected<OdometryResult, VoError>`

  ### Phase 2 — Failing Tests (Red)

  - [ ] `tests/visual_odometry_tests.cpp`:
    - `CameraIntrinsics::project` round-trip (project 3D point → unproject → same direction)
    - `CameraIntrinsics::project` behind camera (z ≤ 0) → returns `std::nullopt`
    - `CameraIntrinsics::K()` — diagonal entries match fx, fy, cx, cy
    - `VisualOdometry` construction — no crash
    - `estimatePose` with fewer than 8 matches → `VoError::InsufficientMatches`
    - Synthetic pure-translation test: 20 point pairs generated from known 0.1 m x-translation → recovered rotation within 5° of identity
  - [ ] Verify tests fail

  ```bash
  cmake --build build --target visual_odometry_tests
  cd build && ctest -R visual_odometry --output-on-failure
  ```

  ### Phase 3 — Implementation (Green)

  - [ ] `src/visual_odometry.cpp`:
    - `CameraIntrinsics::project`, `unproject`, `K()` — real implementations
    - `estimatePose`: normalised 8-point algorithm inside RANSAC (Sampson distance inlier test); rank-2 enforcement via SVD; F→E = K^T F K; decompose E → 4 (R,t) candidates; cheirality test selects unique valid pose; return `OdometryResult`
    - `estimatePoseWithDepth`: stub returning `isValid=false` with `WARN` log "depth-based VO not yet implemented"
  - [ ] `common::getLogger("visual_odometry")` used
  - [ ] Only links against `common`

  ### Phase 4 — Passing Tests

  - [ ] All `visual_odometry_tests` pass

  ```bash
  cmake --build build --target visual_odometry_tests
  cd build && ctest -R visual_odometry --output-on-failure
  ```

  ### Phase 4.5 — Observability

  - [ ] `estimatePose` entry logged at `DEBUG` (match count)
  - [ ] RANSAC inlier count logged at `DEBUG`
  - [ ] Per-iteration residual logged at `TRACE`

  ### Phase 5 — Simulation Integration

  _Called by `test_vo_integration` in `workspace/simulation/tests/`. Not wired into sim loop directly._

  ### Phase 6 — Frontend Visualization

  _Deferred to M6.5/M9 (camera view + feature tracks overlay)._

  ### Phase 7 — Docs Polish

  - [ ] Update `workspace/robotics/perception/visual_odometry/README.md`
  - [ ] Move this file to `repo-plans/modules/done/visual_odometry.md`
  ```

- [ ] **Step 2: Verify**

  ```bash
  head -5 repo-plans/modules/visual_odometry.md
  ```

- [ ] **Step 3: Commit**

  ```bash
  git add repo-plans/modules/visual_odometry.md
  git commit -m "plan(M6): add visual_odometry module task file"
  ```

---

### Task 9: Create `repo-plans/modules/visual_slam.md`

- [ ] **Step 1: Create the file**

  ```markdown
  # Module: visual_slam

  **Milestone:** M6.5 — SLAM
  **Status:** Not Started
  **Depends on:** common (including `common/camera.hpp`, `common/transforms`)

  > Pre-scaffolded at `workspace/robotics/state_estimation/visual_slam/`.
  > CMake target `robotlib::visual_slam` already present and correctly links only `common`.
  > IMPORTANT: Do NOT add `feature_extraction` or `visual_odometry` to CMakeLists.txt — architecture rule.

  > **Prerequisite:** M6 (`feature_extraction` + `visual_odometry`) must be complete.
  > `ekf_slam` and `lidar_slam` within M6.5 have no M6 dependency.

  ---

  ### Phase 1 — Interface Design

  - [ ] `include/visual_slam/visual_slam.hpp`:
    - `CameraIntrinsics` from `<common/camera.hpp>`
    - `Observation` (landmarkId, pixel `Eigen::Vector2d`, descriptor 32-byte array)
    - `Landmark` (id, position `Eigen::Vector3d`, descriptor, observationCount, isActive)
    - `KeyFrame` (id, pose `Eigen::Isometry3d`, observations, timestamp)
    - `TrackingResult` (Status enum: Tracking/Lost/Relocalized; cameraPose; trackedLandmarks; isKeyFrame)
    - `SlamError` (Code: NotInitialized/TrackingLost/MapEmpty/NumericalFailure; message)
    - `VisualSlamConfig` (minInitFeatures=100, minTrackedFeatures=30, keyFrameInterval=5, keyFrameFeatureRatio=0.8, loopClosureScoreThreshold=0.3, enableLoopClosure=true)
    - `VisualSlam::FrameInput` / `FeatureObservation` (x, y, descriptor, depth)
    - `VisualSlam::processFrame(FrameInput)` → `std::expected<TrackingResult, SlamError>`
  - [ ] Inputs to `processFrame` are pre-computed feature observations — caller runs `FeatureExtractor` and passes results in. `VisualSlam` does not call `FeatureExtractor` internally.

  ### Phase 2 — Failing Tests (Red)

  - [ ] `tests/visual_slam_tests.cpp`:
    - Construction → not initialized, no landmarks
    - Empty frame → Lost status or SlamError
    - Init with ≥ 150 features → `isInitialized()` true
    - 5 frames with overlapping features → status Tracking after init
    - `reset()` → not initialized, landmarks empty
    - < 5 features → not initialized

  ### Phase 3 — Implementation (Green)

  - [ ] `src/visual_slam.cpp`:
    - Map initialization: back-project features to 3D (unit depth for monocular, metric for depth-aided); log `INFO` "map initialized landmarks={}"
    - Tracking: feature association + PnP pose estimation (stub TODO acceptable); log `DEBUG` "tracking features={}"
    - Keyframe selection: interval + feature-ratio criteria; log `DEBUG` "keyframe inserted id={}"
    - Loop closure: bag-of-words similarity database within this module; log `DEBUG` "loop closure check score={}"
    - Pose graph correction: Gauss-Newton (Eigen-only) when loop closed
  - [ ] `common::getLogger("visual_slam")` used
  - [ ] Only links against `common` — no cross-module links

  ### Phase 4 — Passing Tests

  - [ ] All `visual_slam_tests` pass

  ### Phase 4.5 — Observability

  - [ ] Init, tracking, lost, keyframe insert, loop closure all logged at `DEBUG`
  - [ ] Per-frame feature count logged at `TRACE`

  ### Phase 5 — Simulation Integration

  - [ ] Wire into sim pipeline: sim runs `FeatureExtractor` → `VisualOdometry` → `VisualSlam::processFrame`
  - [ ] `visual_slam` landmark map streamed over WebSocket (landmark positions as JSON array)
  - [ ] Frontend: 2D top-down landmark map overlay; trajectory trace (M6.5 frontend tasks)
  - [ ] Mini-demo: robot explores unknown room, builds feature map, detects loop closure, corrects trajectory

  ### Phase 6 — Frontend Visualization

  - [ ] Camera view panel (what robot sees) — native frontend ImGui texture
  - [ ] Feature tracks overlay on camera view
  - [ ] Landmark map (3D points projected onto 2D top-down view)

  ### Phase 7 — Docs Polish

  - [ ] Update `workspace/robotics/state_estimation/visual_slam/README.md`
  - [ ] Move this file to `repo-plans/modules/done/visual_slam.md`
  ```

- [ ] **Step 2: Verify**

  ```bash
  head -5 repo-plans/modules/visual_slam.md
  ```

- [ ] **Step 3: Final verification — all new files present**

  ```bash
  ls repo-plans/milestones/M6* repo-plans/modules/feature_extraction.md repo-plans/modules/visual_odometry.md repo-plans/modules/visual_slam.md
  ```
  Expected: all 4 files listed.

- [ ] **Step 4: Commit**

  ```bash
  git add repo-plans/modules/visual_odometry.md repo-plans/modules/visual_slam.md
  git commit -m "plan(M6/M6.5): add visual_odometry and visual_slam module task files"
  ```

---

## Final Verification

- [ ] **Confirm milestone file list**

  ```bash
  ls repo-plans/milestones/
  ```
  Expected: `M6-visual-perception.md` and `M6.5-slam.md` present; no `M6-slam.md`.

- [ ] **Confirm README links are valid**

  ```bash
  grep -o "milestones/M[^)]*" repo-plans/README.md | sort
  ```
  Expected: all referenced files exist on disk.

- [ ] **Confirm README has no dead links to old M6-slam.md**

  ```bash
  grep "M6-slam.md" repo-plans/README.md
  ```
  Expected: no output.

- [ ] **Confirm `CameraImage` is gone from M6.5-slam.md** (fully renamed to `CameraFrame`)

  ```bash
  grep "CameraImage" repo-plans/milestones/M6.5-slam.md
  ```
  Expected: no output.

- [ ] **Confirm no module links against non-common targets**

  ```bash
  grep "target_link_libraries" workspace/robotics/perception/feature_extraction/CMakeLists.txt workspace/robotics/perception/visual_odometry/CMakeLists.txt workspace/robotics/state_estimation/visual_slam/CMakeLists.txt
  ```
  Expected: all three show only `common` as a linked library.
