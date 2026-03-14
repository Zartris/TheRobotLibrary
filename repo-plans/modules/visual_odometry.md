# Module: `visual_odometry`

**Milestone:** M6 — Visual Perception Building Blocks
**Status:** Not Started (pre-scaffolded)
**Depends on:** common (including `common/camera.hpp`)

---

### Phase 1 — Interface Design

- [ ] `include/visual_odometry/visual_odometry.hpp`:
  - `PointMatch` (pixel_a, pixel_b — both `Eigen::Vector2d`)
  - `OdometryResult` (relative pose `SE3`, inlier_count)
  - `VoError` enum (insufficient_matches, degenerate_configuration, cheirality_failed)
  - `VisualOdometryConfig` (min_matches, ransac_iterations, ransac_threshold)
  - `VisualOdometry` — `estimatePose(matches, intrinsics) → std::expected<OdometryResult, VoError>`
- [ ] `CameraIntrinsics` imported from `<common/camera.hpp>` — not redefined here
- [ ] All public API errors returned via `std::expected<T, VoError>`

### Phase 2 — Failing Tests (Red)

- [ ] `tests/CMakeLists.txt` with Catch2 target `visual_odometry_tests`
- [ ] `tests/visual_odometry_tests.cpp`:
  - `CameraIntrinsics::project` → `unproject` round-trip (pixel within 1e-6)
  - `CameraIntrinsics::project` behind camera returns `std::nullopt`
  - `CameraIntrinsics::K()` diagonal matches fx, fy, cx, cy
  - `VisualOdometry` construction with default config succeeds
  - Insufficient matches error returned when fewer than 8 correspondences
  - Pure-translation synthetic scenario: known 0.25 m forward step, recovered rotation within 5°
- [ ] Verify tests fail before implementation

```bash
cmake --build build --target visual_odometry_tests
cd build && ctest -R visual_odometry --output-on-failure
```

### Phase 3 — Implementation (Green)

- [ ] `src/visual_odometry.cpp`:
  - 8-point algorithm: build 9×9 linear system from point correspondences; SVD null-space
  - Rank-2 enforcement: zero smallest singular value of F via SVD
  - F → E via camera intrinsics: `E = K^T F K`
  - SVD decompose E into 4 (R, t) candidates
  - Cheirality check: triangulate test point in both cameras; select candidate with positive depth in both
  - RANSAC outer loop: Sampson distance inlier classification, best model retained
- [ ] Only depends on `common` (Eigen via common)
- [ ] Use `common::getLogger("visual_odometry")`

### Phase 4 — Passing Tests

- [ ] All `visual_odometry_tests` pass

```bash
cmake --build build --target visual_odometry_tests
cd build && ctest -R visual_odometry --output-on-failure
```

### Phase 4.5 — Observability

> **This phase gates module completion.**

- [ ] `ILogger` injected via `common::getLogger("visual_odometry")`
- [ ] State transitions at `DEBUG` (estimatePose entry, inlier count, pose accepted/rejected)
- [ ] RANSAC iteration count at `TRACE`
- [ ] At least one test asserts expected log lines appear
- [ ] Zero `ERROR`-level entries during nominal test runs

```bash
cmake --build build --target visual_odometry_tests
cd build && ctest -R visual_odometry --output-on-failure 2>&1 | grep "\[DEBUG\]\|\[TRACE\]"
```

### Phase 5 — Simulation Integration

- [ ] `VisualOdometry` used by sim integration test `test_vo_integration` (in `workspace/simulation/tests/`)
- [ ] Module itself does not link against simulation — sim links the module

### Phase 6 — Frontend Visualization

- [ ] (M6.5 / M9) VO trajectory trace overlay on top-down map view

### Phase 7 — Docs Polish

- [ ] Update `workspace/robotics/perception/visual_odometry/README.md` with usage examples
- [ ] Add 8-point algorithm + cheirality theory to `docs/theory.md`
- [ ] Move this file to `repo-plans/modules/done/visual_odometry.md`
