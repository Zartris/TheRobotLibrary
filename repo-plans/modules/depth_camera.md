# Module: depth_camera

**Milestone:** M19 — Depth Perception & 3D Understanding  
**Status:** Not Started  
**Depends on:** common, M6 (`common/camera.hpp` `CameraIntrinsics`), M4 (conceptual predecessor)

---

### Phase 1 — Interface Design

- [ ] `include/depth_camera/depth_image_processor.hpp` — `DepthImageProcessor`
- [ ] `include/depth_camera/rgbd_camera.hpp` — `RgbdCamera` (intrinsics from `common/camera.hpp`, depth scale factor, min/max valid depth range)
- [ ] `include/depth_camera/depth_types.hpp` — `DepthImage` (float32 row-major grid, NaN = invalid), `RgbdFrame`, `PointCloud` (`Eigen::MatrixX3f`)

### Phase 2 — Failing Tests (Red)

- [ ] `tests/CMakeLists.txt`
- [ ] `tests/test_depth_camera.cpp`:
  - Flat plane at known depth `Z = 1.0 m`: mean Z within 0.1% of 1.0 m
  - Three known pixels verify correct 3D position via `X = (u − cx)·d/fx`
  - 20% random NaN holes: after hole filling, < 2% NaN remaining; filled values within 5% of ground truth
  - Statistical outlier removal: 5 outlier points 10σ away → outliers removed, inliers intact
  - Frustum filter: points beyond max depth excluded; points within frustum retained
  - Empty depth image (all NaN): `toPointCloud()` → empty point cloud; no crash

### Phase 3 — Implementation (Green)

- [ ] `src/depth_image_processor.cpp` — depth → point cloud projection; hole filling (NaN → average of valid 8-neighbours, max 2 passes); statistical outlier removal; frustum filtering

### Phase 4 — Passing Tests

- [ ] All tests pass

### Phase 4.5 — Observability

> **This phase gates module completion.** Both human developers and AI agents must be able
> to verify correct behavior through logs and metrics — not just frontend visuals.

- [ ] `ILogger` injected into module constructor via `common::getLogger("depth_camera")` (mockable in tests)
- [ ] All state transitions logged at `DEBUG` level (init, reset, mode changes, error paths, valid pixel count + hole-fill pass count)
- [ ] Hot-loop performance metrics logged at `TRACE` level (projection time per frame in µs)
- [ ] At least one test asserts expected log lines appear via stdout capture
- [ ] Zero `ERROR`-level log entries during all nominal test runs

```bash
# Confirm logging output during tests:
cmake --build build --target depth_camera_tests
cd build && ctest -R depth_camera --output-on-failure 2>&1 | grep "\[DEBUG\]\|\[TRACE\]"
```

### Phase 5 — Simulation Integration

- [ ] Sim note: live RGB-D rendering deferred; offline/synthetic testing covered by unit tests

### Phase 6 — Frontend Visualization

- [ ] False-colour depth overlay panel (hot = close, cool = far)

### Phase 7 — Docs Polish

- [ ] Update README.md
- [ ] Move this file to `repo-plans/modules/done/depth_camera.md`
