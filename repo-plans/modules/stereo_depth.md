# Module: stereo_depth

**Milestone:** M17 — Camera Perception II  
**Status:** Not Started  
**Depends on:** common, M6 (`common/camera.hpp` types: `CameraIntrinsics`, `StereoCalibration`)

---

### Phase 1 — Interface Design

- [ ] `include/stereo_depth/stereo_matcher.hpp` — `StereoMatcher` (block matching; optional SGM flag)
- [ ] `include/stereo_depth/stereo_rectifier.hpp` — `StereoRectifier` (apply `StereoCalibration` from `common/camera.hpp`)
- [ ] `include/stereo_depth/stereo_types.hpp` — `DisparityMap`, `DepthMap`, `StereoCalibration` (baseline, K_left, K_right, R, t)

### Phase 2 — Failing Tests (Red)

- [ ] `tests/CMakeLists.txt`
- [ ] `tests/test_stereo_depth.cpp`:
  - Synthetic stereo pair (known disparity, integer shifts): block matching recovers disparity within ±1 px
  - Disparity-to-depth: `Z = f·b/d` — computed depth matches known ground truth within 1% for d ≥ 2 px
  - Rectified pair: after `StereoRectifier`, corresponding points share the same row (epipolar constraint verified on 10 point pairs)
  - Invalid pixels: disparity = 0 (occluded) → depth = `std::numeric_limits<float>::infinity()` or NaN (documented choice)
  - Edge case: block size larger than image → returns error via `std::expected`

### Phase 3 — Implementation (Green)

- [ ] `src/stereo_matcher.cpp` — block matching disparity; disparity-to-depth via `Z = f·b/d`
- [ ] `src/stereo_rectifier.cpp` — apply `StereoCalibration` to undistort + row-align stereo pairs

### Phase 4 — Passing Tests

- [ ] All tests pass

### Phase 4.5 — Observability

> **This phase gates module completion.** Both human developers and AI agents must be able
> to verify correct behavior through logs and metrics — not just frontend visuals.

- [ ] `ILogger` injected into module constructor via `common::getLogger("stereo_depth")` (mockable in tests)
- [ ] All state transitions logged at `DEBUG` level (init, disparity search range, match quality)
- [ ] Hot-loop performance metrics logged at `TRACE` level (per-row solve time in µs)
- [ ] At least one test asserts expected log lines appear via stdout capture
- [ ] Zero `ERROR`-level log entries during all nominal test runs

```bash
# Confirm logging output during tests:
cmake --build build --target stereo_depth_tests
cd build && ctest -R stereo_depth --output-on-failure 2>&1 | grep "\[DEBUG\]\|\[TRACE\]"
```

### Phase 5 — Simulation Integration

- [ ] M17 scope covers offline/synthetic testing only
- [ ] Live sim stereo rendering deferred to post-M17 sim upgrade (requires two camera viewpoints offset by baseline)

### Phase 6 — Frontend Visualization

- [ ] Depth map false-colour overlay
- [ ] Disparity histogram panel

### Phase 7 — Docs Polish

- [ ] Update README.md
- [ ] Move this file to `repo-plans/modules/done/stereo_depth.md`
