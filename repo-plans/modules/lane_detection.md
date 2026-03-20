# Module: lane_detection

**Milestone:** M22 — Optimal Output Feedback & Automotive Perception  
**Status:** Not Started  
**Depends on:** common, M6 (`common/camera.hpp` image types), M4 (Canny edge images as input)

---

### Phase 1 — Interface Design

- [ ] `include/lane_detection/lane_detector.hpp` — `LaneDetector`
- [ ] `include/lane_detection/lane_detection_config.hpp` — `LaneDetectionConfig` (Hough: rho resolution, theta resolution, threshold, min_line_length, max_line_gap; polynomial degree; RANSAC iterations)
- [ ] `include/lane_detection/lane_types.hpp` — `LanePolynomial` (coefficients `std::vector<double>`, degree, validity flag), `LaneResult` (left: `LanePolynomial`, right: `LanePolynomial`)

### Phase 2 — Failing Tests (Red)

- [ ] `tests/CMakeLists.txt`
- [ ] `tests/test_lane_detection.cpp`:
  - Synthetic edge image (two known straight lanes): detected polynomials within 5% of ground truth coefficients
  - Curved lanes (known quadratic): degree-2 polynomial fit within tolerance; curvature extracted correctly
  - Only left lane present: `right.validity = false`; left lane detected correctly; no crash
  - Pure noise edge image: both lanes report `validity = false`; no false-positive lines
  - Very narrow lane gap: both lanes still separated correctly if Hough threshold met

### Phase 3 — Implementation (Green)

- [ ] `src/lane_detector.cpp` — Hough transform (Eigen-based accumulator; no OpenCV dependency); left/right split by slope + image position; polynomial fit via RANSAC least-squares on Hough line sample points

### Phase 4 — Passing Tests

- [ ] All tests pass

### Phase 4.5 — Observability

> **This phase gates module completion.** Both human developers and AI agents must be able
> to verify correct behavior through logs and metrics — not just visual inspection.

- [ ] `ILogger` injected into module constructor via `common::getLogger("lane_detection")` (mockable in tests)
- [ ] All state transitions logged at `DEBUG` level (init, reset, mode changes, error paths, detected line count + polynomial fit residual)
- [ ] Hot-loop performance metrics logged at `TRACE` level (Hough accumulator build time in µs)
- [ ] At least one test asserts expected log lines appear via stdout capture
- [ ] Zero `ERROR`-level log entries during all nominal test runs

```bash
# Confirm logging output during tests:
cmake --build build --target lane_detection_tests
cd build && ctest -R lane_detection --output-on-failure 2>&1 | grep "\[DEBUG\]\|\[TRACE\]"
```

### Phase 5 — Simulation Integration

- [ ] Sim note: operates on pre-computed binary edge images; sim edge rendering deferred to post-M22 extension

### Phase 6 — Visualization

- [ ] Render left/right lane polynomial curves overlaid on camera image

### Phase 7 — Docs Polish

- [ ] Update README.md
- [ ] Move this file to `repo-plans/modules/done/lane_detection.md`
