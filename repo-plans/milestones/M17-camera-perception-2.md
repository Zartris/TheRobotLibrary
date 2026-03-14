# M17 — Camera Perception II

**Status:** Not Started  
**Dependencies:** M6 (`feature_extraction` descriptors, `common/camera.hpp` types). No dependency on M14 or M15 (though `place_recognition` output can be fed into `pose_graph` from M14 post-completion).  
**Scope:** Descriptor-based loop closure detection (works with both lidar and visual descriptors) plus stereo camera disparity and depth estimation. Both modules are independent within M17.

---

## Goal

Extend camera-based perception with two capabilities that build directly on the M6
infrastructure: `place_recognition` provides a reusable descriptor-based place database
for loop closure detection (usable by both lidar and visual SLAM); `stereo_depth` provides
block-matching disparity estimation and depth conversion, establishing the foundation for
scale-resolved visual odometry and RGB-D SLAM. Both modules are independently implementable.

---

## Modules

### perception/place_recognition

Descriptor-based place database. Supports both lidar scan descriptors (e.g., a
scan-context-style histogram) and visual feature descriptor aggregates. Query returns
candidate frame IDs with similarity scores above a threshold.

- [ ] `include/place_recognition/place_recognizer.hpp` — `PlaceRecognizer`
- [ ] `include/place_recognition/place_database.hpp` — `PlaceDatabase` (indexed by frame ID)
- [ ] `include/place_recognition/place_types.hpp` — `PlaceDescriptor` (generic `std::vector<float>`), `PlaceCandidate` (frame_id + score)
- [ ] `src/place_recognizer.cpp` — L2 / cosine similarity search; nearest-neighbour with score threshold
- [ ] `tests/test_place_recognizer.cpp`:
  - 10-frame database: query with same descriptor → correct frame ID returned (score ≥ threshold)
  - Query with perturbed descriptor (5% noise): top-1 still correct
  - False positive rate: random descriptor → no match above threshold (100 random queries, < 2 false positives)
  - Lidar descriptor: scan histogram passed as `PlaceDescriptor`; visual descriptor: aggregated BRIEF as `PlaceDescriptor` — same API handles both
  - Large database (1000 frames): query completes in < 10 ms (linear scan acceptable at this scale)
- [ ] Phase 4.5: `ILogger`, database size + query time at `DEBUG`, candidate scores at `TRACE`
- [ ] Integration note: output `PlaceCandidate` list can be routed to `pose_graph (M14)` as loop closure edges

### perception/stereo_depth

Block-matching stereo disparity estimation and disparity-to-depth conversion.
`StereoRectifier` applies calibration to undistort and row-align stereo pairs
(epipolar lines become horizontal rows).

- [ ] `include/stereo_depth/stereo_matcher.hpp` — `StereoMatcher` (block matching; optional SGM flag)
- [ ] `include/stereo_depth/stereo_rectifier.hpp` — `StereoRectifier` (apply `StereoCalibration` from `common/camera.hpp`)
- [ ] `include/stereo_depth/stereo_types.hpp` — `DisparityMap`, `DepthMap`, `StereoCalibration` (baseline, K_left, K_right, R, t)
- [ ] `src/stereo_matcher.cpp` + `src/stereo_rectifier.cpp`
- [ ] `tests/test_stereo_depth.cpp`:
  - Synthetic stereo pair (known disparity, integer shifts): block matching recovers disparity within ±1 px
  - Disparity-to-depth: `Z = f·b/d` — computed depth matches known ground truth within 1% for d ≥ 2 px
  - Rectified pair: after `StereoRectifier`, corresponding points share the same row (epipolar constraint verified on 10 point pairs)
  - Invalid pixels: disparity = 0 (occluded) → depth = `std::numeric_limits<float>::infinity()` or NaN (documented choice)
  - Edge case: block size larger than image → returns error via `std::expected`
- [ ] Phase 4.5: `ILogger`, disparity search range + match quality at `DEBUG`, per-row solve time at `TRACE`
- [ ] Sim note: stereo integration requires two camera viewpoints offset by baseline. M17 scope covers offline/synthetic testing; live sim stereo rendering deferred to post-M17 sim upgrade.
- [ ] Frontend: depth map false-colour overlay; disparity histogram panel

---

## Deliverables

- [ ] `perception/place_recognition` module: interface, implementation, tests
- [ ] `perception/stereo_depth` module: interface, implementation, tests
- [ ] Frontend shows depth map false-colour overlay and disparity histogram
- [ ] Place recognizer integration note documented for post-M17 pose_graph wiring
- [ ] All modules pass Phase 4.5 — Observability gate

---

## Exit Criteria

1. Place recognizer: 10-frame database → correct revisit detected; false positive rate < 2/100
2. Works with both lidar scan and visual feature descriptor formats (same `PlaceDescriptor` type)
3. Stereo depth: synthetic pair with known disparity → depth error < 1%
4. StereoRectifier: epipolar constraint satisfied on all test point pairs after rectification
5. All unit tests pass, CI green
6. All modules pass Phase 4.5 — Observability gate

---

## NOT IN

Semi-global matching full graph cuts (block matching is the M17 target; SGM is a flag for
future extension), RGB-D camera processing (→ P6 `depth_camera`), dense 3D reconstruction,
Bag-of-Words vocabulary training (→ `visual_slam` uses its own vocabulary internally in M6.5),
online stereo calibration.
