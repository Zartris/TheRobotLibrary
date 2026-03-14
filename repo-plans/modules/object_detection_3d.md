# Module: object_detection_3d

**Milestone:** M19 — Depth Perception & 3D Understanding  
**Status:** Not Started  
**Depends on:** common, M4 (conceptual extension of `obstacle_detection`), M19 (`depth_camera` as point cloud source)

---

### Phase 1 — Interface Design

- [ ] `include/object_detection_3d/object_detector_3d.hpp` — `ObjectDetector3D`
- [ ] `include/object_detection_3d/object_detection_3d_config.hpp` — `ObjectDetection3DConfig` (DBSCAN: `eps`, `min_points`; box: `min_cluster_size`, `max_cluster_size`)
- [ ] `include/object_detection_3d/object_3d_types.hpp` — `Object3D` (centroid, dimensions, orientation `Eigen::Matrix3f`, `ObjectClass` enum: `PERSON / CAR / UNKNOWN`), `Detection3DList`

### Phase 2 — Failing Tests (Red)

- [ ] `tests/CMakeLists.txt`
- [ ] `tests/test_object_detector_3d.cpp`:
  - Synthetic cloud with 3 isolated clusters: DBSCAN returns exactly 3 clusters
  - Flat disk cluster (known PCA): box major axis matches disk normal (< 5° angular error)
  - Tall thin cluster 0.4×0.4×1.8 m: classified as `PERSON`
  - Wide flat cluster 2.0×1.5×0.5 m: classified as `CAR`
  - Single-point cluster (below `min_points`): filtered out
  - Empty point cloud: returns empty `Detection3DList`; no crash

### Phase 3 — Implementation (Green)

- [ ] `src/object_detector_3d.cpp` — 3D DBSCAN with ε-neighbour queries; PCA box fitting via `SelfAdjointEigenSolver`; size-based classification (`PERSON` if height > 1.0 m AND footprint < 0.5×0.5 m, `CAR` if footprint > 1.5×1.5 m)

### Phase 4 — Passing Tests

- [ ] All tests pass

### Phase 4.5 — Observability

> **This phase gates module completion.** Both human developers and AI agents must be able
> to verify correct behavior through logs and metrics — not just frontend visuals.

- [ ] `ILogger` injected into module constructor via `common::getLogger("object_detection_3d")` (mockable in tests)
- [ ] All state transitions logged at `DEBUG` level (init, reset, mode changes, error paths, cluster count + per-cluster point count)
- [ ] Hot-loop performance metrics logged at `TRACE` level (DBSCAN + PCA time in µs)
- [ ] At least one test asserts expected log lines appear via stdout capture
- [ ] Zero `ERROR`-level log entries during all nominal test runs

```bash
# Confirm logging output during tests:
cmake --build build --target object_detection_3d_tests
cd build && ctest -R object_detection_3d --output-on-failure 2>&1 | grep "\[DEBUG\]\|\[TRACE\]"
```

### Phase 5 — Simulation Integration

- [ ] Integrate with lidar point cloud pipeline; 3D objects renderable as oriented boxes

### Phase 6 — Frontend Visualization

- [ ] Render oriented bounding boxes with class label overlays in 3D scene view

### Phase 7 — Docs Polish

- [ ] Update README.md
- [ ] Move this file to `repo-plans/modules/done/object_detection_3d.md`
