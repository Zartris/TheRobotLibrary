# M19 — Depth Perception & 3D Understanding

**Status:** Not Started  
**Dependencies:** M6 (`common/camera.hpp` intrinsics + image types), M4 (`obstacle_detection` 2D clustering as conceptual predecessor; `OccupancyGrid` types).  
**Scope:** Adds RGB-D camera processing and 3D object detection from point clouds. Both modules deepen the perception stack into the true 3D domain. `depth_camera` handles a new sensor type (structured light / ToF); `object_detection_3d` elevates M4's 2D DBSCAN to full 3D oriented bounding boxes.

---

## Goal

M4's `obstacle_detection` detects 2D clusters in a lidar scan. M19 extends this in two directions: `depth_camera` adds a new sensor pathway (RGB-D → point cloud) feeding downstream processing; `object_detection_3d` recognizes full 3D extent and orientation of objects, enabling height-aware classification needed for real environments (pedestrians, cars, robots of different sizes). Both modules are independently implementable.

---

## Modules

### perception/depth_camera

RGB-D camera processing pipeline. Converts raw depth images (uint16 mm or float32 m) to dense point clouds using camera intrinsics, fills holes via averaging, removes statistical outliers, and provides an `RgbdCamera` wrapper that bundles aligned RGB + depth.

- [ ] `include/depth_camera/depth_image_processor.hpp` — `DepthImageProcessor`
- [ ] `include/depth_camera/rgbd_camera.hpp` — `RgbdCamera` (intrinsics from `common/camera.hpp` `CameraIntrinsics`, depth scale factor, min/max valid depth range)
- [ ] `include/depth_camera/depth_types.hpp` — `DepthImage` (float32 row-major grid, NaN = invalid), `RgbdFrame` (rgb image + depth image + timestamp), `PointCloud` (`Eigen::MatrixX3f`, one row per valid point)
- [ ] `src/depth_image_processor.cpp` — depth → point cloud: `X = (u − cx)·d/fx`, `Y = (v − cy)·d/fy`, `Z = d`; hole filling: NaN → average of valid 8-neighbours (max 2 passes); outlier removal: statistical (mean + k·σ distance filter)
- [ ] Frustum filtering: discard points outside configured min/max depth and field-of-view cone
- [ ] `tests/test_depth_camera.cpp`:
  - Flat plane at known depth `Z = 1.0 m`: all valid pixels → point cloud; mean Z within 0.1% of 1.0 m
  - Three known pixels verify correct 3D position (corner + centre) via `X = (u − cx)·d/fx`
  - 20% random NaN holes: after hole filling, < 2% NaN remaining; filled values within 5% of ground truth
  - Statistical outlier removal: 5 outlier points 10σ away → outliers removed, inliers intact
  - Frustum filter: points beyond max depth excluded; points within frustum retained
  - Empty depth image (all NaN): `toPointCloud()` → empty point cloud; no crash
- [ ] Phase 4.5: `ILogger`, valid pixel count + hole-fill pass count at `DEBUG`, projection time per frame at `TRACE`
- [ ] Sim note: depth camera simulation requires sim renderer extension. M19 covers offline/synthetic testing; live sim RGB-D rendering deferred to a post-M19 sim upgrade.
- [ ] Frontend: false-colour depth overlay panel (hot = close, cool = far)

### perception/object_detection_3d

3D object detector from point clouds. Runs 3D DBSCAN to find clusters, fits an oriented bounding box to each cluster using PCA, and applies a size-based classification stub.

- [ ] `include/object_detection_3d/object_detector_3d.hpp` — `ObjectDetector3D`
- [ ] `include/object_detection_3d/object_detection_3d_config.hpp` — `ObjectDetection3DConfig` (DBSCAN: `eps`, `min_points`; box: `min_cluster_size`, `max_cluster_size`)
- [ ] `include/object_detection_3d/object_3d_types.hpp` — `Object3D` (centroid `Eigen::Vector3f`, dimensions `Eigen::Vector3f`, orientation `Eigen::Matrix3f`, `ObjectClass` enum: `PERSON / CAR / UNKNOWN`), `Detection3DList`
- [ ] `src/object_detector_3d.cpp` — 3D DBSCAN: KD-tree for ε-neighbour queries; PCA box fitting: covariance → `SelfAdjointEigenSolver` → eigenvectors = box axes; classification: `PERSON` if height > 1.0 m AND footprint < 0.5×0.5 m, `CAR` if footprint > 1.5×1.5 m, else `UNKNOWN`
- [ ] `tests/test_object_detector_3d.cpp`:
  - Synthetic cloud with 3 isolated clusters (known centroid, separation > 2ε): DBSCAN returns exactly 3 clusters
  - Flat disk cluster (known PCA): box major axis matches disk normal (< 5° angular error)
  - Tall thin cluster 0.4×0.4×1.8 m: classified as `PERSON`
  - Wide flat cluster 2.0×1.5×0.5 m: classified as `CAR`
  - Single-point cluster (below `min_points`): filtered out; not returned as object
  - Empty point cloud: returns empty `Detection3DList`; no crash
- [ ] Phase 4.5: `ILogger`, cluster count + per-cluster point count at `DEBUG`, DBSCAN + PCA time at `TRACE`
- [ ] Sim: integrate with lidar point cloud pipeline; 3D objects renderable as oriented boxes in frontend
- [ ] Frontend: render oriented bounding boxes with class label overlays in 3D scene view

---

## Deliverables

- [ ] `perception/depth_camera` module: interface, implementation, tests
- [ ] `perception/object_detection_3d` module: interface, implementation, tests
- [ ] Depth → point cloud: pixel positions correctly back-projected (3 known pixels verified)
- [ ] Hole filling: < 2% NaN remaining after filling a 20% hole depth image
- [ ] 3D DBSCAN + PCA orientation verified on synthetic point clouds
- [ ] All modules pass Phase 4.5 — Observability gate

---

## Exit Criteria

1. Depth → point cloud: pixel positions correctly back-projected (verified on 3 known pixels)
2. Hole filling: < 2% NaN remaining after filling a 20% hole depth image
3. DBSCAN finds correct cluster count on synthetic 3-cluster point cloud
4. PCA box major axis within 5° of ground truth orientation
5. Classification stub: PERSON / CAR rules produce correct labels on synthetic clusters
6. All unit tests pass, CI green
7. All modules pass Phase 4.5 — Observability gate (state transitions at `DEBUG`, metrics at `TRACE`)

---

## NOT IN

Dense 3D reconstruction (TSDF / surfel maps), semantic 3D labelling (→ M23 `semantic_segmentation`), real-time GPU-accelerated DBSCAN, IMU-aligned point clouds, RGB-D SLAM (requires M6.5 + future extension).
