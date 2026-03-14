# depth_camera

RGB-D camera processing pipeline. Converts raw depth images (uint16 mm or float32 m) to dense 3D point clouds using camera intrinsics, fills NaN holes via neighbourhood averaging, removes statistical outliers, and provides an `RgbdCamera` wrapper that bundles aligned RGB + depth frames.

**Milestone:** M19 — Depth Perception & 3D Understanding  
**Status:** Scaffold only — awaiting implementation

## Features

- `DepthImageProcessor` — depth → point cloud projection using pinhole model
- Hole filling: NaN pixels → average of valid 8-neighbours (up to 2 passes)
- Statistical outlier removal: mean + k·σ distance filter
- Frustum filtering: discard points outside configured depth range and FoV cone
- `RgbdCamera` wrapper bundling intrinsics, depth scale, and valid range

## Dependencies

- `common` (logging, `CameraIntrinsics` from `common/camera.hpp`)
- Eigen3 (`MatrixX3f` point cloud representation)
