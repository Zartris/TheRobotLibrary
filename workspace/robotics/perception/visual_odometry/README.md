# visual_odometry

Frame-to-frame camera motion estimation (relative SE(3) pose) from matched 2D visual
features using essential matrix decomposition.  This module is a foundational building
block for visual SLAM pipelines.

## Algorithm Summary

1. Accept a set of `PointMatch` correspondences between consecutive frames.
2. Estimate the Fundamental Matrix **F** via RANSAC + the normalised 8-point algorithm.
3. Lift to the Essential Matrix **E = K^T F K** using the camera intrinsics.
4. Decompose **E** via SVD into four candidate (R, t) solutions.
5. Apply the cheirality test to select the unique physically valid pose.

In monocular mode the translation is recovered up to an unknown scale (unit-norm direction
only).  When per-point depth is available (`estimatePoseWithDepth`) the full metric pose
can be obtained via a PnP solver (see `docs/theory.md` §7; implementation is a TODO).

## Public API

### Types

| Type | Description |
|------|-------------|
| `CameraIntrinsics` | Pinhole camera intrinsics (fx, fy, cx, cy, width, height) |
| `PointMatch` | 2D–2D pixel correspondence between frame 1 and frame 2 |
| `OdometryResult` | Recovered relative pose, inlier count, validity flag |
| `VoError` | Error type with codes: `InsufficientMatches`, `DegenerateConfiguration`, `NumericalFailure` |
| `VisualOdometryConfig` | Tuning parameters (RANSAC threshold, min inlier ratio, etc.) |
| `VisualOdometry` | Main estimator class |

### VisualOdometry

```cpp
// Construction
VisualOdometry vo(camera, config);   // config is optional

// Monocular: estimate pose from 2D–2D matches (scale-ambiguous translation)
std::expected<OdometryResult, VoError>
vo.estimatePose(const std::vector<PointMatch>& matches);

// RGB-D / stereo: estimate metric pose using depth at frame-1 points
std::expected<OdometryResult, VoError>
vo.estimatePoseWithDepth(const std::vector<PointMatch>& matches,
                         const std::vector<double>& depths);
```

### CameraIntrinsics helpers

```cpp
cam.project(point3d);      // -> std::optional<Eigen::Vector2d>
cam.unproject(pixel);      // -> Eigen::Vector3d (unit bearing vector)
cam.K();                   // -> Eigen::Matrix3d (3x3 intrinsic matrix)
```

## Usage Example

```cpp
#include <visual_odometry/visual_odometry.hpp>

// 1. Set up camera intrinsics.
visual_odometry::CameraIntrinsics cam;
cam.fx = 525.0;  cam.fy = 525.0;
cam.cx = 319.5;  cam.cy = 239.5;
cam.width = 640; cam.height = 480;

// 2. Optionally tune the estimator.
visual_odometry::VisualOdometryConfig cfg;
cfg.ransacThresholdPx   = 1.5;
cfg.maxRansacIterations = 500;
cfg.minInlierRatio      = 0.4;

// 3. Construct the estimator.
visual_odometry::VisualOdometry vo(cam, cfg);

// 4. Supply 2D–2D matches from your feature matcher.
std::vector<visual_odometry::PointMatch> matches = /* ... */;

// 5. Estimate relative pose.
auto result = vo.estimatePose(matches);
if (result.has_value() && result->isValid) {
    Eigen::Isometry3d T_2from1 = result->relativePose;
    // T_2from1.linear()      -> rotation matrix R
    // T_2from1.translation() -> translation direction (unit norm, monocular)
} else {
    // result.error().code / result.error().message
}
```

## Configuration Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `minMatchesRequired` | 8 | Minimum correspondences before attempting estimation |
| `ransacThresholdPx` | 2.0 | Sampson distance threshold (pixels) for inlier classification |
| `ransacConfidence` | 0.999 | Desired RANSAC success probability (informational) |
| `maxRansacIterations` | 1000 | Maximum RANSAC iterations |
| `minInlierRatio` | 0.3 | Reject result if fewer than this fraction are inliers |
| `useDepth` | false | Enable RGB-D mode (requires `estimatePoseWithDepth`) |

## Dependencies

- `common` (Eigen, spdlog via `common::getLogger`)

## Known Limitations / TODOs

- The 8-point algorithm is used inside RANSAC; replacing it with Nister's 5-point solver
  would reduce the minimum sample size and improve robustness.
- `estimatePoseWithDepth` (PnP / P3P) is stubbed and returns `isValid = false`.
- No bundle-adjustment refinement of the final pose; this is left to a higher-level SLAM
  module.
- Scale is unobservable in monocular mode; integration into a SLAM system with loop closure
  is required for drift-free trajectories.

## Further Reading

See `docs/theory.md` for a detailed treatment of epipolar geometry, the essential matrix,
RANSAC, E decomposition, scale ambiguity, and depth-aided VO.
