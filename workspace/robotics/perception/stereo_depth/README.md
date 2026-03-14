# stereo_depth

Block-matching stereo disparity estimation and disparity-to-depth conversion. Includes
`StereoRectifier` to undistort and row-align stereo pairs (epipolar lines → horizontal
rows) using `StereoCalibration` from `common/camera.hpp`.

## Components

- **`StereoMatcher`** — block matching disparity estimation; optional SGM flag for future extension
- **`StereoRectifier`** — applies `StereoCalibration` to undistort and rectify a stereo pair
- **`StereoTypes`** — `DisparityMap`, `DepthMap`, `StereoCalibration`

## Depth Formula

$$Z = \frac{f \cdot b}{d}$$

where $f$ is focal length (pixels), $b$ is baseline (metres), $d$ is disparity (pixels).
Occluded pixels ($d = 0$) yield `std::numeric_limits<float>::infinity()`.

## Dependencies

- `common` (camera types: `CameraIntrinsics`, `StereoCalibration`, logging)
- `Eigen3`

## Usage

```cpp
#include <stereo_depth/stereo_rectifier.hpp>
#include <stereo_depth/stereo_matcher.hpp>

StereoRectifier rectifier(calibration, common::getLogger("stereo_depth"));
auto [rect_left, rect_right] = rectifier.rectify(left_img, right_img);

StereoMatcher matcher(block_size, common::getLogger("stereo_depth"));
DisparityMap disp = matcher.compute(rect_left, rect_right);
DepthMap depth    = matcher.disparityToDepth(disp, calibration);
```

## Milestone

Part of **M17 — Camera Perception II**.  
See `repo-plans/modules/stereo_depth.md` for full task checklist.
