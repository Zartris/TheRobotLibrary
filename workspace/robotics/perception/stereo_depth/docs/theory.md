# Stereo Depth Estimation Theory

## 1. Stereo Geometry

Two cameras separated by baseline $b$ observe the same 3D point $P$.
The point projects to pixel $u_L$ in the left image and $u_R$ in the right image.
**Disparity:** $d = u_L - u_R > 0$ for points in front of both cameras.

The 3D depth is recovered by:
$$Z = \frac{f \cdot b}{d}$$

where $f$ is the focal length in pixels and $b$ is the baseline in metres.

## 2. Stereo Rectification

Raw stereo images have non-horizontal epipolar lines due to lens distortion and
camera mounting imprecision. **Rectification** applies a homography to both images
so that:
- Lens distortion is removed
- Epipolar lines become horizontal (same row for corresponding points)

Rectification uses the `StereoCalibration` (intrinsics $K_L$, $K_R$, extrinsics $R$, $t$)
from `common/camera.hpp`. After rectification, a horizontal search suffices for
disparity estimation.

## 3. Block Matching

For each pixel $(u, v)$ in the left image, find the best matching pixel in row $v$
of the right image by minimising the sum of absolute differences (SAD) over a block
of size $B \times B$:
$$d^* = \arg\min_{d \in [d_{min}, d_{max}]} \text{SAD}(u, v, d)$$

**Limitations:** block matching fails in textureless regions, on occlusion boundaries,
and in specular reflections. Invalid pixels are marked with disparity = 0.

## 4. Disparity-to-Depth Conversion

Dense disparity map $d(u,v)$ → dense depth map $Z(u,v) = f \cdot b / d(u,v)$.
Invalid pixels ($d = 0$, occluded or unreliable) yield depth = infinity or NaN
(caller's choice, documented in `stereo_types.hpp`).

## 5. Semi-Global Matching (SGM) — Future Extension

SGM propagates cost along multiple 1D paths to enforce smoothness, dramatically improving
disparity quality near object boundaries. The `StereoMatcher` includes an `sgm` flag
reserved for future implementation — the M17 target is block matching only.

## 6. Depth Accuracy

Depth error scales as $\sigma_Z \approx Z^2 \sigma_d / (f \cdot b)$. For a given
disparity uncertainty $\sigma_d$, depth accuracy degrades quadratically with range.
This motivates using stereo only for near-to-medium range depth (a few metres for
typical robotics baselines).
