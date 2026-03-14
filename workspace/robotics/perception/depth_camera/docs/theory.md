# Depth Camera Theory

## Pinhole Projection Model

A depth camera associates each pixel $(u, v)$ with a depth value $d$ (in metres). The 3D point is recovered via the pinhole back-projection:

$$X = \frac{(u - c_x) \cdot d}{f_x}, \quad Y = \frac{(v - c_y) \cdot d}{f_y}, \quad Z = d$$

where $(f_x, f_y)$ are the focal lengths and $(c_x, c_y)$ is the principal point (all from `CameraIntrinsics`).

## Hole Filling

Invalid pixels (NaN or zero depth) are filled by averaging valid 8-neighbours. Up to 2 passes are applied to propagate filled values into larger gaps.

## Statistical Outlier Removal

For each point $p_i$, compute the mean distance $\bar{d}_i$ to its k nearest neighbours. Points where $\bar{d}_i > \mu + k\sigma$ (mean + $k$ standard deviations of all mean distances) are removed as outliers.

## Frustum Filtering

Points outside the configured depth range $[d_{\min}, d_{\max}]$ or outside the field-of-view cone are discarded before downstream processing.

## References

- Zhang, "A Flexible New Technique for Camera Calibration," IEEE TPAMI, 2000
- Rusu & Cousins, "3D is here: Point Cloud Library," ICRA 2011
