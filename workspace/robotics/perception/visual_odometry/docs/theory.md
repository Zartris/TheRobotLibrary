# Visual Odometry — Theory

## 1. Overview

Visual odometry (VO) is the process of estimating the motion (pose change) of a camera by
analysing the sequence of images it captures.  A single VO step takes two consecutive frames
and a set of 2D–2D point correspondences, and returns the relative pose
T_{2←1} ∈ SE(3) that maps points expressed in frame 1 into frame 2.

---

## 2. Epipolar Geometry

### 2.1 The Epipolar Constraint

Given two camera views of the same 3D scene, a point **p** in frame 1 and its corresponding
point **p'** in frame 2 satisfy the **epipolar constraint**:

```
p'^T  F  p = 0
```

where **F** ∈ ℝ^{3×3} is the **Fundamental Matrix** (rank 2, 7 degrees of freedom).
Geometrically, the ray through **p** and the camera 1 centre defines the **epipolar plane**;
the intersection of this plane with the image plane of camera 2 is the **epipolar line** on
which **p'** must lie.

### 2.2 The Fundamental Matrix F

F encodes the full projective geometry between two views and is independent of scene structure.
It satisfies:

```
F = K_2^{-T}  E  K_1^{-1}
```

where K₁, K₂ are the 3×3 intrinsic matrices.

### 2.3 The Essential Matrix E

When intrinsics are known, the **Essential Matrix** E ∈ ℝ^{3×3} is preferred:

```
E = K_2^T  F  K_1
```

E encodes only metric (Euclidean) geometry.  In normalised image coordinates
**x̂ = K^{-1} p_homogeneous**:

```
x̂_2^T  E  x̂_1 = 0
```

E has 5 degrees of freedom (3 for rotation, 2 for the direction of translation — scale is
unobservable from image pairs alone).

---

## 3. Essential Matrix Estimation

### 3.1 The 8-Point Algorithm (Longuet-Higgins 1981)

Rewrite the epipolar constraint as a linear equation in the 9 elements of F (or E):

```
[x₂x₁, x₂y₁, x₂, y₂x₁, y₂y₁, y₂, x₁, y₁, 1] · f = 0
```

Stack N ≥ 8 such equations into A ∈ ℝ^{N×9}.  The solution **f** is the right singular
vector of A corresponding to its smallest singular value (via SVD).  After reshaping to a
3×3 matrix, the rank-2 constraint is enforced by zeroing the smallest singular value of the
resulting matrix.

**Normalisation** (Hartley 1997): before forming A, translate and scale the pixel
coordinates so that their centroid is at the origin and the mean distance to the origin is
√2.  This dramatically improves numerical conditioning.

### 3.2 The 5-Point Algorithm (Nister 2004)

The minimal solver for the Essential Matrix requires only 5 correspondences (matching the
5 DOF of E).  It solves a system of polynomial equations whose solutions form a degree-10
univariate polynomial.  Up to 10 real solutions may exist; each is tested for consistency
with the data.

The 5-point algorithm is preferable inside RANSAC because smaller minimal sample sets lead
to exponentially fewer iterations for a given inlier ratio.  The implementation currently
uses the 8-point algorithm as a placeholder; replacing the `buildDesignMatrix` + SVD step
with a 5-point solver is marked `TODO` in the source.

---

## 4. RANSAC for Robust Estimation

Raw feature matches contain outliers (false correspondences).  **RANSAC**
(Random Sample Consensus, Fischler & Bolles 1981) robustly estimates E in the
presence of outliers:

1. Randomly sample the minimum number of correspondences (8 for the 8-point algorithm,
   5 for the 5-point algorithm).
2. Compute a candidate E from the sample.
3. Count **inliers**: correspondences whose **Sampson distance** to the epipolar line is
   below a pixel threshold `ransacThresholdPx`.
4. Retain the hypothesis with the most inliers.
5. Repeat for `maxRansacIterations` iterations (or until early termination when a
   sufficiently high inlier fraction is reached).

**Sampson distance** is a first-order approximation of the geometric reprojection error
and is cheap to compute:

```
d_S = (x₂^T F x₁)² / (‖(Fx₁)_{1:2}‖² + ‖(F^T x₂)_{1:2}‖²)
```

---

## 5. Decomposing E into (R, t)

The SVD of the essential matrix E = U Σ V^T yields **four candidate pose solutions**:

```
R₁ = U W   V^T,    t₁ = +U[:,2]
R₂ = U W   V^T,    t₂ = −U[:,2]
R₃ = U W^T V^T,    t₃ = +U[:,2]
R₄ = U W^T V^T,    t₄ = −U[:,2]
```

where  W = [[0,−1,0],[1,0,0],[0,0,1]].

Only one of the four combinations produces a geometry in which the triangulated 3D points
lie **in front of both cameras** (positive depth in both coordinate frames).  This is the
**cheirality check**: triangulate a set of correspondences for each candidate and select the
(R, t) that maximises the number of points with positive depth in both cameras.

---

## 6. Scale Ambiguity in Monocular VO

From two calibrated images alone, the translation **t** can only be recovered up to an
unknown positive scalar **s**:

```
T_{2←1} = (R, s·t̂)     where ‖t̂‖ = 1
```

This is a fundamental limitation of monocular geometry.  The consequence is **metric scale
drift**: pose estimates accumulate a scale factor that cannot be corrected without additional
information.

Common strategies to resolve scale:

- **Known scene structure**: if the size of one landmark is known (e.g. a calibration target
  or a map element with known metric dimensions), the scale can be fixed.
- **IMU integration**: inertial measurements provide metric acceleration, bounding
  the absolute translation scale.
- **Stereo baseline**: a calibrated stereo rig provides a known baseline length, giving
  direct metric scale (see §7).
- **RGB-D depth sensor**: per-pixel depth directly provides metric 3D points.

---

## 7. Depth-Aided VO (RGB-D / Stereo Mode)

When a per-point depth d is available for frame-1 points, the 2D–2D problem becomes 3D–2D:

1. **Lift** each frame-1 match to 3D:  `X_i = d_i · K⁻¹ · [u_i, v_i, 1]^T`
2. **Solve PnP** (Perspective-n-Point): find R, t such that
   `x₂_i ≈ π(R · X_i + t)` for all i,
   where π is the perspective projection.
3. Minimal solver: **P3P** (3 point correspondences) inside RANSAC.
   Over-determined: **DLT** (Direct Linear Transform) or iterative non-linear minimisation
   (Levenberg-Marquardt).

Because depth is metric, the recovered translation has physical scale.  This is the
`estimatePoseWithDepth` API path; the full PnP solver is marked `TODO` in the current
implementation.

---

## 8. Drift and Loop Closure

VO concatenates relative poses to build a trajectory estimate.  Errors accumulate over time:

- **Rotational error** leads to heading drift.
- **Scale drift** (monocular) causes the trajectory to compress or expand.
- **Noise** in feature detection and matching propagates into each pose estimate.

To correct drift, visual SLAM systems add a **loop closure** module that detects when the
camera revisits a previously seen location and introduces a pose-graph constraint.
Optimisation (pose-graph SLAM, bundle adjustment) then distributes the correction across all
past poses, yielding a globally consistent map.

The `VisualOdometry` module implements only the local, frame-to-frame estimation step.
Integration into a full SLAM pipeline (keyframe management, map, loop closure) is outside
its scope.

---

## 9. References

- R. Hartley and A. Zisserman, *Multiple View Geometry in Computer Vision*, 2nd ed.,
  Cambridge University Press, 2004.
- H. C. Longuet-Higgins, "A computer algorithm for reconstructing a scene from two
  projections," *Nature*, 293:133–135, 1981.
- D. Nister, "An efficient solution to the five-point relative pose problem,"
  *IEEE TPAMI*, 26(6):756–770, 2004.
- M. A. Fischler and R. C. Bolles, "Random sample consensus: a paradigm for model fitting
  with applications to image analysis and automated cartography," *CACM*, 24(6):381–395, 1981.
- R. Hartley, "In defense of the eight-point algorithm," *IEEE TPAMI*, 19(6):580–593, 1997.
- D. Scaramuzza and F. Fraundorfer, "Visual odometry: Part I — the first 30 years and
  fundamentals," *IEEE Robotics & Automation Magazine*, 18(4):80–92, 2011.
