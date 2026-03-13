# Theory: Feature-Based Visual SLAM

---

## 1. The SLAM Problem

Simultaneous Localisation and Mapping (SLAM) is the problem of jointly estimating:

- **Map** $\mathcal{M} = \{\mathbf{p}_1, \mathbf{p}_2, \ldots, \mathbf{p}_N\}$ — a set of $N$ 3-D landmarks in the world frame.
- **Trajectory** $\mathcal{X} = \{T_1, T_2, \ldots, T_K\}$ — a sequence of $K$ camera poses $T_{wc} \in SE(3)$ (world-from-camera transforms).

Given a sequence of observations $\mathcal{Z} = \{z_{ij}\}$ where $z_{ij}$ is the pixel measurement of landmark $j$ in frame $i$, the maximum a posteriori (MAP) estimate is:

$$
\mathcal{X}^*, \mathcal{M}^* = \arg\min_{\mathcal{X},\, \mathcal{M}}
\sum_{(i,j) \in \mathcal{E}} \rho\!\left(\left\|\mathbf{e}_{ij}\right\|^2_{\Sigma_{ij}^{-1}}\right)
$$

where $\mathbf{e}_{ij} = z_{ij} - \pi(T_i, \mathbf{p}_j)$ is the **re-projection error**, $\pi$ is the camera projection function, $\Sigma_{ij}$ is the measurement covariance, and $\rho$ is a robust cost function (e.g. Huber).

---

## 2. Feature-Based Visual SLAM Pipeline

```
 ┌─────────────────────────────────────────────────────────────────┐
 │  FRONT-END  (per-frame, real-time)                               │
 │                                                                  │
 │  Image → Feature Detection → Descriptor Extraction              │
 │       → Feature Matching (frame-to-map)                         │
 │       → PnP Pose Estimation + RANSAC                            │
 │       → New Feature Triangulation                               │
 │       → Keyframe Selection                                       │
 └────────────────────────┬────────────────────────────────────────┘
                          │  Keyframe + Observations
 ┌────────────────────────▼────────────────────────────────────────┐
 │  BACK-END  (asynchronous, may be slower)                         │
 │                                                                  │
 │  Local Bundle Adjustment (sliding window of recent keyframes)   │
 │  Loop Closure Detection (place recognition → geometric verify)  │
 │  Global Pose Graph Optimisation (on loop closure)               │
 └─────────────────────────────────────────────────────────────────┘
```

---

## 3. Front-End: Feature Detection, Tracking, and Mapping

### 3.1 Feature Detection and Description

A **feature** is a locally distinctive image region.  Classic choices:

| Detector/Descriptor | Type   | Key Property |
|---|---|---|
| ORB  | Binary | Fast, royalty-free; used by ORB-SLAM2/3 |
| SIFT | Float  | Scale and rotation invariant; patented until 2020 |
| SURF | Float  | Faster SIFT approximation |
| BRIEF | Binary | Very fast matching but not rotation-invariant |

Binary descriptors compare pixel intensities at predefined pattern pairs, yielding a compact bitstring.  **Hamming distance** (popcount of XOR) is the matching metric.

### 3.2 Feature Matching

Given descriptors $\{d_i\}$ in the current frame and $\{d_j\}$ projected from the map, matching finds pairs $(i, j)$ minimising descriptor distance subject to:

- **Ratio test** (Lowe): best match distance / second-best match distance $< 0.8$.
- **Epipolar constraint** (initialisation only): matches lie near corresponding epipolar lines.
- **Re-projection filter**: after pose estimation, discard matches with re-projection error $> \tau$ pixels.

### 3.3 Perspective-n-Point (PnP) Pose Estimation

Given $n$ matched 3-D–2-D correspondences $\{(\mathbf{p}_j, z_{ij})\}$, PnP recovers the rotation $R$ and translation $\mathbf{t}$ satisfying:

$$
\lambda \begin{bmatrix} u \\ v \\ 1 \end{bmatrix}
= K \begin{bmatrix} R & \mathbf{t} \end{bmatrix}
\begin{bmatrix} \mathbf{p}_j \\ 1 \end{bmatrix}
$$

where $K$ is the $3\times3$ intrinsic matrix and $\lambda > 0$ is a projective scale factor.

**EPnP** (Lepetit et al.) solves this in $O(n)$ using four virtual control points.  A RANSAC outer loop handles outlier correspondences.

### 3.4 Keyframe Selection

A new keyframe is inserted when either:

1. **Time criterion**: at least $N_{\text{kf}}$ frames have elapsed since the last keyframe.
2. **Feature ratio criterion**: the number of tracked landmarks falls below $\alpha$ times the feature count at the last keyframe ($\alpha \approx 0.8$).

Keyframes are the primary storage unit: they anchor poses, hold observations, and are queried for loop closure.

---

## 4. Back-End: Bundle Adjustment

**Bundle adjustment (BA)** jointly minimises the sum of squared (or robust) re-projection errors over all keyframe poses and landmark positions:

$$
\min_{T_1,\ldots,T_K,\, \mathbf{p}_1,\ldots,\mathbf{p}_N}
\sum_{(i,j) \in \mathcal{E}}
\rho\!\left(\left\|z_{ij} - \pi(T_i, \mathbf{p}_j)\right\|^2_{\Sigma^{-1}_{ij}}\right)
$$

This is a nonlinear least-squares problem solved with **Gauss-Newton** or **Levenberg-Marquardt** iteration.  At each step the normal equations are:

$$
\left(J^\top W J + \lambda I\right) \delta x = -J^\top W \mathbf{e}
$$

where $J$ is the re-projection Jacobian, $W = \Sigma^{-1}$ the information matrix, and $\delta x$ the incremental update to all poses and landmark positions in their respective tangent spaces.

The system is **sparse**: each re-projection error involves only one camera pose and one landmark.  Exploiting the **Schur complement** (eliminating landmarks first) reduces the dense linear system to a $6K \times 6K$ problem in poses only — the **reduced camera system** — which is solved with sparse Cholesky factorisation.

**Local BA** operates on a sliding window of the most recent $W$ keyframes plus their co-visible landmarks, keeping per-frame cost $O(W^2)$.

**Global BA** optimises the entire graph; triggered rarely (e.g. after loop closure).

---

## 5. Loop Closure

Accumulated dead-reckoning drift causes the map to diverge from the true trajectory.  Loop closure detects revisited places and adds long-range constraints to correct the drift globally.

### 5.1 Place Recognition

**Bag-of-Words (BoW):** Descriptors are quantised against a pre-trained visual vocabulary (a $k$-d tree of cluster centres).  Each keyframe is represented as a sparse vector of word frequencies (TF-IDF weighted).  Cosine similarity between BoW vectors gives a fast approximate place match score in $O(W)$ per query where $W \ll N_{\text{vocab}}$.

**Alternative:** Direct descriptor-index similarity (e.g. FAISS nearest-neighbour search on ORB descriptors concatenated into a compact embedding).

### 5.2 Geometric Verification

A candidate loop is accepted only if a sufficient number of feature matches pass an Essential/Homography RANSAC between the query keyframe and the candidate, confirming a geometrically consistent relative pose.

### 5.3 Pose Graph Optimisation

Once a loop constraint $T_{\text{rel}}$ between keyframes $i$ and $j$ is verified, it is added to the **pose graph**:

$$
\min_{T_1,\ldots,T_K}
\sum_{(i,j) \in \mathcal{E}_{\text{odom}}} \left\|\log(T_j^{-1} T_i \hat{T}_{ij})\right\|^2
+ \sum_{(i,j) \in \mathcal{E}_{\text{loop}}} \left\|\log(T_j^{-1} T_i T_{\text{rel}}^{-1})\right\|^2
$$

where $\log : SE(3) \to \mathfrak{se}(3)$ is the matrix logarithm.  This distributes the loop-closure correction smoothly across all intermediate poses.  After pose graph optimisation, landmark positions are updated by re-triangulation or a final global BA.

---

## 6. Map Initialisation

### 6.1 Stereo Initialisation

With a calibrated stereo rig, landmark depths are directly available via triangulation from the stereo baseline.  Initialisation is immediate and **metric** (true scale).

### 6.2 Monocular Initialisation

Without a known baseline, absolute scale is unobservable.  Two approaches:

**Homography decomposition** (for planar scenes): fit $H$ to four matches; decompose into $(R, \mathbf{t})$ pairs via SVD.

**Essential matrix decomposition** (for general 3-D scenes): fit $E$ to five matches (Nistér algorithm); decompose into four $(R, \mathbf{t})$ candidates; disambiguate by requiring positive depth for all triangulated points.

The resulting map has an arbitrary scale factor $s$ — distances are relative, not metric.  Scale can be recovered by fusing with IMU measurements or known object sizes.

---

## 7. Scale Ambiguity in Monocular SLAM

A key limitation of monocular visual SLAM is that the trajectory and map are only determined up to a global scale.  Formally, if $(\mathcal{X}, \mathcal{M})$ is a valid solution then so is $(\mathcal{X}', \mathcal{M}')$ where translation norms are scaled by $s > 0$ and landmark positions by the same $s$.

**Practical consequences:**
- Depth estimates from triangulation cannot be converted to metric without external reference.
- Integration of velocity or position from GPS/wheel-odometry requires scale estimation.
- Visual-inertial odometry (VIO) recovers metric scale from IMU accelerometer readings.

---

## 8. Data Association

Correct matching of observations to landmarks is critical.  Strategies:

| Method | Description |
|---|---|
| Descriptor matching | Nearest neighbour in descriptor space (Hamming / L2) |
| Ratio test | Lowe's ratio test rejects ambiguous matches |
| RANSAC | Random sample consensus removes geometric outliers |
| Re-projection filter | Reject matches with pixel error > threshold after pose estimation |
| Chi-squared test | Mahalanobis-distance based outlier rejection |

**Data association failure** (wrong matches accepted) causes map corruption and is the primary cause of tracking loss.  Robust cost functions (Huber, Cauchy) in BA partially mitigate the effect of residual outliers.

---

## 9. System Comparison

| System | Type | Features | Loop Closure | Scale |
|---|---|---|---|---|
| **ORB-SLAM2** | Feature (stereo/RGBD/mono) | ORB | BoW (DBoW2) | Metric (stereo/RGBD), relative (mono) |
| **ORB-SLAM3** | Feature + inertial | ORB | BoW | Metric (VIO) |
| **PTAM** | Feature (mono) | FAST + SSD | None | Relative |
| **LSD-SLAM** | Semi-dense direct | Gradient magnitude | — | Relative |
| **DSO** | Sparse direct | Intensity patches | None | Relative |
| **ElasticFusion** | Dense (RGBD) | — | Deformation graph | Metric |

Feature-based systems (ORB-SLAM family) are the dominant choice for reliable long-term mapping due to their robustness to appearance change and efficient relocalization via BoW.

---

## 10. Bundle Adjustment: Detailed Mathematics

The reprojection function for a pinhole camera is:

$$
\pi(T_{wc}, \mathbf{p}_w) =
\begin{bmatrix} f_x \frac{X_c}{Z_c} + c_x \\ f_y \frac{Y_c}{Z_c} + c_y \end{bmatrix},
\quad
\begin{bmatrix} X_c \\ Y_c \\ Z_c \end{bmatrix} = R^\top (\mathbf{p}_w - \mathbf{t})
$$

The Jacobian of the re-projection error with respect to the pose perturbation $\delta \xi \in \mathfrak{se}(3)$ and landmark position $\delta \mathbf{p}$ can be written in closed form using the chain rule over the rotation Lie group $SO(3)$.

For a Lie group parameterisation $T \leftarrow T \cdot \exp(\delta\xi)$ (left perturbation):

$$
\frac{\partial \mathbf{e}}{\partial \delta\xi}
= \frac{\partial \mathbf{e}}{\partial \mathbf{X}_c}
\cdot \frac{\partial \mathbf{X}_c}{\partial \delta\xi},
\quad
\frac{\partial \mathbf{X}_c}{\partial \delta\xi}
= \begin{bmatrix} I_{3\times3} & -[\mathbf{X}_c]_\times \end{bmatrix}
$$

where $[\cdot]_\times$ is the $3\times3$ skew-symmetric cross-product matrix.

The sparse BA structure exploits the fact that each residual $\mathbf{e}_{ij}$ depends on at most one pose $T_i$ and one landmark $\mathbf{p}_j$, so $J$ has a characteristic **arrow-head** sparsity pattern after reordering.

---

## References

1. **Mur-Artal, R., Tardós, J. D.** — *ORB-SLAM2: An Open-Source SLAM System for Monocular, Stereo and RGB-D Cameras*. IEEE Trans. Robotics, 2017.
2. **Mur-Artal, R., Tardós, J. D.** — *ORB-SLAM3: An Accurate Open-Source Library for Visual, Visual-Inertial and Multi-Map SLAM*. IEEE Trans. Robotics, 2021.
3. **Klein, G., Murray, D.** — *Parallel Tracking and Mapping for Small AR Workspaces (PTAM)*. ISMAR, 2007.
4. **Engel, J., Schöps, T., Cremers, D.** — *LSD-SLAM: Large-Scale Direct Monocular SLAM*. ECCV, 2014.
5. **Engel, J., Koltun, V., Cremers, D.** — *Direct Sparse Odometry*. IEEE TPAMI, 2018.
6. **Triggs, B. et al.** — *Bundle Adjustment — A Modern Synthesis*. Vision Algorithms: Theory & Practice, LNCS 1883, 2000.
7. **Lepetit, V., Moreno-Noguer, F., Fua, P.** — *EPnP: An Accurate O(n) Solution to the PnP Problem*. IJCV, 2009.
8. **Nistér, D.** — *An Efficient Solution to the Five-Point Relative Pose Problem*. IEEE TPAMI, 2004.
9. **Thrun, S., Burgard, W., Fox, D.** — *Probabilistic Robotics*, MIT Press, 2005. Chapter 10 (Graph SLAM).
10. **Gálvez-López, D., Tardós, J. D.** — *Bags of Binary Words for Fast Place Recognition in Image Sequences*. IEEE Trans. Robotics, 2012. (DBoW2)
