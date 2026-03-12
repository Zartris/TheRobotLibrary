# Theory: Lidar SLAM

---

## 1. Architecture

Lidar SLAM has three loosely coupled components:

```
lidar scans
    │
    ▼
Scan Matching ──► relative pose edge (odometry)
    │
    ▼
Pose Graph ──────► loop closure detection
    │                        │
    │            ◄── loop constraint (edge)
    ▼
Graph Optimization ──► globally consistent map
```

---

## 2. Scan Matching: ICP

**Iterative Closest Point (ICP)** estimates the rigid transform $T = (R, \mathbf{t})$ that
best aligns two point sets $\mathcal{P}$ (source) and $\mathcal{Q}$ (target).

Iterate until convergence:
1. **Associate:** for each point $\mathbf{p}_i \in \mathcal{P}$, find the closest
   $\mathbf{q}_i \in \mathcal{Q}$
2. **Solve:** minimize $\sum_i \|R\mathbf{p}_i + \mathbf{t} - \mathbf{q}_i\|^2$
   — closed-form via SVD of the cross-covariance matrix
3. **Apply:** transform $\mathcal{P} \leftarrow T(\mathcal{P})$

ICP convergences to a local minimum — a good initial estimate (from odometry) is required.
Point-to-line ICP (match each point to the nearest line segment, not point) converges
faster and is more accurate for 2D lidar structure.

---

## 3. NDT (Normal Distributions Transform)

NDT divides the map into cells and fits a Gaussian $\mathcal{N}(\boldsymbol{\mu}_k, \Sigma_k)$
to the points in each cell. Alignment score for a scan point $\mathbf{p}$:

$$s(\mathbf{p}) = \exp\!\left(-\frac{(\mathbf{p} - \boldsymbol{\mu}_k)^\top \Sigma_k^{-1} (\mathbf{p} - \boldsymbol{\mu}_k)}{2}\right)$$

Maximize total score over all scan points. NDT is more robust than ICP to partial overlaps
and is faster for dense maps.

---

## 4. Pose Graph

The **pose graph** is a sparse graph $G = (V, E)$:
- **Nodes** $V$: robot poses $\{\mathbf{x}_i\}$ at selected keyframe timestamps
- **Edges** $E$: relative transform measurements $(T_{ij}, \Omega_{ij})$ between poses $i$ and $j$,
  with information matrix $\Omega_{ij} = \Sigma_{ij}^{-1}$

**Odometry edges** are added continuously. **Loop closure edges** are added when the robot
recognizes a previously visited place (by comparing current scan to stored keyframe scans).

---

## 5. Pose Graph Optimization

Find pose estimates $\{\hat{\mathbf{x}}_i\}$ minimizing the sum of squared residuals:

$$\min_{\{\mathbf{x}_i\}} \sum_{(i,j) \in E} \mathbf{e}_{ij}^\top \Omega_{ij}\, \mathbf{e}_{ij}$$

where $\mathbf{e}_{ij} = \mathbf{x}_j \ominus (T_{ij} \oplus \mathbf{x}_i)$ is the error
between the measured and predicted relative pose ($\oplus, \ominus$ are pose composition operators).

Solved iteratively with Gauss-Newton or Levenberg-Marquardt. The sparsity of $\Omega$ (most
pose pairs have no direct edge) allows efficient sparse Cholesky factorization.

Modern solvers: **g2o**, **GTSAM**, **Ceres**. This module includes a minimal 2D solver.

---

## 6. Loop Closure Detection

When the robot returns to a previously visited area, this must be detected to add a
corrective constraint that removes accumulated drift.

Simple approach: compare the current scan to all stored keyframe scans using a scan
similarity metric (e.g. correlation of occupancy subgrids). Match candidates are verified
with scan matching; a match with low ICP residual is accepted as a loop closure.

---

## Further Reading

- Besl & McKay — *A Method for Registration of 3-D Shapes (ICP)*, PAMI 1992
- Biber & Strasser — *The Normal Distributions Transform*, IROS 2003
- Grisetti et al. — *A Tutorial on Graph-Based SLAM*, IEEE T-ITS 2010
- Dellaert & Kaess — *Factor Graphs for Robot Perception*, FNTRobotics 2017
