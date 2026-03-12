# Theory: Obstacle Detection and Tracking

---

## 1. Detection Pipeline

```
raw scan / image
      │
      ▼
  Preprocessing    ← filter noise, remove ground plane
      │
      ▼
  Clustering       ← group nearby points belonging to the same object
      │
      ▼
  Bounding box     ← fit a geometric primitive (circle, AABB) per cluster
      │
      ▼
  Obstacle list    ← {position, extent, confidence}
      │
      ▼
  Tracker          ← associate detections across frames → persistent IDs + velocity
```

---

## 2. Lidar Clustering (DBSCAN)

**DBSCAN (Density-Based Spatial Clustering of Applications with Noise)** groups scan points
into clusters without requiring a fixed number of clusters.

Parameters: $\varepsilon$ (neighbourhood radius), $\text{MinPts}$ (minimum points to be a core point).

Algorithm:
1. For each unvisited point $p$: find all points within $\varepsilon$ (ε-neighbourhood)
2. If $|\text{neighbourhood}| \ge \text{MinPts}$: $p$ is a **core point** — start a cluster
3. Expand the cluster by recursively adding density-reachable points
4. Points reachable from a cluster boundary but not core points themselves are **border points**
5. Remaining points are **noise** (outliers)

DBSCAN is the standard choice for lidar clustering because it handles arbitrarily shaped
clusters and explicitly marks outliers as noise.

---

## 3. Multi-Object Tracking

### Data association

Given a set of new detections $\{d_j\}$ and existing tracks $\{t_i\}$, find the best
assignment. The simplest approach: **greedy nearest-neighbour** — assign each detection
to the closest track within a gating radius.

For more accuracy, use the **Hungarian algorithm** (also called the assignment problem)
which finds the globally optimal minimum-cost assignment in $O(n^3)$.

### Track lifecycle

- **Tentative:** new track, not yet confirmed (requires $k$ consecutive detections)
- **Confirmed:** stable track with a persistent ID
- **Lost:** track had no associated detection for $m$ frames → delete

### State estimation per track

Each track maintains a **Kalman filter** on state $\mathbf{x} = (x, y, v_x, v_y)^\top$:

$$\mathbf{x}_{k+1} = \begin{pmatrix} 1 & 0 & \Delta t & 0 \\ 0 & 1 & 0 & \Delta t \\ 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 1 \end{pmatrix} \mathbf{x}_k + \mathbf{w}_k$$

Measurement model: observe position only $\mathbf{z} = H\mathbf{x} + \mathbf{v}$ where
$H = [I_2 \mid 0_2]$.

This gives velocity estimates even though only position is directly measured.

---

## 4. Monocular Obstacle Detection

Detecting obstacles from a single camera without depth estimation is fundamentally
ambiguous. Common approaches:
- **Optical flow** — moving obstacles produce distinctive flow patterns against the static background
- **Learned detectors** — object detection networks (YOLO, SSD) produce bounding boxes; depth
  is estimated from object size priors or a learned depth network
- **Ground plane assumption** — obstacles are anything above the estimated ground plane

Monocular depth is inherently scale-ambiguous without calibration; a known baseline
(stereo or time-of-flight depth sensor) is more reliable for metric distance estimation.

---

## Further Reading

- Ester et al. — *A Density-Based Algorithm for Discovering Clusters in Large Spatial Databases with Noise*, KDD 1996 (DBSCAN)
- Kuhn — *The Hungarian Method for the Assignment Problem*, 1955
- Bewley et al. — *Simple Online and Realtime Tracking (SORT)*, ICASSP 2016
