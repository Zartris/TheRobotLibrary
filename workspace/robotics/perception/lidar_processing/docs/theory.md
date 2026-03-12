# Theory: Lidar Processing

---

## 1. The Beam Sensor Model

A 2D lidar emits $N$ beams at angles $\alpha_1, \ldots, \alpha_N$ and measures the range
$z_i$ to the first surface each beam hits. The true range $z_i^*$ (from ray casting
through the ground-truth map) differs from the measurement due to noise.

The **beam mixture model** (Thrun et al.) expresses the measurement likelihood as:

$$p(z_i \mid z_i^*) = w_{\text{hit}}\,\mathcal{N}(z_i;\, z_i^*, \sigma^2)
+ w_{\text{short}}\,\lambda e^{-\lambda z_i}\,\mathbf{1}[z_i \le z_i^*]
+ w_{\text{max}}\,\mathbf{1}[z_i = z_{\max}]
+ w_{\text{rand}}\,\frac{1}{z_{\max}}$$

The four weights sum to 1 and are tuned to the specific sensor.

---

## 2. Scan Filtering

Raw scans contain noise that degrades downstream processing. Common filters:

**Range clamping** — discard beams outside $[z_{\min}, z_{\max}]$:
$$z_i \leftarrow \begin{cases} \text{invalid} & z_i < z_{\min} \text{ or } z_i > z_{\max} \\ z_i & \text{otherwise} \end{cases}$$

**Median filter** (window $w$) — replace each reading with the median of its $w$ neighbors:
Effective against impulse noise (spurious close readings from glass, dust).

---

## 3. Scan Segmentation

A **segment** is a group of consecutive beams likely belonging to the same surface.
The simplest method: **distance-based segmentation**.

For consecutive beams $i$ and $i+1$, compute the Euclidean distance between their
Cartesian endpoints $(x_i, y_i)$ and $(x_{i+1}, y_{i+1})$:

$$d = \sqrt{(x_{i+1} - x_i)^2 + (y_{i+1} - y_i)^2}$$

Start a new segment when $d > d_{\text{threshold}}$ (gap between surfaces).

---

## 4. Line Extraction (RANSAC)

Extract lines from segments for structured environments (walls, corridors):

1. Randomly sample 2 points from the segment
2. Fit a line; count **inliers** (points within distance $\epsilon$)
3. Repeat for $K$ iterations; keep the best line
4. Refine with least-squares on inliers

RANSAC is robust to outliers inside the segment, which simple least-squares least-squares is not.

**Least-squares line fit** (after inlier selection): given inlier points $(x_j, y_j)$,
minimize $\sum_j (a x_j + b y_j + c)^2$ subject to $a^2 + b^2 = 1$ — reduces to an
eigenvector problem on the covariance matrix of the points.

---

## Further Reading

- Thrun, Burgard, Fox — *Probabilistic Robotics*, Chapter 5
- Borges & Aldon — *Line extraction in 2D range images for mobile robotics*, JIRS 2004
