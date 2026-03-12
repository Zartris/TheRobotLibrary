# Theory: Ray Casting

---

## 1. The Problem

Given a 2D occupancy grid map $\mathbf{m}$ and a sensor origin $(x_s, y_s)$ oriented at
angle $\alpha$, find the range $z^*$ to the first occupied cell along the beam:

$$z^* = \min\bigl\{\, d \ge 0 \mid m\bigl(\lfloor x_s + d\cos\alpha \rfloor,\; \lfloor y_s + d\sin\alpha \rfloor\bigr) = 1 \bigr\}$$

If no occupied cell is hit within $z_{\max}$, return $z_{\max}$.

---

## 2. Bresenham Line Traversal

Bresenham's algorithm steps exactly through the grid cells a line passes through using
only integer arithmetic — no trigonometry per step.

For a ray from $(x_0, y_0)$ to $(x_1, y_1)$ in grid coordinates:
1. Compute $\Delta x = |x_1 - x_0|$, $\Delta y = |y_1 - y_0|$, step signs
2. Maintain an error accumulator; advance in the primary axis every step, secondary axis when error overflows
3. Check each visited cell for occupancy; stop at first hit

**Complexity:** $O(z_{\max} / \text{resolution})$ per ray — linear in range.

---

## 3. DDA (Digital Differential Analyzer)

DDA tracks when the ray crosses each grid boundary (x-boundary vs y-boundary) and jumps
directly to the next boundary, avoiding re-checking cells the ray only touches a corner of.

For each axis, compute the distance $t$ to the next grid boundary:
$$t_{x,\text{init}} = \frac{x_{\text{boundary}} - x_s}{\cos\alpha}, \quad \delta_{t_x} = \frac{1}{|\cos\alpha| \cdot \text{res}}$$

At each step, advance to $\min(t_x, t_y)$ and check the entered cell.

DDA is more accurate than Bresenham for ray casting because it never skips thin walls that
Bresenham would miss when traversing diagonally.

---

## 4. Noise Injection

A noiseless ray cast produces $z^*$ (ground truth). To simulate realistic sensor output,
add Gaussian noise:

$$z = z^* + \mathcal{N}(0, \sigma_{\text{hit}}^2)$$

For a complete beam mixture model including short readings and max-range returns, use the
full model described in [`../lidar_processing/docs/theory.md`](../lidar_processing/docs/theory.md).

---

## Further Reading

- Amanatides & Woo — *A Fast Voxel Traversal Algorithm for Ray Tracing*, Eurographics 1987
- Thrun, Burgard, Fox — *Probabilistic Robotics*, Chapter 5.3 (likelihood field model, ray casting)
