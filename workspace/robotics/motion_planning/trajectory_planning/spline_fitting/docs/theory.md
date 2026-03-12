# Theory: Spline Fitting

---

## 1. Why Smooth the Path?

Raw waypoints from a grid planner (A\*, Dijkstra) are piecewise linear — the robot would have
to stop at every corner to change direction. Splines produce:
- **Continuous heading** (first derivative): no sudden direction changes
- **Bounded curvature** (second derivative): speeds can remain non-zero throughout
- **Predictable arc length**: enabling uniform-speed traversal

---

## 2. Cubic Spline (Natural / Clamped)

Given $N$ waypoints $\{(x_i, y_i)\}$, fit a piecewise cubic polynomial on each segment
$[t_i, t_{i+1}]$ (parametrize by chord length initially):

$$\mathbf{P}_i(t) = a_i + b_i(t-t_i) + c_i(t-t_i)^2 + d_i(t-t_i)^3$$

Conditions:
- Interpolation: $\mathbf{P}_i(t_i) = \mathbf{p}_i$, $\mathbf{P}_i(t_{i+1}) = \mathbf{p}_{i+1}$
- $C^1$ continuity: $\mathbf{P}_i'(t_{i+1}) = \mathbf{P}_{i+1}'(t_{i+1})$
- $C^2$ continuity: $\mathbf{P}_i''(t_{i+1}) = \mathbf{P}_{i+1}''(t_{i+1})$

This leaves $N-2$ free equations; the **natural spline** sets $\mathbf{P}''(t_0) = \mathbf{P}''(t_N) = 0$
(zero end curvature). This produces a symmetric tridiagonal system solvable in $O(N)$.

---

## 3. Catmull-Rom Spline

A local-support interpolating spline requiring no linear system solve. The tangent at
waypoint $i$ is computed directly from its neighbors:

$$\mathbf{m}_i = \frac{\mathbf{p}_{i+1} - \mathbf{p}_{i-1}}{2\,\Delta t}$$

Each segment cubic is then determined by the two endpoint values and the two tangents
via the Hermite basis. Only $C^1$ (tangent-continuous, not curvature-continuous).
Very easy to implement and well-suited for real-time path smoothing.

---

## 4. Bézier Curves

A degree-$n$ Bézier curve is defined by $n+1$ control points $\mathbf{P}_0, \ldots, \mathbf{P}_n$:

$$\mathbf{B}(t) = \sum_{i=0}^n \binom{n}{i}(1-t)^{n-i}t^i\,\mathbf{P}_i, \quad t \in [0,1]$$

**Key properties:**
- The curve lies within the convex hull of the control points.
- Only the endpoints are interpolated; inner points act as "attraction" handles.
- $\mathbf{B}'(0) = n(\mathbf{P}_1 - \mathbf{P}_0)$, so the tangent is easy to control.

**Composite Bézier:** chain degree-3 Béziers for long paths. For $G^2$ continuity
(continuous curvature), enforce:
- $G^1$: shared tangent direction at junctions
- $G^2$: matching curvature magnitude — requires constraining the second control point of
  the next segment relative to the first segment's last control point.

---

## 5. B-Splines

A B-spline with knot vector $\mathbf{t}$ and control points $\{\mathbf{d}_i\}$:
$$\mathbf{C}(t) = \sum_{i=0}^{n} N_{i,p}(t)\,\mathbf{d}_i$$

where $N_{i,p}(t)$ are the B-spline basis functions of degree $p$ computed via the
**Cox–de Boor recursion**:

$$N_{i,0}(t) = \begin{cases}1 & t_i \le t < t_{i+1}\\0 & \text{otherwise}\end{cases}$$
$$N_{i,p}(t) = \frac{t - t_i}{t_{i+p} - t_i}N_{i,p-1}(t) + \frac{t_{i+p+1}-t}{t_{i+p+1}-t_{i+1}}N_{i+1,p-1}(t)$$

**Advantages over Bézier:** local support (moving one control point affects only $p+1$ spans),
arbitrary length without raising degree, $C^{p-1}$ continuity.

Cubic B-splines ($p=3$) give $C^2$ continuity and are the standard choice for smooth path
representation in robotics.

---

## 6. Arc-Length Parametrization

Planners and controllers work with arc-length $s$, not the raw parameter $t$. The arc length is:
$$s(t) = \int_0^t \|\mathbf{C}'(\tau)\|\,d\tau$$

This integral has no closed form for most splines. In practice:
1. Sample the spline at many values of $t$, compute cumulative chord length → lookup table $s[i]$.
2. For a query $s$, binary search the table and linearly interpolate $t$.

---

## Further Reading

- de Boor — *A Practical Guide to Splines*, Springer 2001 (canonical B-spline reference)
- Piegl & Tiller — *The NURBS Book*, Springer 1995
- Lau et al. — *Improved Path Planning by Tightly Combining Lattice Graphs and Numeric Optimization*, T-ASE 2021
