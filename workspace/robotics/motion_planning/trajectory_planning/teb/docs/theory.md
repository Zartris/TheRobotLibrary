# Theory: Timed Elastic Band (TEB)

---

## 1. Elastic Band Concept

An **elastic band** (Quinlan & Khatib 1993) is a path represented as a sequence of
configuration-space waypoints $\{Q_i\}$ connected by virtual springs. The band is
"stretched" toward the goal and "repelled" by obstacles until it finds a near-optimal
collision-free path.

**Timed** elastic band extends this idea: each waypoint also carries a time gap $\Delta T_i$
so the sequence fully defines a trajectory, not just a path.

The band: $\mathcal{B} = \{Q_1, \Delta T_1, Q_2, \Delta T_2, \ldots, Q_n, \Delta T_n, Q_{n+1}\}$

---

## 2. Optimization Problem

TEB solves the multi-objective minimization:

$$\min_{\mathcal{B}} \sum_k \gamma_k f_k(\mathcal{B})$$

where the objective terms $f_k$ are:

| Term | Purpose |
|---|---|
| $f_{\text{time}}(\Delta T_i)$ | Minimize total traversal time |
| $f_{\text{kinematics}}(Q_i, Q_{i+1}, \Delta T_i)$ | Enforce velocity / acceleration limits |
| $f_{\text{obstacle}}(Q_i)$ | Maintain clearance from obstacles |
| $f_{\text{shortest}}(Q_i, Q_{i+1})$ | Penalize path length |

**Velocity constraint** (for diff-drive):
$$v_i = \frac{\|Q_{i+1} - Q_i\|}{\Delta T_i} \le v_{\max}$$
$$\omega_i = \frac{\Delta\theta_i}{\Delta T_i} \le \omega_{\max}$$

**Obstacle term** (repulsive potential):
$$f_{\text{obs}}(Q_i) = \begin{cases}\left(\frac{d_{\min}(Q_i) - d_{\text{safe}}}{\epsilon}\right)^2 & d < d_{\text{safe}} + \epsilon\\0 & \text{otherwise}\end{cases}$$

where $d_{\min}(Q_i)$ is the clearance to the nearest obstacle.

---

## 3. Sparse Graph Formulation

Each $f_k$ involves only a small number of consecutive poses. This makes the Hessian
of the full problem **sparse** — ideal for the $g^2o$ framework (or any sparse
Gauss-Newton solver).

Variables: $\mathbf{z} = (Q_1, \Delta T_1, \ldots, Q_n, \Delta T_n)$

Gauss-Newton update:
$$H\, \delta\mathbf{z} = -\mathbf{b}, \quad H = J^\top\Sigma^{-1}J, \quad \mathbf{b} = J^\top\Sigma^{-1}\mathbf{e}$$

where $\mathbf{e}$ is the vector of all residuals and $J$ their Jacobian. $H$ is sparse and
positive semi-definite. Solved efficiently via sparse Cholesky.

---

## 4. Adaptive Pose Sampling (Hysteresis)

The number of poses in the band is not fixed. After each optimization iteration:
- If two consecutive poses are too far apart ($\|\Delta Q\| > d_{\max}$), **insert** a new pose.
- If too close (highly redundant), **remove** the pose.

Hysteresis prevents oscillation: use a larger threshold to remove than to insert.
A fixed $\Delta T_{\text{ref}}$ is the target time gap; after insertion/deletion the
$\Delta T_i$ values are rescaled accordingly.

---

## 5. Topological Planning (Multiple Homotopy Classes)

TEB can explore **multiple topologically distinct paths** simultaneously (e.g., passing
an obstacle on the left vs. the right) by maintaining several bands in parallel and
selecting the one with lowest cost. This avoids the local minima typical of single-path
elastic band optimizers.

The homotopy class of a band is identified by examining which obstacles the band
"winds around" — recorded efficiently via a complex-valued line integral (h-signature).

---

## 6. TEB vs. Two-Step (Spline + Velocity Profiler)

| Aspect | TEB | Spline + Velocity Profiler |
|---|---|---|
| Obstacle-aware? | Yes, at every iteration | No (path fixed before profiling) |
| Simultaneous path+time opt? | Yes | No (sequential) |
| Dynamic obstacle replanning | Yes (replan online) | Hard (must redo everything) |
| Computational cost | Higher (iterative) | Low (one-shot) |
| Use when | Dynamic environments, narrow passages | Open environments, fast execution |

---

## Further Reading

- Rösmann et al. — *Trajectory modification considering dynamic constraints of autonomous robots*, ROBOTIK 2012
- Rösmann et al. — *Efficient trajectory optimization using a sparse model*, ECC 2013
- Rösmann et al. — *Kinodynamic trajectory optimization and control for car-like robots*, IROS 2017
- Quinlan & Khatib — *Elastic bands: Connecting path planning and control*, ICRA 1993
