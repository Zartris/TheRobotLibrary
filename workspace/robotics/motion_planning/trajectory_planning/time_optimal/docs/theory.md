# Theory: Time-Optimal Trajectory Planning

---

## 1. Problem Formulation

Given a geometric path $\sigma(s)$, $s \in [0, L]$, find the time-parametrization $s(t)$
that minimizes $T$ (total traversal time) subject to:

$$\dot{s} \ge 0, \quad |\ddot{s}| \le a_{\max}(s), \quad \dot{s} \le v_{\max}(s)$$

This is a scalar optimal control problem in $(\dot{s}, \ddot{s})$ — note that all
constraints become scalar even though the robot moves in 2D/3D, because we have
**fixed the path** and only optimize speed along it (path-speed decoupling).

---

## 2. Pontryagin Minimum Principle — Bang-Bang Control

For the time-optimal problem without higher-order constraints, the Pontryagin Minimum
Principle shows the optimal control $\ddot{s}^*(t)$ is **bang-bang**: it switches
between maximum acceleration ($+a_{\max}$) and maximum deceleration ($-a_{\max}$).

This gives the trapezoidal profile (or triangle if the path is short). The switching
point is where the maximum-speed reachable by accelerating from the start equals the
minimum-speed reachable by decelerating from the end.

The optimal profile in the **phase plane** $(\dot{s}, s)$ consists of:
- **Acceleration curves:** $\dot{s}^2 = \dot{s}_0^2 + 2a_{\max}(s - s_0)$
- **Deceleration curves:** $\dot{s}^2 = \dot{s}_f^2 + 2a_{\max}(s_f - s)$

---

## 3. Time-Optimal Path Parameterization (TOPP)

For robot arms with joint torque limits, $a_{\max}(s)$ depends on path position and
velocity (inertia is configuration-dependent). The **TOPP** algorithm:

1. Compute the **Maximum Velocity Curve** (MVC): at each $s$, the maximum $\dot{s}$
   satisfying all constraints (torque limits, velocity limits).
2. Integrate forward (max acceleration) and backward (max deceleration) in the phase plane.
3. The optimal trajectory follows: max-accel until it hits the MVC, slides along the MVC,
   then max-decel to the goal.

Switch points (where $\dot{s}$ must change from accel to decel) are identified by
intersecting the forward and backward integration curves.

---

## 4. TOPP-RA (Reachability Analysis)

TOPP classically requires detecting switch points, which is numerically fragile when the
MVC has discontinuities. **TOPP-RA** (Pham & Pham 2018) reformulates the problem as a
sequence of **linear programs**: at each grid point $s_i$, compute the set of
reachable $\dot{s}$ values by solving a small LP.

Complexity: $O(N \cdot n_c)$ where $n_c$ is the number of constraints, vs. the
integration-based $O(N^2)$ worst case. TOPP-RA is the current practical standard.

---

## 5. Minimum-Snap Trajectories

Used in quadrotors and high-precision robots. Instead of fixing a path and optimizing
speed, **jointly** optimize the trajectory as a piecewise polynomial of degree $2k-1$
that minimizes the $k$-th derivative:

- $k=2$: minimize acceleration (min-accel)
- $k=4$: minimize snap = $d^4\mathbf{x}/dt^4$ (min-snap) — standard for quadrotors

**QP formulation:** choose polynomial coefficients $\mathbf{c}$ to minimize:

$$\min_{\mathbf{c}} \int_0^T \left\|\frac{d^k\mathbf{x}}{dt^k}\right\|^2 dt = \mathbf{c}^\top Q \mathbf{c}$$

subject to: continuity constraints at waypoints ($C^{2k-1}$), endpoint conditions.

The cost matrix $Q$ is computed analytically from the polynomial basis. This is an
unconstrained (or simple equality-constrained) QP with closed-form solution.

**Minimum jerk** ($k=3$) produces $C^5$-smooth trajectories suitable for robot
arms where smooth acceleration is critical.

---

## 6. Summary: When to Use What

| Method | Constraints | Optimality | Cost |
|---|---|---|---|
| Double-pass (trapezoidal) | $v_{\max}$, $a_{\max}$ | Global optimal (fixed path) | $O(N)$ |
| TOPP / TOPP-RA | $v_{\max}$, $a_{\max}$, torques | Global optimal (fixed path) | $O(N)$ |
| Min-snap QP | Continuity, waypoints | Minimum snap energy | $O(N^3)$ |
| TEB | Obstacles + kinematics | Local optimum | $O(\text{iter} \cdot N)$ |

---

## Further Reading

- Bobrow, Dubowsky & Gibson — *Time-Optimal Control of Robotic Manipulators Along Specified Paths*, IJRR 1985
- Pham & Pham — *A New Approach to Time-Optimal Path Parameterization Based on Reachability Analysis*, T-RO 2018
- Mellinger & Kumar — *Minimum Snap Trajectory Generation and Control for Quadrotors*, ICRA 2011
- Mueller, Hehn & D'Andrea — *A Computationally Efficient Motion Primitive for Quadrocopter Trajectory Generation*, T-RO 2015
