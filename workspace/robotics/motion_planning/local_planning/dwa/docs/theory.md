# Theory: Dynamic Window Approach (DWA)

See [`local_planning/docs/theory.md`](../docs/theory.md) for the full DWA algorithm,
objective function, and comparison with TEB. This file adds implementation detail on:
the dynamic window construction, sampling strategy, and tuning guidance.

---

## 1. Feasible Velocity Space

At each timestep, the robot has current state $(v_c, \omega_c)$. Given limits:
- $v \in [v_{\min}, v_{\max}]$, $\omega \in [-\omega_{\max}, \omega_{\max}]$
- Acceleration: $|v - v_c| \le a_v \cdot \Delta t$, $|\omega - \omega_c| \le a_\omega \cdot \Delta t$

The **dynamic window** $W$ is the rectangle:
$$W = [v_c - a_v\Delta t,\ v_c + a_v\Delta t] \times [-\omega_{\max},\ \omega_{\max}]$$
clipped to the global velocity limits.

**Admissibility constraint:** a velocity is admissible only if the robot can brake to a
full stop before hitting an obstacle:
$$v \le \sqrt{2 \cdot a_v \cdot \text{dist}(v, \omega)}, \quad \text{dist} = \text{minimum distance to obstacle along circular arc}$$

Inadmissible velocities are removed from $W$.

---

## 2. Objective Function Decomposition

$$G(v, \omega) = \sigma(\alpha \cdot \text{heading}(v,\omega) + \beta \cdot \text{clearance}(v,\omega) + \gamma \cdot v)$$

**heading** $\phi$: after simulating forward $\Delta t_{\text{sim}}$, the angular difference
between robot heading and bearing-to-goal. Normalized: $\phi \in [0, 1]$ where 1 = facing goal.

**clearance** $c$: minimum distance to any obstacle along the simulated arc.
Saturated at `clearance_max` (beyond which it has no effect):
$$c = \min(d_{\text{obs}}, c_{\text{max}}) / c_{\text{max}}$$

**velocity** term: current linear speed normalized to $v_{\max}$. Encourages forward motion
and prevents the robot from stopping unnecessarily.

$\sigma(\cdot)$: normalizes each term to $[0, 1]$ before weighting.

---

## 3. Simulation Horizon

For each sampled $(v, \omega)$, simulate the circular arc for $t_{\text{sim}}$ seconds
(typically 1–3 s). Check for collisions with the occupancy grid along the arc.

**Arc parametrization (diff-drive):**
$$x(\tau) = -\frac{v}{\omega}\sin\theta_0 + \frac{v}{\omega}\sin(\theta_0 + \omega\tau)$$
$$y(\tau) = \frac{v}{\omega}\cos\theta_0 - \frac{v}{\omega}\cos(\theta_0 + \omega\tau)$$
$$\theta(\tau) = \theta_0 + \omega\tau$$

If $\omega \approx 0$ (straight line): $x(\tau) = v\tau\cos\theta_0$, $y(\tau) = v\tau\sin\theta_0$.

---

## 4. Tuning Guidance

| Parameter | Effect | Typical range |
|---|---|---|
| $\alpha$ (heading weight) | Higher → bee-line to goal | 0.5–2.0 |
| $\beta$ (clearance weight) | Higher → more conservative | 0.1–1.0 |
| $\gamma$ (velocity weight) | Higher → faster motion | 0.1–0.5 |
| $v_{\min}$ | Allow reverse motion | 0 (or negative for reverse) |
| $\Delta t_{\text{sim}}$ | Lookahead depth | 1–3 s |
| Velocity resolution | Finer = better solution, slower | 10–20 samples per axis |

**Common tuning problems:**
- Robot oscillates: increase $\alpha$ relative to $\beta$.
- Robot hugs walls: increase $\beta$.
- Robot too slow: increase $\gamma$ or reduce $\beta$.
- Robot fails to reach goal: add a global path heading to the `heading` term (DWA+).

---

## 5. DWA with Global Plan (DWA+)

Vanilla DWA may get stuck in local minima (U-shaped obstacles). The standard fix:
feed the nearest waypoint on the global plan (from A\*) as the DWA goal, updated
every few cycles. The global planner provides topology; DWA handles local dynamics.

This is the pattern used in ROS `move_base` (nav stack).

---

## Further Reading

- Fox, Burgard & Thrun — *The Dynamic Window Approach to Collision Avoidance*, RA-M 1997 (original paper)
- Brock & Khatib — *High-Speed Navigation Using the Global Dynamic Window Approach*, ICRA 1999 (global plan integration)
- Marder-Eppstein et al. — *The Office Marathon: Robust Navigation in an Indoor Office Environment*, ICRA 2010 (ROS nav stack with DWA)
