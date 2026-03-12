# Theory: Velocity Profiling

---

## 1. Problem Statement

Given:
- A geometric path $\sigma(s)$, $s \in [0, L]$ (arc-length parametrized)
- Kinematic constraints: $v_{\max}$, $a_{\max}$, $j_{\max}$

Find: a speed schedule $v^*(s) \ge 0$ such that the resulting trajectory respects all
constraints and minimizes traversal time $T = \int_0^L \frac{ds}{v(s)}$.

---

## 2. Trapezoidal Velocity Profile

The globally time-optimal profile for a straight segment (no curvature constraint) under
acceleration limit $a_{\max}$ is a **trapezoidal** shape: accelerate, cruise, decelerate.

**Acceleration phase** ($t \in [0, t_1]$):
$$v(t) = a_{\max}\,t, \quad s(t) = \tfrac{1}{2}a_{\max}t^2$$

Reaches $v_{\max}$ at $t_1 = v_{\max}/a_{\max}$, covering $s_1 = v_{\max}^2 / (2a_{\max})$.

**Cruise phase** ($t \in [t_1, t_2]$):
$$v(t) = v_{\max}, \quad s(t) = s_1 + v_{\max}(t - t_1)$$

**Deceleration phase:** mirror of acceleration phase from the end.

**Triangle profile** (path too short to reach $v_{\max}$): peak speed is
$v_{\text{peak}} = \sqrt{a_{\max}\,L}$.

---

## 3. S-Curve (Jerk-Limited) Profile

Limits $j = \dot{a}$ to $j_{\max}$. The profile has **7 phases**:

| Phase | Jerk | Acceleration |
|---|---|---|
| 1 | $+j_{\max}$ | Increases from 0 to $a_{\max}$ |
| 2 | 0 | Constant $a_{\max}$ |
| 3 | $-j_{\max}$ | Decreases from $a_{\max}$ to 0 |
| 4 | 0 (cruise) | 0 |
| 5 | $-j_{\max}$ | Decreases from 0 to $-a_{\max}$ |
| 6 | 0 | Constant $-a_{\max}$ |
| 7 | $+j_{\max}$ | Increases from $-a_{\max}$ to 0 |

Parameters:
- Time to reach max accel: $t_j = a_{\max}/j_{\max}$
- Additional time at constant accel: $t_a = v_{\max}/a_{\max} - t_j$

If $t_a < 0$, the profile never reaches $a_{\max}$: $a_{\text{peak}} = \sqrt{j_{\max}\,v_{\max}}$.

---

## 4. Curvature Constraint (Speed on Curves)

For a path of curvature $\kappa(s) = 1/R(s)$, the centripetal acceleration is
$a_c = v^2\,\kappa$. To keep $a_c \le a_{\perp,\max}$:

$$v_{\max}(s) = \sqrt{\frac{a_{\perp,\max}}{\kappa(s)}} = \sqrt{a_{\perp,\max}\,R(s)}$$

**Algorithm — curvature-aware profile:**
1. Sample $v_{\max}(s)$ along the path (curvature constraint).
2. **Forward pass:** propagate the maximum achievable speed forwards:
   $v_{i+1} \le \sqrt{v_i^2 + 2\,a_{\max}\,\Delta s}$
3. **Backward pass:** propagate backwards for deceleration feasibility:
   $v_i \le \sqrt{v_{i+1}^2 + 2\,a_{\max}\,\Delta s}$
4. Take the element-wise minimum of all three constraints.
5. Integrate $v(s)$ to obtain $t(s)$ and hence $\mathbf{x}(t)$.

This double-pass algorithm is $O(N)$ and produces the time-optimal profile under
all constraints simultaneously.

---

## 5. Integration to Trajectory

Once $v(s)$ is known, time is recovered by integration:
$$t(s) = \int_0^s \frac{d\sigma}{v(\sigma)}$$

This can be approximated numerically:
$$\Delta t_i = \frac{\Delta s_i}{\bar{v}_i}, \quad t_i = \sum_{k<i} \Delta t_k$$

The trajectory is then sampled at a fixed control rate $\Delta t_{\text{ctrl}}$ using
linear or cubic interpolation in $s(t)$.

---

## Further Reading

- Biagiotti & Melchiorri — *Trajectory Planning for Automatic Machines and Robots*, Springer 2008 (definitive reference)
- Kunz & Stilman — *Time-Optimal Trajectory Parameterization for Redundant Robots*, 2012 (double-pass O(N) algorithm)
