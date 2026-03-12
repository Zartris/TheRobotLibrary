# Theory: Trajectory Planning

---

## 1. Path vs. Trajectory

| | Path | Trajectory |
|---|---|---|
| Definition | Geometric curve $\sigma(s)$, $s \in [0,1]$ | Time-indexed curve $\mathbf{x}(t)$, $t \in [0, T]$ |
| Contains velocity? | No | Yes — $\dot{\mathbf{x}}(t)$ known |
| Contains acceleration? | No | Yes if twice differentiable |
| Respects robot dynamics? | No | Yes (with constraints) |

Trajectory planning is the step that **bridges** path planning and control.

---

## 2. Parametrization

A geometric path $\sigma(s)$ (e.g. a spline) is re-parametrized by time by choosing
a velocity profile $v(s)$ or equivalently a time mapping $s(t)$:

$$\mathbf{x}(t) = \sigma(s(t))$$
$$\dot{\mathbf{x}}(t) = \sigma'(s)\,\dot{s}(t) = \sigma'(s)\,v(t)$$

The speed $v(t) = \dot{s}(t)$ is what the velocity profiler computes.

---

## 3. Velocity Profiling (Trapezoidal Profile)

The simplest profile: **accelerate** at $a_{\max}$ to $v_{\max}$, **cruise**, then
**decelerate** to zero at the goal.

Three phases:
1. **Acceleration:** $v(t) = a_{\max}\, t$, $s(t) = \tfrac{1}{2}a_{\max}t^2$
2. **Cruise:** $v = v_{\max}$, $s(t) = s_1 + v_{\max}(t - t_1)$
3. **Deceleration:** mirror of phase 1

If the path is too short to reach $v_{\max}$, the profile is a triangle (no cruise phase).

**Curvature constraint:** on a curve of radius $R$, the centripetal acceleration is
$v^2/R$. To limit lateral acceleration $a_{\perp} \le a_{\perp,\max}$, cap speed:
$$v_{\max}(s) = \sqrt{a_{\perp,\max} \cdot R(s)}$$

This means the robot automatically slows at corners.

---

## 4. S-Curve (Jerk-Limited) Profile

The trapezoidal profile has discontinuous acceleration (infinite jerk at transition points),
which causes mechanical stress and vibration. An **S-curve** profile limits jerk $j = \dot{a}$:

The profile has 7 phases: $j_{\max}$ ramp-up, constant max accel, $j_{\max}$ ramp-down,
cruise, mirror for deceleration. This produces smooth position, velocity, and acceleration
curves — important for precise manipulators and high-speed platforms.

---

## 5. Spline Fitting

A **spline** is a piecewise polynomial curve passing through (interpolating) or near (approximating)
a set of waypoints, with smoothness constraints at the junctions.

**Cubic spline (interpolating):** $C^2$ continuous (continuous position, slope, curvature).
Solve a tridiagonal linear system for the cubic coefficients — $O(N)$ for $N$ waypoints.

**Bézier curve:** parametric curve defined by control points. Degree-3 Bézier:
$$\mathbf{B}(t) = (1-t)^3\mathbf{P}_0 + 3(1-t)^2 t\mathbf{P}_1 + 3(1-t)t^2\mathbf{P}_2 + t^3\mathbf{P}_3$$

Bézier curves don't interpolate the control points (they are attracted to them), making
them useful for smooth path shaping without passing through every discrete waypoint.

**B-spline:** generalizes Bézier to many control points while maintaining local support
(moving one control point only affects the curve locally).

---

## 6. Timed Elastic Band (TEB)

TEB represents a trajectory as a sequence of robot poses $\{Q_i\}$ with associated time
gaps $\{\Delta T_i\}$ — an "elastic band" that is deformed by optimization.

Objective:
$$\min \sum_i \underbrace{f_{\text{kinematic}}(Q_i, \Delta T_i)}_{\text{dynamics}} + \sum_i \underbrace{f_{\text{obstacle}}(Q_i)}_{\text{clearance}} + f_{\text{goal}}(Q_N)$$

TEB jointly optimizes path shape and timing — unlike the two-step approach of fitting
a spline then applying a velocity profile. It produces trajectories that naturally slow
near obstacles and accelerate in free space. Implemented as a sparse nonlinear least-squares
problem (solved with $g^2o$).

---

## 7. Time-Optimal Trajectories

Given a path and kinematic/dynamic constraints, find the trajectory that minimizes total
execution time $T$. Formally, maximize $\dot{s}$ subject to:

$$\dot{s} \ge 0, \quad |\ddot{s}| \le a_{\max}(s), \quad \text{velocity constraints}$$

The **bang-bang** principle: the time-optimal control alternates between maximum
acceleration and maximum deceleration. The Pontryagin maximum principle guarantees this.

**Minimum-snap trajectories** (used in quadrotors, also applicable to mobile robots):
minimize the integral of snap (4th derivative of position) — the trajectory is a piecewise
polynomial of degree 7, solved as a QP. Produces very smooth motions suitable for
high-speed execution.

---

## Further Reading

- Biagiotti & Melchiorri — *Trajectory Planning for Automatic Machines and Robots* (comprehensive reference)
- Rösmann et al. — *Trajectory modification considering dynamic constraints of autonomous robots*, ROBOTIK 2012 (TEB)
- Mellinger & Kumar — *Minimum Snap Trajectory Generation and Control for Quadrotors*, ICRA 2011
- Bobrow et al. — *Time-Optimal Control of Robotic Manipulators Along Specified Paths*, IJRR 1985
