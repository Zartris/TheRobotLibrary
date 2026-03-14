# Polynomial Trajectory Generation Theory

## 1. Problem Statement

Given $n$ waypoints $\{p_0, p_1, \ldots, p_{n-1}\}$ with associated segment durations
$\{T_0, T_1, \ldots, T_{n-2}\}$, find a piecewise polynomial trajectory that:
1. Passes through all waypoints (position continuity)
2. Satisfies boundary conditions on derivatives (velocity, acceleration at endpoints)
3. Minimises an integral cost on a chosen derivative order

## 2. Polynomial Segment

Each segment $s_k(t)$ is represented as a polynomial of degree $d$:
$$s_k(t) = \sum_{i=0}^{d} c_{k,i} \, t^i, \quad t \in [0, T_k]$$

Position, velocity, and acceleration at any $t$ are obtained by evaluating the polynomial
and its derivatives analytically.

## 3. Minimum-Jerk (5th Order)

Minimise $\int_0^T (\dddot{s})^2 \, dt$ (squared jerk). The unique 5th-degree polynomial
satisfying endpoint constraints on position, velocity, and acceleration is computed by
solving a $6 \times 6$ linear system — closed-form for a single segment.

## 4. Minimum-Snap (7th Order)

Minimise $\int_0^T (s^{(4)})^2 \, dt$ (squared snap, the 4th derivative). Used in
quadrotor motion planning where snap is directly related to rotor thrust. Requires 7th
(or 8th) degree polynomials to satisfy $C^3$ continuity across segments.

## 5. Multi-Segment QP

For $m$ segments with $n$ waypoints, the coefficient vector $\mathbf{c}$ is found by
minimising $\mathbf{c}^T H \mathbf{c}$ subject to linear equality constraints (waypoint
passage, continuity). Eigen's `LDLT` solver handles the small, dense system.

## 6. Bézier Curve (de Casteljau)

A degree-$d$ Bézier curve with control points $\{P_0, \ldots, P_d\}$ is defined by:
$$B(t) = \sum_{i=0}^{d} \binom{d}{i} (1-t)^{d-i} t^i P_i$$

The de Casteljau algorithm evaluates $B(t)$ numerically via repeated linear interpolation.
**Convex hull property:** all points of $B(t)$ lie within the convex hull of the control
points — providing a safety guarantee for collision avoidance.

## 7. Continuity at Segment Junctions

Minimum-jerk trajectories satisfy $C^2$ continuity (position, velocity, acceleration
continuous) at interior waypoints. Minimum-snap satisfies $C^3$ continuity. The QP
equality constraints enforce this automatically.
