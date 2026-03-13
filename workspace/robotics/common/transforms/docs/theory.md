# Rigid-Body Transforms: SE(2), SE(3), and SO(3)

## Overview

This module provides C++ value types for the three most important Lie groups in
robotics: **SE(2)**, **SE(3)**, and **SO(3)**.  Understanding why these groups
exist, how their operations work, and where they appear in robotics code is
essential for writing correct frame-transform logic.

---

## Mathematical Background

### Groups and Lie Groups

A **group** is a set _G_ with a binary operation `∘` satisfying:
1. **Closure**: `a ∘ b ∈ G` for all `a, b ∈ G`
2. **Associativity**: `(a ∘ b) ∘ c = a ∘ (b ∘ c)`
3. **Identity**: There exists `e ∈ G` such that `e ∘ a = a ∘ e = a`
4. **Inverse**: For each `a ∈ G`, there exists `a⁻¹ ∈ G` with `a ∘ a⁻¹ = e`

A **Lie group** is additionally a smooth differentiable manifold where the group
operations are smooth maps.  SE(2), SE(3), and SO(3) are all Lie groups.

---

## SO(3) — Special Orthogonal Group in 3D

### Definition

SO(3) is the group of all **3×3 orthogonal matrices with determinant +1**:

```
SO(3) = { R ∈ ℝ³ˣ³  |  R^T R = I,  det(R) = +1 }
```

These are exactly the **proper rotations** in 3D (no reflections, no scaling).

### Homogeneous Matrix Representation

A rotation is stored as a 3×3 matrix:

```
R = [ r₁₁  r₁₂  r₁₃ ]
    [ r₂₁  r₂₂  r₂₃ ]
    [ r₃₁  r₃₂  r₃₃ ]
```

Applying `R` to a 3D point `p` gives the rotated point: `p' = R p`.

### Group Operations

| Operation | Formula |
|-----------|---------|
| Compose   | `R_ab = R_a · R_b` (matrix multiply) |
| Inverse   | `R⁻¹ = R^T` (transpose, because R is orthogonal) |
| Identity  | `I₃` (3×3 identity matrix) |

### Quaternion Representation

This library stores SO(3) as a **unit quaternion** `q = (w, x, y, z)` with
`|q| = 1`.  Quaternions avoid gimbal lock and are numerically stable for
composition.  The relationship to axis-angle is:

```
q = cos(θ/2) + sin(θ/2)(x·i + y·j + z·k)
```

where `(x, y, z)` is the unit rotation axis and `θ` is the rotation angle.

Quaternion composition mirrors matrix multiplication:

```
q_ab = q_a ⊗ q_b
```

The inverse of a unit quaternion is its conjugate: `q⁻¹ = (w, -x, -y, -z)`.

### Euler Angles (Roll-Pitch-Yaw)

Roll-Pitch-Yaw (RPY) uses the **intrinsic ZYX** convention:

```
R = Rz(yaw) · Ry(pitch) · Rx(roll)
```

This is the standard aerospace and robotics convention where:
- **Roll** `φ` — rotation about X (body forward axis)
- **Pitch** `θ` — rotation about Y (body left axis)
- **Yaw** `ψ` — rotation about Z (body up axis)

**Caution:** Euler angles suffer from **gimbal lock** at `pitch = ±90°`.  For
numerical computation, always prefer quaternions or rotation matrices; only
convert to RPY for human-readable output.

---

## SE(2) — Special Euclidean Group in 2D

### Definition

SE(2) is the group of **2D rigid-body transforms**: a planar translation combined
with a rotation.  A pose `T ∈ SE(2)` maps a child frame to a parent frame.

### Homogeneous Matrix Representation

SE(2) elements are represented as 3×3 homogeneous matrices:

```
T = [ cos θ  -sin θ   tx ]
    [ sin θ   cos θ   ty ]
    [   0       0      1 ]
```

This embeds the 2D rotation and translation into a matrix that acts on
**homogeneous coordinates** `p̄ = [px, py, 1]^T`:

```
p'_bar = T · p_bar
```

Expanding: `p' = R₂ p + t`, where `R₂` is the 2×2 rotation block and `t` is
the translation vector.

### Group Operations

| Operation | Formula |
|-----------|---------|
| Compose   | `T_ab = T_a · T_b` |
| Inverse   | `T⁻¹ = [-R^T t; 0 1]`, or `T⁻¹ = [-R^T · t,  R^T]` (expanded) |
| Identity  | `I₃` with zero translation and zero rotation |

The explicit formulas implemented in this library:

**Compose** `T_a ∘ T_b`:
```
x_result  = x_a + cos(θ_a) · x_b − sin(θ_a) · y_b
y_result  = y_a + sin(θ_a) · x_b + cos(θ_a) · y_b
θ_result  = θ_a + θ_b
```

**Inverse** of `T = (x, y, θ)`:
```
x_inv = -(cos θ · x + sin θ · y)
y_inv = -(-sin θ · x + cos θ · y)
θ_inv = -θ
```

### Interpolation on SE(2)

Linear interpolation (LERP) on SE(2) blends translation linearly and uses
**shortest-arc angular interpolation** for the rotation:

```
T(t) = (x_a + t(x_b − x_a),
        y_a + t(y_b − y_a),
        θ_a + t · wrap(θ_b − θ_a))
```

where `wrap(·)` normalizes the angle difference to `(−π, π]`.

This is not a geodesic on the SE(2) manifold (which would require matrix
exponential/logarithm), but it is the standard robotics approximation for small
angular differences and is numerically cheap.

---

## SE(3) — Special Euclidean Group in 3D

### Definition

SE(3) is the group of **3D rigid-body transforms**: a 3D translation combined
with a 3D rotation.  This is the fundamental type for expressing the pose of
any rigid body (robot, sensor, obstacle) in 3D space.

### Homogeneous Matrix Representation

SE(3) elements are represented as 4×4 homogeneous matrices:

```
T = [ R    t  ]  =  [ r₁₁ r₁₂ r₁₃  tx ]
    [ 0    1  ]     [ r₂₁ r₂₂ r₂₃  ty ]
                    [ r₃₁ r₃₂ r₃₃  tz ]
                    [  0   0   0    1  ]
```

where `R ∈ SO(3)` is the rotation block and `t ∈ ℝ³` is the translation vector.

Applying `T` to a 3D point `p`:

```
p'_bar = T · p_bar     (homogeneous multiply)
p'     = R p + t        (explicit form)
```

### Group Operations

| Operation | Formula |
|-----------|---------|
| Compose   | `T_ab = T_a · T_b` |
| Inverse   | `T⁻¹ = [R^T,  −R^T t; 0, 1]` |
| Identity  | 4×4 identity with zero translation and identity rotation |

The explicit formulas:

**Compose** `T_a ∘ T_b`:
```
t_result = t_a + R_a · t_b
R_result = R_a · R_b
```

**Inverse** of `T = (t, R)`:
```
t_inv = −R^T · t
R_inv = R^T
```

In quaternion form:
```
t_inv = −q_inv ⊗ t ⊗ q        (conjugate action)
q_inv = conjugate(q)
```

### This Library's Storage

SE3 stores:
- `translation` — `Eigen::Vector3d`
- `rotation`    — `Eigen::Quaterniond` (unit quaternion, always normalised)

This avoids redundant storage (a 4×4 matrix would store 16 doubles; we use 7)
and keeps composition numerically stable (quaternion products do not accumulate
drift as quickly as repeated float matrix multiplies).

### Projection to SE(2)

A 3D pose can be projected to a 2D ground-plane pose by:
1. Taking the `(x, y)` components of the translation.
2. Extracting the **yaw** angle from the rotation (rotation about the global Z axis).

This is valid when the robot operates primarily in the horizontal plane (ground
robots, UAVs at constant altitude) and is the standard way to bridge a full 3D
state estimator output (e.g. from EKF fusing IMU + odometry) to a 2D motion
planner.

---

## Why These Groups Matter in Robotics

### Frame Transforms

Every sensor and body part has a **pose** relative to some reference frame.
SE(3) transforms compose these poses:

```
T_world_camera = T_world_base · T_base_camera
```

Reading right-to-left: "express camera in base frame, then express base in
world frame."  This is just matrix multiplication on the homogeneous matrices
(or `compose()` in this library).

### Coordinate Change

To express a point `p_camera` (in the camera frame) in world coordinates:

```
p_world = T_world_camera.transformPoint(p_camera)
```

To go the other way (world to camera):

```
p_camera = T_world_camera.inverse().transformPoint(p_world)
```

### Odometry and Dead Reckoning

A robot's absolute pose `T_k` is updated by composing with the relative
incremental odometry transform `ΔT`:

```
T_{k+1} = T_k ∘ ΔT
```

This is a direct `compose()` call and is the inner loop of any odometry or
dead-reckoning system.

### Sensor Fusion

EKF and UKF state estimators maintain the robot pose as an SE(2) or SE(3)
element.  Predicted poses come from `compose()`; corrections involve computing
the **pose error** `T_error = T_predicted.inverse().compose(T_measured)`.

---

## Numerical Considerations

| Issue | Mitigation in this library |
|-------|---------------------------|
| Quaternion drift (‖q‖ ≠ 1 after many composes) | `normalized()` called on every compose result |
| Gimbal lock in RPY extraction | Use quaternions internally; only convert RPY for output |
| Angle wrap-around in SE2 | `normalizeAngle()` applied to all theta values |
| Floating-point epsilon in equality | `operator==` uses 1e-9 tolerance |

---

## References

1. **Lynch & Park, *Modern Robotics: Mechanics, Planning, and Control*,
   Cambridge University Press, 2017** — Chapter 3 covers SO(3) and SE(3)
   rigorously with Lie group theory; Chapter 4 covers forward kinematics.

2. **Barfoot, *State Estimation for Robotics*, Cambridge University Press,
   2017** — Chapter 7 discusses perturbation theory on SO(3) and SE(3) for EKF.

3. **Eigen documentation** — https://eigen.tuxfamily.org/dox/classEigen_1_1Quaternion.html
   for the `Quaterniond` API used internally.

4. **Diebel, *Representing Attitude: Euler Angles, Unit Quaternions, and
   Rotation Vectors*, 2006** — definitive reference for angle conventions.
