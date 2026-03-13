# Frenet-Serret Frame Controller — Theory

## Overview

The Frenet-Serret frame is a moving coordinate system attached to a point on a curve.
At any arc-length position `s` along a reference path, the frame has:

- **Tangent vector `T(s)`** — points along the path direction
- **Normal vector `N(s)`** — points toward the center of curvature (perpendicular to T)

The **lateral error** `e_lat` is the signed distance from the robot to the closest point
on the path (positive = left of path). The **heading error** `e_θ` is the difference
between the robot's heading and the path's tangent direction at the projection point.

## Error Dynamics

For a kinematic robot moving with speed `v` and angular rate `ω`, the Frenet error
dynamics are:

```
ė_lat = v · sin(e_θ) ≈ v · e_θ          (for small heading errors)
ė_θ   = ω − κ(s) · v                    (κ(s) = path curvature at projection)
```

This gives a cascaded linear system in `(e_lat, e_θ)`:

```
ë_lat = v · ė_θ = v · (ω − κ·v)
```

## Control Law

**Lateral control** (produces `ω`):
```
ω = κ(s) · v − k_lat · e_lat − k_d · ė_lat
             ^^^^^^^^^^^^^^^^^^^^^^^^^^^
             PD correction around zero lateral error
```

The feedforward term `κ(s)·v` compensates for path curvature, so the robot
naturally follows curves without requiring large lateral error to develop first.
This is the key advantage over pure-pursuit on curved paths.

**Longitudinal control** (produces `v`):
```
v = v_ref(s) + k_lon · e_lon + k_i · ∫e_lon dt
```

where `v_ref(s)` comes from the `IVelocityProfiler` TimedPath, and `e_lon` is the
signed along-track position error.

## Closest-Point Projection

The controller maintains the **projection index** `s*` — the arc-length parameter of
the point on the reference path closest to the robot:

```
s* = argmin_s ‖p_robot − p_path(s)‖
```

In practice, search is initialized from the previous projection and advances along the
path to avoid jumping backward.

## Comparison with Pure Pursuit

| Property | Pure Pursuit | Frenet Controller |
|---|---|---|
| Lateral error definition | Implicit (lookahead geometry) | Explicit signed distance |
| Longitudinal control | None (speed set externally) | Integrated PI loop |
| Curvature feedforward | Implicit via lookahead point | Explicit κ(s)·v term |
| Sensitivity to path noise | Low (lookahead averages) | Medium (projection sensitive) |
| Requires arc-length parameterization | No | Yes |

## References

- Coulter, *Implementation of the Pure Pursuit Path Tracking Algorithm*, CMU-RI-TR-92-01.
- Kong et al., "Kinematic and Dynamic Vehicle Models for Autonomous Driving Control
  Design," *IEEE Intelligent Vehicles Symposium*, 2015.
- Rajamani, *Vehicle Dynamics and Control*, Springer, 2012, ch. 3.
