# Control Barrier Functions — Theory

## Overview

A Control Barrier Function (CBF) `h: ℝⁿ → ℝ` defines a **safe set** `C = {x | h(x) ≥ 0}`.
A controller is considered *safe* if it keeps the system state `x(t)` in `C` for all `t ≥ 0`.
CBFs provide a formal, mathematically guaranteed approach to safety — unlike heuristic
obstacle-avoidance penalties in cost functions.

## CBF Safety Condition

For a control-affine system `ẋ = f(x) + g(x)u`, a differentiable function `h` is a valid
CBF if there exists an extended class-K function `α` such that:

```
sup_{u ∈ U}  [ L_f h(x) + L_g h(x) · u ]  ≥  −α( h(x) )
```

where `L_f h` and `L_g h` are the Lie derivatives of `h` along `f` and `g` respectively.

When this condition holds for all `x ∈ C`, any controller satisfying it renders `C` forward-
invariant: if `x(0) ∈ C` then `x(t) ∈ C` for all `t ≥ 0`.

## Application to 2D Collision Avoidance

For a robot at position `p_r` and `N` obstacles at positions `p_i` with safety radius `r_safe`:

```
h_i(x) = ‖p_r − p_i‖² − r_safe²
```

The CBF safety filter solves a **Quadratic Program** at every control timestep:

```
min_u  ‖u − u_nom‖²
s.t.   L_g h_i(x) · u ≥ −L_f h_i(x) − α · h_i(x),   ∀i = 1…N
```

This **minimally deviates** from the nominal command `u_nom` while simultaneously
guaranteeing `h_i(x(t)) ≥ 0` for all obstacles `i`.

## Lie Derivatives for a Kinematic 2D Robot

For a pure-kinematic robot (no dynamics), `f(x) = 0` so `L_f h_i = 0`. With `u = [v, ω]ᵀ`
and the robot velocity in world frame `[v·cos θ, v·sin θ]ᵀ`:

```
∂h_i/∂p_r = 2(p_r − p_i)ᵀ

L_g h_i · u = 2(p_r − p_i)ᵀ · J_cmd · u
```

where `J_cmd` maps the control vector `u` to the robot's Cartesian velocity.

## Implementation Notes

- The QP is small (2 decision variables, ≤ N_obs constraints) — solved with Eigen's
  `colPivHouseholderQr` via the active-set reformulation. No external QP library required.
- The class-K function is chosen as `α(h) = k_α · h` (linear gain, configurable).
- Only obstacles within a user-defined detection radius are included as constraints to
  limit QP size in dense environments.
- The filter is a **decorator**: it wraps any `IController` without coupling to a specific
  controller implementation.áaattt

## References

- Ames et al., "Control Barrier Function Based Quadratic Programs for Safety Critical
  Systems," *IEEE Transactions on Automatic Control*, 2017.
- Zeng et al., "Safety-Critical Model Predictive Control with Discrete-Time Control
  Barrier Function," *American Control Conference*, 2021.
