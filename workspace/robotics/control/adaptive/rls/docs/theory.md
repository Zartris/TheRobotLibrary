# Recursive Least Squares — Theory

## Overview

Recursive Least Squares (RLS) is an online algorithm for estimating the parameters `θ` of
a linear regression model `y(k) = φ(k)ᵀ θ + ε(k)`, where `φ(k)` is the regressor vector
and `ε(k)` is noise. Unlike batch least squares, RLS processes one measurement at a time
and updates the estimate incrementally — suitable for real-time embedded systems.

## RLS Update Equations

Given a new measurement pair `(φ(k), y(k))`:

```
e(k)   = y(k) − φ(k)ᵀ θ̂(k−1)            (prediction error)
K(k)   = P(k−1) φ(k) / (λ + φ(k)ᵀ P(k−1) φ(k))   (Kalman gain)
θ̂(k)  = θ̂(k−1) + K(k) · e(k)            (parameter update)
P(k)   = (I − K(k) φ(k)ᵀ) P(k−1) / λ    (covariance update)
```

- `λ ∈ (0, 1]` is the **forgetting factor**: values < 1 weight recent data more heavily,
  enabling tracking of time-varying parameters. Typical values: `λ ∈ [0.95, 0.99]`.
- `P(k)` is the parameter covariance matrix (initialized large: `P(0) = α·I`).
- The algorithm reduces to standard Kalman filtering when `λ = 1`.

## Application to 2D Robot Dynamics

For a differential-drive robot, the linear motion model is:

```
m · a_x = F_drive − b_v · v
```

This gives the regression form:

```
a_x = (F_drive / m) − (b_v / m) · v
y(k) = φ(k)ᵀ θ,   φ = [F_drive, −v]ᵀ,   θ = [1/m, b_v/m]ᵀ
```

`RlsEstimator` solves for `θ` and recovers mass `m` and friction `b_v` for use in
model-based controllers.

## Numerical Stability

- **Covariance inflation:** If `trace(P) < ε_min`, reinflate `P ← αI` to avoid estimator
  lock-up (parameter estimates becoming frozen).
- **Upper bound on P:** Clamp `max eigenvalue(P) ≤ P_max` to prevent numerical overflow.
- **Directional forgetting** (optional): forget only in the direction of the latest regressor,
  preserving information in unexcited directions.

## References

- Ljung, *System Identification: Theory for the User*, 2nd ed., Prentice Hall, 1999.
- Söderström & Stoica, *System Identification*, Prentice Hall, 1989, ch. 3.
