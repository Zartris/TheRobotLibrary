# UKF Theory

## 1. Motivation

The Extended Kalman Filter (EKF) linearises nonlinear functions via first-order Taylor
expansion, introducing errors for strongly nonlinear systems. The Unscented Kalman Filter
(UKF) uses the **Unscented Transform** to capture the mean and covariance through a
minimal set of deterministic points — no Jacobians required.

## 2. Sigma Points (Merwe Scaled)

For an $n$-dimensional state, generate $2n+1$ sigma points:
$$\mathcal{X}_0 = \bar{x}, \quad
\mathcal{X}_i = \bar{x} + \left(\sqrt{(n+\lambda)P}\right)_i, \quad
\mathcal{X}_{i+n} = \bar{x} - \left(\sqrt{(n+\lambda)P}\right)_i$$
where $\lambda = \alpha^2(n + \kappa) - n$, $\alpha$ controls spread, $\kappa$ is a
secondary scaling parameter, and $\beta$ incorporates prior knowledge of the distribution
($\beta = 2$ is optimal for Gaussian).

## 3. Unscented Transform

Each sigma point is propagated through the nonlinear function $f$:
$$\mathcal{Y}_i = f(\mathcal{X}_i)$$

The weighted mean and covariance of the transformed points recover the predicted
distribution to third-order accuracy (vs. first-order for EKF).

## 4. Predict Step

Propagate sigma points through the process model, compute weighted mean $\bar{x}^-$ and
covariance $P^-$, add process noise $Q$.

## 5. Update Step

Propagate predicted sigma points through the measurement model $h$, compute predicted
measurement mean $\bar{z}$ and innovation covariance $S$, cross-covariance $P_{xz}$.
Kalman gain: $K = P_{xz} S^{-1}$. Update state and covariance.

## 6. Comparison to EKF

| Property | EKF | UKF |
|----------|-----|-----|
| Nonlinearity handling | First-order (Jacobian) | Third-order (sigma points) |
| Jacobian required | Yes | No |
| Computational cost | $O(n^2)$ | $O(n^2)$ |
| Accuracy on nonlinear | Lower | Higher |
