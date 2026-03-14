# ukf

Unscented Kalman Filter (UKF) with Merwe scaled sigma points. Provides a drop-in nonlinear
upgrade from the EKF by propagating a deterministic set of `2n+1` sigma points through the
(possibly nonlinear) process and measurement functions, avoiding Jacobian computation.

## Algorithm

- **Type:** Nonlinear state estimator
- **Interface:** `UKF : IStateEstimator`
- **Config:** `UKFConfig` — α, β, κ sigma-point parameters, Q, R

## Dependencies

- `common` (IStateEstimator interface, logging)
- `Eigen3`

## Usage

```cpp
#include <ukf/ukf.hpp>
#include <ukf/ukf_config.hpp>

UKFConfig cfg;
cfg.alpha = 1e-3;
cfg.beta  = 2.0;
cfg.kappa = 0.0;
cfg.Q = ...; // Process noise covariance
cfg.R = ...; // Measurement noise covariance

auto f = [](const VectorXd& x, const VectorXd& u) { /* process model */ };
auto h = [](const VectorXd& x)                   { /* measurement model */ };

UKF ukf(cfg, f, h, common::getLogger("ukf"));
ukf.predict(u);
ukf.update(z);
```

## Milestone

Part of **M14 — Advanced State Estimation II**.  
See `repo-plans/modules/ukf.md` for full task checklist.
