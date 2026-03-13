# Recursive Least Squares (RLS) Parameter Estimator

`RlsEstimator` identifies robot dynamic parameters (mass, friction coefficient, linear drag)
online from input-output measurements, without requiring offline system identification.
The estimated parameters can be fed into `AdaptivePidController` or `MPCController` to
improve model accuracy over time.

## Theory

See `docs/theory.md`.

## Usage

```cpp
#include <rls/rls_estimator.hpp>

RlsConfig cfg{.forgetting_factor = 0.98, .initial_covariance = 1e4};
RlsEstimator estimator(cfg);

// Each control tick:
estimator.update(applied_twist, measured_acceleration);
auto params = estimator.getEstimatedParams();  // RobotModelParams
mpc_controller.setModelParams(params);
```
