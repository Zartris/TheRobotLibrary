# mppi

Model Predictive Path Integral (MPPI) controller. An advanced stochastic MPC variant that handles non-convex costs by sampling N control trajectories as Gaussian perturbations of a nominal sequence, propagating each through a configurable motion model, and computing the importance-weighted optimal control update.

**Milestone:** M18 — Advanced Nonlinear Control  
**Status:** Scaffold only — awaiting implementation

## Features

- `MPPIController : IController` — drop-in replacement for any `IController`
- Configurable: N rollouts, horizon H, temperature λ (exploration/exploitation tradeoff), noise covariance Σ, timestep dt
- `MPPICostFn = std::function<double(state, control)>` — user-supplied cost function
- Configurable motion model via `std::function<VectorXd(VectorXd, VectorXd)>`
- Importance weight update: `u* = Σ (w_k · ε_k)` where `w_k = exp(−J_k / λ)`

## Usage

```cpp
#include <mppi/mppi_controller.hpp>
#include <mppi/mppi_config.hpp>

MPPIConfig config{.N = 512, .H = 20, .lambda = 0.1, .dt = 0.05};
auto cost = [](const Eigen::VectorXd& state, const Eigen::VectorXd& ctrl) { ... };
MPPIController controller{config, cost, motionModel, logger};

Eigen::VectorXd u = controller.compute(currentState, referenceState);
```

## Dependencies

- `common` (logging, types)
- Eigen3 (matrix math)
