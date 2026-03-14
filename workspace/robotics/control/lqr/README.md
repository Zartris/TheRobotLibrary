# lqr

Discrete-time infinite-horizon Linear Quadratic Regulator (LQR). Solves the Discrete
Algebraic Riccati Equation (DARE) using Eigen and computes the optimal state-feedback
gain `K` such that `u = -Kx` minimises the quadratic cost `Σ (x'Qx + u'Ru)`.

## Algorithm

- **Type:** Optimal state-feedback controller
- **Interface:** `LQRController : IController`
- **Config:** `LQRConfig` — Q, R, A, B matrices; discretization dt

## Dependencies

- `common` (IController interface, logging)
- `Eigen3` (DARE solver via `GeneralizedSelfAdjointEigenSolver`)

## Usage

```cpp
#include <lqr/lqr_controller.hpp>
#include <lqr/lqr_config.hpp>

LQRConfig cfg;
cfg.Q = ...;   // State cost matrix
cfg.R = ...;   // Control cost matrix
cfg.A = ...;   // Discrete-time system matrix
cfg.B = ...;   // Input matrix
cfg.dt = 0.05; // Discretization timestep

LQRController ctrl(cfg, common::getLogger("lqr"));
auto cmd = ctrl.compute(state, setpoint);
```

## Milestone

Part of **M13 — Classical & Optimal Control**.  
See `repo-plans/modules/lqr.md` for full task checklist.
