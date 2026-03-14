# stanley

Stanley path-tracking controller from Stanford's DARPA Urban Challenge entry. Combines
heading error and speed-normalized cross-track error (CTE) into a single steering command.
Ackermann-friendly; also applicable to differential-drive via Ackermann approximation.

## Algorithm

- **Type:** Geometric path-following controller
- **Interface:** `StanleyController : IController`
- **Config:** `StanleyConfig` — gain k, velocity softening ε, max steering angle

Steering law: `δ = ψ_e + arctan(k · e_cte / (v + ε))`

where `ψ_e` is the heading error, `e_cte` is the cross-track error at the front axle,
`v` is the forward speed, and `ε` prevents division by zero at rest.

## Dependencies

- `common` (IController interface, logging)
- `Eigen3`

## Usage

```cpp
#include <stanley/stanley_controller.hpp>
#include <stanley/stanley_config.hpp>

StanleyConfig cfg;
cfg.k = 0.5;       // Cross-track gain
cfg.epsilon = 0.1; // Velocity softening
cfg.max_steer = 0.6; // Radians

StanleyController ctrl(cfg, common::getLogger("stanley"));
auto cmd = ctrl.compute(state, path);
```

## Milestone

Part of **M13 — Classical & Optimal Control**.  
See `repo-plans/modules/stanley.md` for full task checklist.
