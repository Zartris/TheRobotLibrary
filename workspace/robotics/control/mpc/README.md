# mpc

Model Predictive Control (MPC) — a receding-horizon optimal controller that explicitly
accounts for the robot's dynamics and respects kinematic and actuator constraints.

**Standalone module.** Depends only on `common`. Copy this folder to any C++ project.

---

## What's in this module

- `MPC` — nonlinear MPC for differential-drive robots (trajectory tracking)
- Configurable prediction horizon $N$, timestep $\Delta t$, cost weights
- Constraint support: max speed, max angular rate, max acceleration
- Uses a simple QP solver (included) for the linear MPC case; pluggable for nonlinear

---

## Integration

```cmake
add_subdirectory(common)
add_subdirectory(mpc)
target_link_libraries(your_target PRIVATE mpc)
```

```cpp
#include <mpc/mpc.hpp>

mpc::Config cfg;
cfg.horizon     = 10;
cfg.dt          = 0.1;
cfg.max_speed   = 1.0;   // m/s
cfg.max_omega   = 1.0;   // rad/s

mpc::MPC controller{cfg};
common::Twist cmd = controller.solve(current_state, reference_trajectory);
```

---

## Theory

See [`docs/theory.md`](docs/theory.md) for the MPC problem formulation, cost function
design, constraint handling, and the receding-horizon principle.
