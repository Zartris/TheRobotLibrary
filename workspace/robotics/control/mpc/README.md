# mpc

Model Predictive Control (MPC) — a receding-horizon optimal controller that explicitly
accounts for the robot's dynamics and respects kinematic and actuator constraints.

**Standalone module.** Depends only on `common`. Copy this folder to any C++ project.

---

## What's in this module

- `MPCController` — receding-horizon MPC for differential-drive robots (trajectory tracking)
- `MPCConfig` — configuration struct (horizon, prediction time, cost weights, velocity limits)
- Dual backend: acados NMPC solver (when `ROBOTLIB_HAS_ACADOS` is defined) with automatic
  fallback to an Eigen-based LTV-MPC condensed QP solver
- Configurable prediction horizon $N$, prediction time $T$, state/control/terminal cost weights
- Constraint support: max linear velocity, max angular rate

---

## Integration

```cmake
add_subdirectory(common)
add_subdirectory(mpc)
target_link_libraries(your_target PRIVATE mpc)
```

```cpp
#include <mpc/mpc_controller.hpp>

robotlib::MPCConfig cfg;
cfg.horizon          = 10;
cfg.predictionTime   = 1.0;   // seconds
cfg.maxLinearVelocity  = 1.0; // m/s
cfg.maxAngularVelocity = 2.0; // rad/s

robotlib::MPCController controller{cfg};

// Optional: provide a reference path for tracking
controller.setReferencePath(path);

robotlib::Twist cmd = controller.compute(current_pose, goal_pose, dt);
```

---

## Backends

| Backend | Condition | Description |
|---------|-----------|-------------|
| acados NMPC | `ROBOTLIB_HAS_ACADOS` defined | Full nonlinear MPC via acados SQP-RTI solver; use `tools/generate_solver.py` to regenerate C code |
| Eigen LTV-MPC | fallback (default) | Linearised time-varying MPC solved via Cholesky-factored condensed QP using Eigen |

The active backend is reported at construction (debug log) and queryable via `controller.backend()`.

---

## Theory

See [`docs/theory.md`](docs/theory.md) for the MPC problem formulation, cost function
design, constraint handling, and the receding-horizon principle.
