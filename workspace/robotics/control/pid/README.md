# pid

PID (Proportional-Integral-Derivative) controller — the most widely used feedback
controller in robotics and automation.

**Standalone module.** Depends only on `common`. Copy this folder to any C++ project.

---

## What's in this module

- `PID` — single-axis PID controller with configurable gains, output clamping, and
  integral anti-windup
- `PID2D` — convenience wrapper pairing two `PID` instances for (x, y) or (v, ω) control

---

## Integration

```cmake
add_subdirectory(common)
add_subdirectory(pid)
target_link_libraries(your_target PRIVATE pid)
```

```cpp
#include <pid/pid.hpp>

pid::PID controller{/* kp */ 1.0, /* ki */ 0.1, /* kd */ 0.05};
controller.setOutputLimits(-1.0, 1.0);   // clamp output
controller.setIntegralLimit(5.0);        // anti-windup

double output = controller.update(setpoint - measurement, dt);
```

---

## Theory

See [`docs/theory.md`](docs/theory.md) for the PID equations, discretization, integral
windup, and tuning guidelines.
