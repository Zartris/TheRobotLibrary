# Gain Scheduling — Adaptive PID

`AdaptivePidController` extends classical PID control by automatically adjusting
`K_p`, `K_i`, and `K_d` online based on observed tracking performance.

## Theory

See `docs/theory.md`.

## Usage

```cpp
#include <gain_scheduling/adaptive_pid_controller.hpp>

AdaptivePidConfig cfg{
    .base_kp = 1.5, .base_ki = 0.1, .base_kd = 0.05,
    .adapt_rate = 0.01, .error_window = 20
};
AdaptivePidController ctrl(cfg);
auto twist = ctrl.compute(current_state, goal, dt);
```
