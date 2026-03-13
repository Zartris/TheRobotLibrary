# Frenet-Serret Frame Controller

`FrenetController` tracks a reference path by decomposing tracking error into
**lateral** (cross-track) and **longitudinal** (along-track) components in the
Frenet-Serret reference frame attached to the path. This decoupling produces
smoother, more intuitive path following than controllers that operate in the
global frame.

## Theory

See `docs/theory.md`.

## Usage

```cpp
#include <frenet/frenet_controller.hpp>

FrenetConfig cfg{
    .lat_kp = 1.2, .lat_kd = 0.3,
    .lon_kp = 0.8, .lon_ki = 0.05,
    .lookahead_dist = 1.0
};
FrenetController ctrl(cfg);
auto twist = ctrl.compute(current_state, goal, dt);
// Note: controller internally projects the robot onto the reference path
// from the ILocalPlanner path input
```

## Difference from Pure Pursuit

| Pure Pursuit | Frenet Controller |
|---|---|
| Geometric: targets a lookahead point | Analytic: minimizes lateral + heading errors |
| No explicit longitudinal control | Separate longitudinal loop for speed tracking |
| Minimal computation | Requires closest-point projection onto path |
| Works well for smooth paths | Works well for any C1-continuous path |
