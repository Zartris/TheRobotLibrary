# Velocity Profiling

Converts a geometric path into a time-parametrized trajectory by assigning a velocity
schedule $v(s)$ that respects kinematic and dynamic limits.

---

## Algorithms Implemented

| Algorithm | Continuity | Jerk-limited | Notes |
|---|---|---|---|
| Trapezoidal | $C^1$ velocity | No | Simplest; use for basic mobile robots |
| S-curve | $C^2$ accel | Yes | 7-phase; use for precision platforms |
| Curvature-aware | $C^1$ with $v_{\max}(s)$ | Optional | Scales speed on curves |

---

## Interface

```cpp
#include "velocity_profiling/velocity_profiler.hpp"

VelocityProfiler profiler(VelocityConstraints{
    .v_max = 1.5,         // m/s
    .a_max = 0.8,         // m/s²
    .j_max = 2.0,         // m/s³  (only used in S-curve mode)
    .a_lateral_max = 1.0  // m/s²  (curvature constraint)
});

// path: sequence of (x,y) waypoints with arc-length parameter s
Trajectory traj = profiler.compute(path, ProfileMode::SCURVE);
// Returns: Trajectory with x(t), y(t), v(t), a(t), theta(t) sampled at dt
```

---

## File Layout

```
velocity_profiling/
├── CMakeLists.txt
├── README.md
├── docs/
│   └── theory.md          ← trapezoidal, S-curve, curvature constraint math
├── include/
│   └── velocity_profiling/
│       ├── velocity_profiler.hpp
│       ├── trajectory.hpp
│       └── constraints.hpp
├── src/
│   ├── velocity_profiler.cpp
│   └── trajectory.cpp
└── tests/
    └── test_velocity_profiling.cpp
```

---

## Dependencies

- `common/` — `Pose2D`, `Twist`

---

## Copying This Module

```
velocity_profiling/
└── (entire folder)
```

Implement the interface in `velocity_profiler.hpp`, delete what you don't need.
