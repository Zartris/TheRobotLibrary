# Time-Optimal Trajectory Planning

Computes the **fastest possible trajectory** along a given path, subject to
velocity, acceleration, and (optionally) jerk constraints.

Go here when total execution time matters: fast pick-and-place, high-speed
racing lines, drone flight, or precision gantry systems.

---

## Algorithms Implemented

| Algorithm | Description | When to Use |
|---|---|---|
| Trapezoidal TOPP | Double-pass velocity profiling (O(N)) | Default for mobile robots |
| Time-Optimal Path Parametrization (TOPP-RA) | Reachability analysis via LP | When torque/inertia constraints matter |
| Minimum-snap QP | Minimize 4th derivative; piecewise poly | Quadrotors, smooth high-speed paths |

---

## Interface

```cpp
#include "time_optimal/time_optimal_planner.hpp"

TimeOptimalConfig cfg;
cfg.v_max   = 2.0;
cfg.a_max   = 1.5;
cfg.method  = TOPPMethod::TOPP_RA;

TimeOptimalPlanner planner(cfg);
Trajectory traj = planner.plan(spline_path, constraints);
// constraints: per-waypoint velocity ceilings (curvature, obstacle proximity, etc.)
```

---

## File Layout

```
time_optimal/
├── CMakeLists.txt
├── README.md
├── docs/
│   └── theory.md          ← bang-bang control, TOPP-RA, minimum-snap
├── include/
│   └── time_optimal/
│       ├── time_optimal_planner.hpp
│       └── time_optimal_config.hpp
├── src/
│   └── time_optimal_planner.cpp
└── tests/
    └── test_time_optimal.cpp
```

---

## Dependencies

- `velocity_profiling/` — the `TOPP` method extends the double-pass algorithm here
- `spline_fitting/` — evaluates curvature $\kappa(s)$ along the path
- `common/`

---

## References

- Bobrow et al. — *Time-Optimal Control of Robotic Manipulators Along Specified Paths*, IJRR 1985
- Pham & Pham — *A New Approach to Time-Optimal Path Parameterization Based on Reachability Analysis*, T-RO 2018 (TOPP-RA)
- Mellinger & Kumar — *Minimum Snap Trajectory Generation and Control for Quadrotors*, ICRA 2011
