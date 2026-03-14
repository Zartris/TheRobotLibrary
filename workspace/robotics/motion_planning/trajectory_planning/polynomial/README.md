# polynomial

Minimum-jerk and minimum-snap polynomial trajectory generation with Bézier curve variant.
Produces smooth, waypoint-constrained trajectories by solving a QP on polynomial
coefficients. Used in drone and arm motion planning where smooth higher-order derivatives
are critical.

## Components

- **`PolynomialTrajectory`** — multi-segment polynomial (minimum-jerk or minimum-snap)
- **`PolynomialSegment`** — single polynomial segment with position/velocity/acceleration sampling
- **`BezierCurve`** — de Casteljau algorithm; satisfies convex hull safety guarantee

## API

```cpp
generate(waypoints, durations) → PolynomialTrajectory
sample(t) → {position, velocity, acceleration}
```

## Dependencies

- `common` (logging)
- `Eigen3` (`LDLT` QP solver for polynomial coefficients)

## Usage

```cpp
#include <polynomial/polynomial_trajectory.hpp>
#include <polynomial/polynomial_config.hpp>

PolynomialConfig cfg;
cfg.order          = 5;   // 5th order → minimum-jerk
cfg.derivative_min = 3;   // minimise 3rd derivative (jerk)
cfg.durations      = {1.0, 1.0}; // seconds per segment

PolynomialTrajectory traj = PolynomialTrajectory::generate(waypoints, cfg,
                              common::getLogger("polynomial"));
auto [pos, vel, acc] = traj.sample(1.5); // sample at t=1.5s
```

## Milestone

Part of **M16 — Planning Upgrades II**.  
See `repo-plans/modules/polynomial.md` for full task checklist.
