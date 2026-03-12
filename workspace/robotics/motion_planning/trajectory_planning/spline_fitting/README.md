# Spline Fitting

Replaces a raw sequence of waypoints (typically from a global planner) with a smooth,
differentiable curve that can be uniformly sampled for trajectory planning.

---

## Algorithms Implemented

| Algorithm | Interpolates? | Continuity | Notes |
|---|---|---|---|
| Natural cubic spline | Yes | $C^2$ | Simple; slight oscillations possible |
| Catmull-Rom spline | Yes | $C^1$ | No system solve; always passes through points |
| Bézier (cubic) | No — approximates | $C^\infty$ per segment | Flexible; use for short path sections |
| B-spline (cubic) | No | $C^2$ | Local support; preferred for long paths |

---

## Interface

```cpp
#include "spline_fitting/spline_fitter.hpp"

SplineFitter fitter;
fitter.setPoints(waypoints);           // std::vector<Eigen::Vector2d>
fitter.setMethod(SplineMethod::BSPLINE);

SplinePath path = fitter.compute();
// Use: path.eval(s)    → Eigen::Vector2d  (position at arc-length s)
//      path.evald(s)   → Eigen::Vector2d  (first derivative = tangent)
//      path.evaldd(s)  → Eigen::Vector2d  (second derivative = curvature numerator)
//      path.length()   → double
```

---

## File Layout

```
spline_fitting/
├── CMakeLists.txt
├── README.md
├── docs/
│   └── theory.md          ← spline math, G2 continuity explanation
├── include/
│   └── spline_fitting/
│       ├── spline_fitter.hpp
│       └── spline_path.hpp
├── src/
│   ├── spline_fitter.cpp
│   └── spline_path.cpp
└── tests/
    └── test_spline_fitting.cpp
```

---

## Dependencies

- `common/` — `Pose2D`
- Eigen (header-only linear algebra)

---

## Copying This Module

Copy the entire `spline_fitting/` folder. If Eigen is available system-wide, no other
dependencies needed beyond `common/`.
