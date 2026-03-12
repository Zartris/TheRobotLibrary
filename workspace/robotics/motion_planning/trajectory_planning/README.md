# trajectory_planning

Adding time, velocity, acceleration, and smoothness to a geometric path.

A geometric path (output of a global planner) is just a sequence of poses with no timing.
Trajectory planning converts this into a **trajectory** — a function of time
$\mathbf{x}(t)$ with continuous position, velocity, and optionally acceleration profiles
that respect the robot's kinematic and dynamic constraints.

---

## Sub-modules

| Sub-module | Description |
|---|---|
| [`velocity_profiling/`](velocity_profiling/) | Trapezoidal and S-curve speed profiles — when to speed up, cruise, slow down |
| [`spline_fitting/`](spline_fitting/) | Smooth spatial curves through waypoints: cubic splines, Bézier, B-splines |
| [`teb/`](teb/) | Timed Elastic Band — jointly optimizes path geometry and timing |
| [`time_optimal/`](time_optimal/) | Minimum-time trajectories under kinematic and dynamic constraints |

---

## Typical workflow

```
Global path (waypoints, no timing)
        │
        ▼ spline_fitting/       → smooth geometric curve
        │
        ▼ velocity_profiling/   → add speed: fast on straights, slow on curves
        │
        ▼ (optional) teb/       → joint re-optimization of path + timing
        │
        ▼ time_optimal/         → find minimum time respecting constraints
        │
      Trajectory: x(t), v(t), a(t)
```

---

## What makes a good trajectory?

A trajectory must be:
- **Continuous in position** — no teleportation
- **Continuous in velocity** — no instantaneous speed change
- **Continuous in acceleration** — no jerk (for mechanical longevity)
- **Respects kinematic limits:** $|v| \le v_{\max}$, $|\omega| \le \omega_{\max}$
- **Respects dynamic limits:** $|\dot{v}| \le a_{\max}$, $|\dot{\omega}| \le \alpha_{\max}$
- **Collision-free** along its entire spatial extent

---

## See also

[`docs/theory.md`](docs/theory.md) for the full theoretical treatment.
