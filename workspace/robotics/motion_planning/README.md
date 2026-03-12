# motion_planning

Path planning and trajectory generation for mobile robots.

Motion planning spans four distinct sub-areas, each answering a different question.
Every leaf sub-module is a **standalone C++ library** — copy only what you need.

---

## Sub-areas

| Sub-area | Question answered | Sub-modules |
|---|---|---|
| [`global_planning/`](global_planning/) | *Is there a collision-free path from A to B?* | `astar/`, `dijkstra/`, `rrt/` |
| [`local_planning/`](local_planning/) | *What velocity command right now follows the path and avoids dynamic obstacles?* | `dwa/` |
| [`trajectory_planning/`](trajectory_planning/) | *How fast, with what profile, in what time does the robot execute the path?* | `velocity_profiling/`, `spline_fitting/`, `teb/`, `time_optimal/` |
| [`multi_robot/`](multi_robot/) | *How do N robots each reach their goal without colliding with each other?* | `orca/`, `priority_planning/`, `cbs/`, `dmpc/`, `mader/` |

---

## The Planning Stack

In a complete robot system, the four layers work together:

```
Goal
 │
 ▼
Global planner   ── "find a path through the static map"
 │         geometric path (list of poses)
 ▼
Trajectory planner ─ "add timing, velocity, acceleration profile to the path"
 │         time-parametrized trajectory
 ▼
Local planner    ── "follow the trajectory, react to dynamic obstacles"
 │         velocity command (v, ω) every 50–100 ms
 ▼
Controller       ── "execute the velocity command" (see workspace/robotics/control/)
```

Multi-robot planning runs alongside this stack, either replacing the global planner
(coordinated) or sitting on top of the local planner (reactive).

---

## Domain overview

See [`docs/theory.md`](docs/theory.md) for a conceptual introduction to the planning
hierarchy before diving into a specific sub-area.

---

## Dependency rule

All sub-modules depend only on `common`. No sub-module may depend on another planning
sub-module, on `simulation`, or on `frontends`. If a planner needs a map representation,
it accepts `occupancy_grid::OccupancyGrid` from `perception/occupancy_grid/` by value or
reference through the `common` interface; it does not link against `perception`.
