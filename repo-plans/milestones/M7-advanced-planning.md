# M7 — Advanced Planning

**Status:** Not Started  
**Dependencies:** M4 (inflation for costmaps). _M3 recommended but not required — MPC experience helps with TEB but they use different solvers._  
**Scope:** Sampling-based planners, trajectory smoothing, optimization-based planning.

---

## Goal

Complete the planning toolkit: from grid-based (A*) to continuous-space (RRT*), from jagged waypoints to smooth splines, from heuristic profiles to time-optimal trajectories. All swappable in the sim.

---

## Modules

### dijkstra

Multi-source Dijkstra for distance/costmap generation. Also usable as a standalone planner.

- [ ] `include/dijkstra/dijkstra_planner.hpp` — `DijkstraPlanner : IGlobalPlanner`
- [ ] `src/dijkstra_planner.cpp`
- [ ] Multi-source distance map: compute distance from all goal cells → costmap
- [ ] `tests/test_dijkstra_planner.cpp`:
  - Same tests as A* (path correctness)
  - Distance map: values decrease toward goal, obstacles have infinity
  - Multi-source: distance from any of N goal cells

### rrt

RRT and RRT* in continuous 2D C-space.

- [ ] `include/rrt/rrt_planner.hpp` — `RRTPlanner : IGlobalPlanner`
- [ ] `src/rrt_planner.cpp`
- [ ] Collision checking against OccupancyGrid
- [ ] RRT*: rewiring for asymptotic optimality
- [ ] Path shortcutting: remove unnecessary waypoints
- [ ] `tests/test_rrt_planner.cpp`:
  - Open space → finds direct-ish path
  - Narrow passage → finds path (probabilistic — run N times, success rate ≥ 95%)
  - No path → returns nullopt after max iterations
  - RRT* path shorter than RRT path (statistically)

### spline_fitting

Smooth path interpolation: cubic spline, Catmull-Rom, B-spline, NURBS.

- [ ] `include/spline_fitting/spline_fitter.hpp` — `SplineFitter`
- [ ] `src/spline_fitter.cpp`
- [ ] `tests/test_spline_fitter.cpp`:
  - Cubic spline through 5 waypoints → C2 continuous
  - Catmull-Rom → passes through all control points
  - B-spline → smooth but doesn't necessarily interpolate control points
  - NURBS → non-uniform parameterization works correctly
  - Curvature at each point computed correctly

### teb

Timed Elastic Band: joint path + timing optimization.

- [ ] `include/teb/teb_planner.hpp` — `TEBPlanner`
- [ ] `src/teb_planner.cpp`
- [ ] g2o integration for sparse nonlinear optimization
- [ ] Constraints: obstacles, kinematics (max vel/accel), temporal spacing
- [ ] `tests/test_teb_planner.cpp`:
  - Optimized path shorter than input path
  - Respects obstacle clearance
  - Respects kinematic constraints
  - Temporal spacing is feasible

### time_optimal

Time-optimal trajectory generation (TOPP-RA / minimum-snap).

- [ ] `include/time_optimal/time_optimal_planner.hpp` — `TimeOptimalPlanner : IVelocityProfiler`
- [ ] `src/time_optimal_planner.cpp`
- [ ] `tests/test_time_optimal_planner.cpp`:
  - Straight path → bang-bang velocity profile
  - Curved path → velocity limited by curvature constraints
  - Result faster than trapezoidal profile on same path
  - Respects velocity + acceleration limits everywhere

---

## Sim & Frontend Integration

- [ ] Planner hot-swap: `PUT /api/robot/global_planner {"type": "dijkstra"|"rrt"}`
- [ ] Trajectory planner selection in pipeline
- [ ] Frontend: visualize planned paths from each planner, curvature profile, velocity profile
- [ ] Mini-demo: compare A* (jagged) → RRT* (smooth) → spline post-processed → TEB-optimized

---

## Deliverables

- [ ] 5 modules: dijkstra, rrt, spline_fitting, teb, time_optimal
- [ ] All with unit tests
- [ ] Planner swap in sim
- [ ] Frontend comparison visualization
- [ ] Demo: planner comparison on same scenario

## Exit Criteria

1. All planners navigate standard scenarios
2. Swap via REST without crash
3. TEB produces smoother/faster paths than A* + trapezoidal
4. All unit tests pass
5. All modules pass Phase 4.5 — Observability gate (state transitions logged at DEBUG, hot-loop metrics at TRACE)

## NOT IN

Multi-robot planning, 3D planning.
