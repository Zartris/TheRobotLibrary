# M16 — Planning Upgrades II

**Status:** Not Started  
**Dependencies:** M7 (`IGlobalPlanner` interface, planning infrastructure), M4 (`OccupancyGrid` for collision checking). M2 (stable common types).  
**Scope:** Probabilistic Roadmap planner (multi-query, sampling-based) plus minimum-jerk / minimum-snap polynomial trajectory generation with Bézier curve variant.

---

## Goal

Extend the planning repertoire with two algorithms that cover distinct capability gaps:
PRM is the canonical multi-query sampling-based planner (the pair to RRT in every robotics
curriculum); polynomial trajectory generation provides minimum-jerk / minimum-snap smooth
paths used in drone and arm motion planning (the higher-order sibling to `spline_fitting`
from M7). Both are independently implementable.

---

## Modules

### motion_planning/global_planning/prm

Multi-query PRM. Roadmap is built once (offline or at startup) and reused across many
planning queries. Local planner: straight-line collision check against `OccupancyGrid`.
Query phase: Dijkstra / A* on the roadmap graph.

- [ ] `include/prm/prm_planner.hpp` — `PRMPlanner : IGlobalPlanner`
- [ ] `include/prm/prm_config.hpp` — `PRMConfig` (max_nodes, k_nearest, max_edge_length, num_samples)
- [ ] `src/prm_planner.cpp` — random sampling in C-space, k-nearest graph construction, Dijkstra query
- [ ] `tests/test_prm_planner.cpp`:
  - Obstacle-free: after N samples, roadmap is connected; query finds short path
  - Narrow corridor: path found with success rate ≥ 90% over 100 runs (probabilistic test)
  - Multi-query: roadmap built once, two distinct queries answered correctly; roadmap not rebuilt
  - No path: all samples blocked → query returns `std::nullopt`; no crash
  - `k_nearest` parameter: larger k → denser roadmap → shorter query paths (statistical test)
- [ ] Phase 4.5: `ILogger`, roadmap build stats (nodes, edges) at `DEBUG`, query solve time at `TRACE`
- [ ] Sim: selectable via `PUT /api/robot/global_planner {"type":"prm"}`
- [ ] Frontend: render roadmap graph (grey edges) + planned path (highlighted)

### motion_planning/trajectory_planning/polynomial

Minimum-jerk (1D: minimize ∫jerk²) and minimum-snap (minimize ∫snap²) polynomial segments.
Waypoint-constrained generation via QP on polynomial coefficients. Bézier curve variant using
de Casteljau algorithm with convex hull safety guarantee.

- [ ] `include/polynomial/polynomial_trajectory.hpp` — `PolynomialTrajectory`, `PolynomialSegment`, `BezierCurve`
- [ ] `include/polynomial/polynomial_config.hpp` — `PolynomialConfig` (order, derivative order to minimize, segment durations)
- [ ] `src/polynomial_trajectory.cpp` — coefficient QP (Eigen `LDLT`), `BezierCurve` via de Casteljau
- [ ] API: `generate(waypoints, durations)` → `PolynomialTrajectory`; `sample(t)` → position/velocity/acceleration
- [ ] `tests/test_polynomial_trajectory.cpp`:
  - 3-waypoint minimum-jerk: endpoint positions match; endpoint derivatives (vel, accel) match specified boundary conditions
  - Minimum-snap: 4th derivative minimized (verify via numerical integration of snap vs. spline baseline)
  - Bézier curve: all points lie strictly within convex hull of control points
  - `sample(t)` continuity: position/velocity continuous at segment junctions; acceleration C0 for min-jerk
  - Single segment: analytic polynomial coefficients match closed-form solution
- [ ] Phase 4.5: `ILogger`, QP solve time + condition number at `DEBUG`, sample computation time at `TRACE`
- [ ] Sim: trajectory planner slot — replaces `velocity_profiling` for drone/arm scenarios
- [ ] Frontend: render polynomial trajectory with curvature color coding

---

## Deliverables

- [ ] `motion_planning/global_planning/prm` module: interface, implementation, tests
- [ ] `motion_planning/trajectory_planning/polynomial` module: interface, implementation, tests
- [ ] PRM selectable via REST mid-run without crash
- [ ] Frontend renders PRM roadmap graph + planned path
- [ ] Frontend renders polynomial trajectory with curvature color coding
- [ ] All modules pass Phase 4.5 — Observability gate

---

## Exit Criteria

1. PRM finds path in cluttered environment within configured sample budget
2. PRM roadmap multi-query verified: second query reuses graph without rebuild
3. Polynomial trajectory through 3+ waypoints satisfies endpoint derivatives
4. Bézier curve satisfies convex hull property (verified by test)
5. All unit tests pass, CI green
6. All modules pass Phase 4.5 — Observability gate

---

## NOT IN

Informed PRM (→ future), 3D C-space PRM, minimum-time polynomial (→ M7 `time_optimal`),
NURBS (→ M7 `spline_fitting`), QP solver library (use Eigen direct solver for small systems
only), parallel roadmap construction.
