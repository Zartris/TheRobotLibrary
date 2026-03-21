# M20 — Planning Upgrades III

**Status:** Not Started  
**Dependencies:** M7 (`IGlobalPlanner` interface, `RRTStar` implementation, planning infrastructure), M4 (`OccupancyGrid` for collision checking). M16 recommended for context but not required.  
**Scope:** Two complementary planning additions: a classical reactive local planner (potential fields) and an asymptotically optimal global planner (Informed RRT*). Both extend M7's planning infrastructure; both are independently implementable within M20.

---

## Goal

M7 established the global planning chapter with RRT* and local planning with DWA/TEB. M20 adds the two missing entries: potential fields as the simplest reactive local planner (every robotics curriculum's starting point; teaches attractive/repulsive gradient field design and local minimum phenomena); Informed RRT* as the sample-efficiency upgrade to RRT* (ellipsoidal heuristic region drastically narrows the search space after a first feasible path is found).

---

## Modules

### motion_planning/local_planning/potential_field

Artificial potential field planner. Combines an attractive parabolic/conic field toward the goal with repulsive inverse-square fields from obstacles. Local minima escape is triggered when the robot's progress stalls, applying a random perturbation.

- [ ] `include/potential_field/potential_field_planner.hpp` — `PotentialFieldPlanner : ILocalPlanner`
- [ ] `include/potential_field/potential_field_config.hpp` — `PotentialFieldConfig` (attractive gain `k_att`, repulsive gain `k_rep`, influence radius `d_0`, step size, stall threshold, escape steps)
- [ ] `src/potential_field_planner.cpp` — attractive force: parabolic `k_att · (goal − pos)` within `d_switch`, conic `k_att · d_switch · (goal − pos)/|goal − pos|` beyond; repulsive force: `k_rep · (1/d − 1/d_0) · (1/d²) · ∇d` for `d < d_0`; gradient descent step
- [ ] Obstacle distance queries via `OccupancyGrid` nearest-obstacle lookup
- [ ] `tests/test_potential_field_planner.cpp`:
  - Open space, no obstacles: robot reaches goal within N steps; final position within 0.1 m of goal
  - Single obstacle between start and goal: robot deflects around obstacle; does not collide
  - Designed local minimum (symmetric obstacle arrangement): escape triggered within M steps; robot eventually reaches goal
  - Goal coincides with repulsive field: returns `std::nullopt`; no division by zero
  - `d_0 = 0` (no repulsion): pure attractive field; straight-line motion
- [ ] Phase 4.5: `ILogger`, net force magnitude + stall detection at `DEBUG`, per-step potential gradient at `TRACE`
- [ ] Sim: selectable via ImGui local planner dropdown
- [ ] ImGui panel: render potential field gradient vectors (arrow overlay, optional toggle)

### motion_planning/global_planning/informed_rrt_star

Informed RRT*. Runs standard RRT* until the first feasible path is found, then switches to sampling exclusively within the prolate hyperspheroid (PHS) defined by the current best path cost `c_best` and the start-goal Euclidean distance `c_min`.

- [ ] `include/informed_rrt_star/informed_rrt_star_planner.hpp` — `InformedRRTStar : IGlobalPlanner`
- [ ] `include/informed_rrt_star/informed_rrt_star_config.hpp` — `InformedRRTStarConfig` (max_iterations, step_size, rewire_radius, initial_rrt_budget)
- [ ] `src/informed_rrt_star_planner.cpp` — standard RRT* phase until first feasible path; PHS transformation: `x_sample = C · diag(r1,...,rn) · x_ball + x_centre`, where `r1 = c_best/2`, `r2 = ... = rn = sqrt(c_best² − c_min²)/2`; switch to informed sampling once `c_best < ∞`
- [ ] `tests/test_informed_rrt_star.cpp`:
  - Obstacle-free: finds path; solution length ≤ 1.05 × `c_min` after sufficient iterations
  - Convergence speed: Informed RRT* reaches `c_best ≤ 1.1 × c_min` in ≤ 50% of iterations vanilla RRT* requires (over 10 seeds)
  - PHS bound check: all samples after first solution lie within ellipsoid (`x^T * M^-1 * x ≤ 1`)
  - C-space obstacle: path correctly avoids obstacle; cost strictly non-increasing across iterations
  - No feasible path: returns `std::nullopt`; no crash
- [ ] Phase 4.5: `ILogger`, `c_best` improvement + PHS volume reduction at `DEBUG`, PHS sampling time at `TRACE`
- [ ] Sim: selectable via ImGui global planner dropdown
- [ ] ImGui panel: render PHS ellipse overlay; show `c_best` convergence curve in side panel

---

## Deliverables

- [ ] `motion_planning/local_planning/potential_field` module: interface, implementation, tests
- [ ] `motion_planning/global_planning/informed_rrt_star` module: interface, implementation, tests
- [ ] Potential field local minimum escape verified in tests
- [ ] Informed RRT* convergence speed verified against vanilla RRT* (10 seeds)
- [ ] All post-first-solution samples lie within the prolate hyperspheroid
- [ ] All modules pass Phase 4.5 — Observability gate

---

## Exit Criteria

1. Potential field planner reaches goal in open space within N steps
2. Potential field local minimum escape triggers within M stall steps and succeeds
3. Informed RRT* finds feasible path in cluttered map
4. Informed RRT* converges to near-optimal cost faster than vanilla RRT* (measured across 10 seeds)
5. All post-first-solution samples lie within the prolate hyperspheroid (geometric check)
6. All unit tests pass, CI green
7. All modules pass Phase 4.5 — Observability gate (state transitions at `DEBUG`, metrics at `TRACE`)

---

## NOT IN

Potential field with navigation function (guaranteed global convergence), 3D APF, Informed PRM, bidirectional Informed RRT*, motion-primitive informed sampling (→ M23 `lattice_planner`).
