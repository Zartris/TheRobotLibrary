Description and intent of the file:
This file is listing down tasks and features that we want before making it into a concrete milestone. There is two  list. The quick todo list is for us to quickly jot down ideas and tasks, while the fleshed out todo list is for us to have a more detailed view of what we want to do and how we want to do it.

---
Quick todo list:
---
Fleshed out todo list:

---

### [P2] motion_planning/multi_robot/rmader — Robust MADER: Decentralized Multi-Agent Trajectory Planning Robust to Communication Delay
- **Domain:** `motion_planning/multi_robot/rmader`
- **Reference:** [RMADER (MIT ACL)](https://github.com/mit-acl/rmader) — Kondo et al., IEEE RA-L + ICRA 2023, arXiv:2303.06222
- **Why:** MADER is already implemented (`multi_robot/mader/`) but assumes reliable, low-latency comms. RMADER is the direct successor that adds a **delay compensation step**: agents hold their candidate trajectory for a delay buffer period before committing, ensuring no neighbor's delayed broadcast can arrive after commitment. Validated at 300ms network delay with 10 agents + dynamic obstacles on hardware. This is the production-ready version of MADER for real-world deployments.
- **Key difference from MADER:** Adds a `DelayBuffer` holding window before the check-commit step; the rest of the pipeline (convex decomposition, MINVO intersection check, Bézier polynomial trajectory) is shared.
- **Scope:**
  - [ ] Scaffold `workspace/robotics/motion_planning/multi_robot/rmader/`
  - [ ] `RmaderAgent : MaderAgent` (or composition) — inherits/wraps MADER pipeline, adds delay buffer and hold-before-commit logic
  - [ ] `RmaderConfig` — extends `MaderConfig` with `delay_budget_ms`, `hold_time_s`
  - [ ] `DelayBuffer` — time-stamped trajectory queue; flushes stale entries; triggers recheck on late arrivals
  - [ ] Shared types with `mader/` via `common/` (no cross-module linking — types passed by value)
  - [ ] Tests: single agent under simulated delay → commits only after hold window; two agents with artificial 200ms delay → no collision in committed segments
  - [ ] `docs/theory.md`: delay compensation proof sketch, comparison with MADER safety guarantee, when RMADER vs MADER is appropriate
  - [ ] Wire into `multi_robot/CMakeLists.txt`; add to M8 or M12 milestone
- **Note:** Reference implementation uses Gurobi; our version should use OSQP (already in `deps.cmake`) — same trade-off as MADER.

### [P3] motion_planning/trajectory_planning/hermite_spline — Hermite Spline-based Efficient Trajectory Planning (MIGHTY)
- **Domain:** `motion_planning/trajectory_planning/hermite_spline`
- **Reference:** [MIGHTY (MIT ACL)](https://github.com/mit-acl/mighty) — Kondo et al., arXiv:2511.10822, submitted to IEEE RA-L
- **Why:** MIGHTY introduces Hermite spline parameterization as a computationally efficient alternative to minimum-snap polynomial trajectories. Hermite splines give direct control over endpoint positions and velocities, enabling fast replanning in cluttered and dynamic environments. It is distinct from our existing `polynomial` (min-snap) and `spline_fitting` modules. Proven on multi-agent UAV hardware in dense forests with dynamic obstacles.
- **Scope:**
  - [ ] Scaffold `workspace/robotics/motion_planning/trajectory_planning/hermite_spline/`
  - [ ] `HermiteSplineTrajectory` — cubic Hermite spline segments parameterized by endpoint positions + velocities; supports N-segment chains
  - [ ] `HermiteTrajectoryPlanner` — generates collision-free trajectories by optimizing knot velocities; convex-hull safety constraint via `DecompUtil`-style corridor (Eigen-only approximation)
  - [ ] Dynamic obstacle avoidance: re-plan over a receding horizon when obstacle proximity triggers replanning
  - [ ] Multi-agent support: inter-agent collision constraints as soft penalty terms
  - [ ] `ITrajectoryPlanner` conformance (same interface as `polynomial`, `teb`)
  - [ ] Tests: single-segment endpoint boundary conditions; multi-segment continuity; replanning triggered by obstacle; convergence to goal within horizon
  - [ ] `docs/theory.md`: Hermite basis functions, knot velocity optimization, comparison with min-snap, receding-horizon replanning loop
  - [ ] Add to appropriate milestone (M16 Planning Upgrades II or new M16.5)

