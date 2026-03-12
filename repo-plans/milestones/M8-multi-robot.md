# M8 — Multi-Robot

**Status:** Not Started  
**Dependencies:** M3 (MPC for DMPC), M7 (planners for per-robot planning)

> **Scope note:** This milestone is large. Consider splitting into M8-A (multi-robot infra + ORCA) and M8-B (priority + CBS + DMPC + MADER) if pacing requires it.  
**Scope:** Sim supports N robots. Five multi-robot algorithms from reactive to optimal.

---

## Goal

Multiple robots in a shared environment. Progressive algorithm complexity: reactive velocity obstacles → centralized sequential → centralized optimal → distributed optimization → decentralized asynchronous. User picks the algorithm and watches the behavioral differences.

---

## Prerequisite: Multi-Robot Sim Infrastructure

Before any algorithm work, the simulation needs N-robot support:

- [ ] World model supports N robots (each with own pose, sensors, pipeline)
- [ ] Per-robot sensor streams (each robot has independent lidar)
- [ ] Per-robot pipeline (each robot can have different controller/planner/estimator)
- [ ] WebSocket state extended: array of robot states
- [ ] REST: `/api/robots` (list), `/api/robots/{id}/pipeline` (per-robot config)
- [ ] Frontend: multi-robot rendering (colored triangles, per-robot paths, per-robot lidar)
- [ ] Scenario JSON supports N robots with independent start/goal positions

---

## Modules

### orca

Optimal Reciprocal Collision Avoidance. Reactive, pairwise, no communication.

- [ ] `include/orca/orca.hpp` — builds on velocity obstacle concepts (VO → RVO → ORCA)
- [ ] `src/orca.cpp`
- [ ] `tests/test_orca.cpp`:
  - 2 robots heading toward each other → both avoid, no collision
  - 4 robots at corners heading to opposite corners → all reach goals
  - Robot + static obstacle → deflects velocity
  - High density (10 robots in small space) → deadlock-free (or graceful degradation)

### priority_planning

Sequential priority-based planning. Higher-priority robots plan first; lower-priority robots treat earlier trajectories as moving obstacles.

- [ ] `include/priority_planning/priority_planner.hpp`
- [ ] `src/priority_planner.cpp`
- [ ] `tests/test_priority_planner.cpp`:
  - 2 robots, crossing paths → higher priority goes straight, lower deviates
  - Priority ordering affects solution quality
  - All robots reach goals

### cbs

Conflict-Based Search. Provably optimal multi-agent pathfinding (MAPF).

- [ ] `include/cbs/cbs_planner.hpp`
- [ ] `src/cbs_planner.cpp`
- [ ] Low-level: A* per agent with constraints
- [ ] High-level: conflict tree search
- [ ] `tests/test_cbs_planner.cpp`:
  - 2 robots on 5×5 grid → optimal MAPF solution
  - Solution respects all space-time constraints (no two robots in same cell at same time)
  - Known benchmark instances match expected optimal cost
  - Scales to ≤ 8 robots on ≤ 20×20 grid

### dmpc

Distributed MPC. Per-robot MPC with inter-robot trajectory constraints. Iterative negotiation.

- [ ] `include/dmpc/dmpc_controller.hpp`
- [ ] `src/dmpc_controller.cpp`
- [ ] OSQP for per-robot QP solve
- [ ] `tests/test_dmpc_controller.cpp`:
  - 2 robots approaching each other → DMPC finds collision-free trajectories
  - Respects per-robot kinematic constraints
  - Converges within iteration limit
  - Solution smoother than ORCA for same scenario

### mader

MADER: decentralized asynchronous replanning. Shared committed trajectory segments.

- [ ] `include/mader/mader_planner.hpp`
- [ ] `src/mader_planner.cpp`
- [ ] `tests/test_mader_planner.cpp`:
  - Asynchronous planning: robots plan at different rates → still collision-free
  - Committed trajectory segments respected by all robots
  - Recovery from unexpected delays

---

## Deliverables

- [ ] Multi-robot sim infrastructure
- [ ] 5 modules: orca, priority_planning, cbs, dmpc, mader — each with unit tests
- [ ] Algorithm selector in frontend (dropdown for multi-robot algorithm)
- [ ] Frontend: multi-robot visualization (colored robots, per-robot paths, collision zones)
- [ ] Mini-demo: 3–5 robots, toggle algorithms to compare behavior

## Exit Criteria

1. 5 robots navigate to goals without collision using each algorithm
2. All unit tests pass
3. Algorithm switching visible in frontend
4. CBS finds provably optimal solution on small instances

## NOT IN

Heterogeneous robots, 3D, communication failure modeling, more than ~10 robots.
