# Module: astar

**Milestone:** M1 — Minimum Viable Robot (Sub-phase M1-G)  
**Status:** Not Started  
**Depends on:** common

---

### Phase 1 — Interface Design

- [ ] `include/astar/astar_planner.hpp` — `AStarPlanner : IGlobalPlanner`

### Phase 2 — Failing Tests (Red)

- [ ] `tests/CMakeLists.txt`
- [ ] `tests/test_astar_planner.cpp`:
  - Straight line in empty grid → direct path
  - L-shaped obstacle → path goes around
  - No path possible (fully enclosed) → `std::nullopt`
  - Start == goal → single-point path
  - Start/goal in occupied cell → `std::nullopt`

### Phase 3 — Implementation (Green)

- [ ] `src/astar_planner.cpp` — priority queue, octile heuristic, 8-connected neighbors

### Phase 4 — Passing Tests

- [ ] All tests pass

### Phase 5 — Simulation Integration

- [ ] Default global planner in robot pipeline (M1-K)
- [ ] Selectable via `PUT /api/robot/global_planner {"type": "astar"}`

### Phase 6 — Frontend Visualization

- [ ] Planned global path rendered as green line (M1-L)

### Phase 7 — Docs Polish

- [ ] Update README.md
- [ ] Move this file to `repo-plans/modules/done/astar.md`
