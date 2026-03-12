# Module: dwa

**Milestone:** M1 — Minimum Viable Robot (Sub-phase M1-H)  
**Status:** Not Started  
**Depends on:** common

---

### Phase 1 — Interface Design

- [ ] `include/dwa/dwa_planner.hpp` — `DWAConfig`, `DWAPlanner : ILocalPlanner`

### Phase 2 — Failing Tests (Red)

- [ ] `tests/CMakeLists.txt`
- [ ] `tests/test_dwa_planner.cpp`:
  - Open space with goal ahead → forward velocity
  - Obstacle directly ahead → turning velocity
  - Boxed in → zero twist
  - Respects dynamic window (acceleration limits)

### Phase 3 — Implementation (Green)

- [ ] `src/dwa_planner.cpp` — velocity sampling → trajectory rollout → cost scoring → best selection

### Phase 4 — Passing Tests

- [ ] All tests pass

### Phase 5 — Simulation Integration

- [ ] Default local planner in robot pipeline (M1-K)
- [ ] Selectable via `PUT /api/robot/local_planner {"type": "dwa"}`

### Phase 6 — Frontend Visualization

- [ ] Top-N trajectory candidates rendered as thin blue arcs (M1-L)

### Phase 7 — Docs Polish

- [ ] Update README.md
- [ ] Move this file to `repo-plans/modules/done/dwa.md`
