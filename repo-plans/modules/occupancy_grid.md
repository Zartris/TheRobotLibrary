# Module: occupancy_grid

**Milestone:** M1 — Minimum Viable Robot (Sub-phase M1-C)  
**Status:** Not Started  
**Depends on:** common, ray_casting (for cell marking during scan update)

---

### Phase 1 — Interface Design

- [ ] `include/occupancy_grid/occupancy_grid_map.hpp` — `OccupancyGridMap` class with log-odds update

### Phase 2 — Failing Tests (Red)

- [ ] `tests/CMakeLists.txt`
- [ ] `tests/test_occupancy_grid_map.cpp`:
  - Empty grid → all cells Unknown
  - Single ray update → cells along ray Free, endpoint Occupied
  - Multiple updates → log-odds accumulate correctly
  - Coordinate conversion round-trip: `toGrid(toWorld(c)) == c`

### Phase 3 — Implementation (Green)

- [ ] `src/occupancy_grid_map.cpp` — log-odds inverse sensor model, Bresenham cell marking

### Phase 4 — Passing Tests

- [ ] All tests pass

### Phase 5 — Simulation Integration

- [ ] Grid updated from filtered lidar scans each sim tick (M1-K)

### Phase 6 — Frontend Visualization

- [ ] Grid rendered as colored cells in native frontend (M1-L)

### Phase 7 — Docs Polish

- [ ] Update README.md
- [ ] Move this file to `repo-plans/modules/done/occupancy_grid.md`
