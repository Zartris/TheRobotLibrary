# Module: ray_casting

**Milestone:** M1 — Minimum Viable Robot (Sub-phase M1-B)  
**Status:** Not Started  
**Depends on:** common

---

### Phase 1 — Interface Design

- [ ] `include/ray_casting/ray_caster.hpp` — `castRay()`, `castScan()`, `RayCastResult`, `ScanConfig`

### Phase 2 — Failing Tests (Red)

- [ ] `tests/CMakeLists.txt` — Catch2 target
- [ ] `tests/test_ray_caster.cpp`:
  - Ray into empty grid → max_range, no hit
  - Ray toward wall at known distance → correct range ± 1 cell
  - Diagonal ray at 45° → expected hit
  - `castScan` with 3 rays → correct LaserScan structure

### Phase 3 — Implementation (Green)

- [ ] `src/ray_caster.cpp` — Bresenham ray traversal on OccupancyGrid

### Phase 4 — Passing Tests

- [ ] All tests pass

### Phase 5 — Simulation Integration

- [ ] Used by simulation to generate simulated lidar data (M1-J)

### Phase 6 — Frontend Visualization

- [ ] Lidar rays rendered in native frontend (M1-L)

### Phase 7 — Docs Polish

- [ ] Update README.md with usage examples
- [ ] Move this file to `repo-plans/modules/done/ray_casting.md`
