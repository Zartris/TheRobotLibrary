# Module: lattice_planner

**Milestone:** M23 — Lattice Planning & Semantic Vision  
**Status:** Not Started  
**Depends on:** common, M7 (`IGlobalPlanner` interface, planning infrastructure), M4 (`OccupancyGrid`)

---

### Phase 1 — Interface Design

- [ ] `include/lattice_planner/lattice_planner.hpp` — `LatticePlanner : IGlobalPlanner`
- [ ] `include/lattice_planner/lattice_planner_config.hpp` — `LatticePlannerConfig` (lattice_resolution, num_headings, num_primitives_per_heading, max_primitive_length, use_astar flag)
- [ ] `include/lattice_planner/lattice_types.hpp` — `LatticeNode` (grid_x, grid_y, heading_idx), `MotionPrimitive` (sequence of `Pose2D` waypoints, cost), `LatticeGraph` (adjacency list)

### Phase 2 — Failing Tests (Red)

- [ ] `tests/CMakeLists.txt`
- [ ] `tests/test_lattice_planner.cpp`:
  - Straight corridor: planner finds straight path using zero-curvature primitives
  - 90° turn required: planner selects turning primitive; output path is kinematically feasible (heading continuity)
  - Blocked corridor: planner routes around via alternate primitives
  - No path exists: returns `std::nullopt`; no crash
  - Lattice resolution: coarser grid → faster planning; path within 20% of fine-grid result
  - Heading continuity: consecutive primitive start/end headings align within heading discretization tolerance

### Phase 3 — Implementation (Green)

- [ ] `src/lattice_planner.cpp` — primitive generation: arcs (straight + left/right curves) connecting to neighbouring lattice nodes; collision check via `OccupancyGrid`; A* / Dijkstra graph search on `LatticeGraph`

### Phase 4 — Passing Tests

- [ ] All tests pass

### Phase 4.5 — Observability

> **This phase gates module completion.** Both human developers and AI agents must be able
> to verify correct behavior through logs and metrics — not just visual inspection.

- [ ] `ILogger` injected into module constructor via `common::getLogger("lattice_planner")` (mockable in tests)
- [ ] All state transitions logged at `DEBUG` level (init, reset, mode changes, error paths, graph node/edge counts + A* expansion count)
- [ ] Hot-loop performance metrics logged at `TRACE` level (primitive generation time in µs)
- [ ] At least one test asserts expected log lines appear via stdout capture
- [ ] Zero `ERROR`-level log entries during all nominal test runs

```bash
# Confirm logging output during tests:
cmake --build build --target lattice_planner_tests
cd build && ctest -R lattice_planner --output-on-failure 2>&1 | grep "\[DEBUG\]\|\[TRACE\]"
```

### Phase 5 — Simulation Integration

- [ ] Selectable via ImGui module selector (global planner type: "lattice")

### Phase 6 — Visualization

- [ ] Render lattice graph (grey edges) + planned path (highlighted) + heading arrows at each node

### Phase 7 — Docs Polish

- [ ] Update README.md
- [ ] Move this file to `repo-plans/modules/done/lattice_planner.md`
