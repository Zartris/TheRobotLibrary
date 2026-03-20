# Module: potential_field

**Milestone:** M20 — Planning Upgrades III  
**Status:** Not Started  
**Depends on:** common, M7 (`ILocalPlanner` interface), M4 (`OccupancyGrid`)

---

### Phase 1 — Interface Design

- [ ] `include/potential_field/potential_field_planner.hpp` — `PotentialFieldPlanner : ILocalPlanner`
- [ ] `include/potential_field/potential_field_config.hpp` — `PotentialFieldConfig` (attractive gain `k_att`, repulsive gain `k_rep`, influence radius `d_0`, step size, stall threshold, escape steps)

### Phase 2 — Failing Tests (Red)

- [ ] `tests/CMakeLists.txt`
- [ ] `tests/test_potential_field_planner.cpp`:
  - Open space, no obstacles: robot reaches goal within N steps; final position within 0.1 m of goal
  - Single obstacle between start and goal: robot deflects around obstacle; does not collide
  - Designed local minimum (symmetric obstacle arrangement): escape triggered within M steps; robot eventually reaches goal
  - Goal coincides with repulsive field: returns `std::nullopt`; no division by zero
  - `d_0 = 0` (no repulsion): pure attractive field; straight-line motion

### Phase 3 — Implementation (Green)

- [ ] `src/potential_field_planner.cpp` — attractive force: parabolic `k_att · (goal − pos)` within `d_switch`, conic beyond; repulsive force: `k_rep · (1/d − 1/d_0) · (1/d²) · ∇d` for `d < d_0`; gradient descent step; obstacle distance via `OccupancyGrid` nearest-obstacle lookup; random perturbation escape

### Phase 4 — Passing Tests

- [ ] All tests pass

### Phase 4.5 — Observability

> **This phase gates module completion.** Both human developers and AI agents must be able
> to verify correct behavior through logs and metrics — not just visual inspection.

- [ ] `ILogger` injected into module constructor via `common::getLogger("potential_field")` (mockable in tests)
- [ ] All state transitions logged at `DEBUG` level (init, reset, mode changes, error paths, net force magnitude + stall detection)
- [ ] Hot-loop performance metrics logged at `TRACE` level (per-step potential gradient computation in µs)
- [ ] At least one test asserts expected log lines appear via stdout capture
- [ ] Zero `ERROR`-level log entries during all nominal test runs

```bash
# Confirm logging output during tests:
cmake --build build --target potential_field_tests
cd build && ctest -R potential_field --output-on-failure 2>&1 | grep "\[DEBUG\]\|\[TRACE\]"
```

### Phase 5 — Simulation Integration

- [ ] Selectable via ImGui module selector (local planner type: "potential_field")

### Phase 6 — Frontend Visualization

- [ ] Render potential field gradient vectors (arrow overlay, optional toggle)

### Phase 7 — Docs Polish

- [ ] Update README.md
- [ ] Move this file to `repo-plans/modules/done/potential_field.md`
