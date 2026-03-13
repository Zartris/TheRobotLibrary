# Module: native (frontend)

**Milestone:** M1 — Minimum Viable Robot (Sub-phase M1-L)  
**Status:** Not Started  
**Depends on:** simulation (running server to connect to)

---

### Phase 1 — Interface Design

- [ ] `include/native/ws_client.hpp` — WebSocket state receiver, JSON parsing
- [ ] `include/native/rest_client.hpp` — REST command sender
- [ ] `include/native/renderer.hpp` — 2D rendering API

### Phase 2 — Failing Tests (Red)

- [ ] `tests/test_ws_client.cpp` — parse valid state JSON → correct data structures
- [ ] `tests/test_rest_client.cpp` — serialize commands → correct JSON

### Phase 3 — Implementation (Green)

- [ ] `src/main.cpp` — SDL2+ImGui init, main loop
- [ ] `src/ws_client.cpp` — IXWebSocket connection, JSON → state structs
- [ ] `src/rest_client.cpp` — cpp-httplib command sender
- [ ] `src/renderer.cpp` — ImDrawList 2D rendering

### Phase 4 — Rendering Layers

- [ ] Grid map (free=white, occupied=black, unknown=gray)
- [ ] Robot (triangle showing heading)
- [ ] Lidar rays (light red)
- [ ] Global planned path (green line)
- [ ] DWA trajectory candidates (thin blue arcs)
- [ ] EKF covariance ellipse (yellow)

### Phase 4.5 — Observability

> **This phase gates module completion.** Both human developers and AI agents must be able
> to verify correct behavior through logs and metrics — not just frontend visuals.

- [ ] `ILogger` injected into module constructor via `common::getLogger("native_frontend")` (mockable in tests)
- [ ] All state transitions logged at `DEBUG` level (init, reset, mode changes, error paths)
- [ ] Hot-loop performance metrics logged at `TRACE` level (cycle time per iteration in µs, iteration count)
- [ ] At least one test asserts expected log lines appear via stdout capture
- [ ] Zero `ERROR`-level log entries during all nominal test runs

```bash
# Confirm logging output during tests:
cmake --build build --target native_frontend_tests
cd build && ctest -R native_frontend --output-on-failure 2>&1 | grep "\[DEBUG\]\|\[TRACE\]"
```

### Phase 5 — UI Panels

- [ ] Sim control buttons (start, stop, reset, step)
- [ ] Robot state text readout
- [ ] Scenario selector dropdown
- [ ] Module selector dropdowns (controller, planner, estimator)
- [ ] Visualization layer toggles

### Phase 6 — Manual Verification

- [ ] Run simulation_server + native_frontend together
- [ ] All rendering layers display correctly
- [ ] Controls work (start/stop/reset, module swap, scenario)

### Phase 7 — Docs Polish

- [ ] Update frontends/native/README.md with build + usage instructions
- [ ] Move this file to `repo-plans/modules/done/native_frontend.md`
