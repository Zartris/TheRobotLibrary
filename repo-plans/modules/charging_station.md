# Module: charging_station

**Milestone:** M24 — Fleet Operations  
**Status:** Not Started  
**Depends on:** common, M12 (fleet infrastructure: `BatteryManagement`, `FleetMonitor`)

---

### Phase 1 — Interface Design

- [ ] `include/charging_station/charging_station_manager.hpp` — `ChargingStationManager`
- [ ] `include/charging_station/charging_station_types.hpp` — `ChargingStation` (id `StationId`, position `Pose2D`, `max_power_kw`, status enum `IDLE / OCCUPIED / FAULT`); `ChargeRequest` (robot_id, battery_level_pct, urgency_flag); `AssignmentResult` (station_id, estimated_arrival_s, estimated_completion_s)

### Phase 2 — Failing Tests (Red)

- [ ] `tests/CMakeLists.txt`
- [ ] `tests/test_charging_station_manager.cpp`:
  - 2 stations, 3 robots requesting simultaneously: first 2 assigned immediately; 3rd queued
  - Priority ordering: robot with `urgency = true` jumps ahead of lower-battery robot without urgency flag
  - `releaseStation()`: queued robot assigned automatically; assignment result returned to caller
  - Fault station: `FAULT` station not assigned; robots redirected to healthy stations
  - `predictCompletion()`: linear model matches hand-calculated time for known capacity / power / soc
  - Empty registry: `requestCharging()` → error via `std::expected`
  - No stations available + queue limit exceeded → documented cap behaviour (return error)

### Phase 3 — Implementation (Green)

- [ ] `src/charging_station_manager.cpp`:
  - `registerStation(ChargingStation)` → void
  - `setFault(StationId)` / `clearFault(StationId)`
  - `requestCharging(ChargeRequest)` → `std::expected<AssignmentResult, std::string>`; priority queue ordering
  - `releaseStation(StationId)` → assigns next in queue
  - `predictCompletion(StationId)` → `t = capacity_kwh × (1 − soc) / power_kw` (linear model)
  - `getStatus()` → summary of all stations + queue depth

### Phase 4 — Passing Tests

- [ ] All tests pass

### Phase 4.5 — Observability

> **This phase gates module completion.** Both human developers and AI agents must be able
> to verify correct behavior through logs and metrics — not just frontend visuals.

- [ ] `ILogger` injected into module constructor via `common::getLogger("charging_station")` (mockable in tests)
- [ ] All state transitions logged at `DEBUG` level (init, reset, mode changes, error paths, assignment decisions + queue depth)
- [ ] Hot-loop performance metrics logged at `TRACE` level (completion prediction computation time in µs)
- [ ] At least one test asserts expected log lines appear via stdout capture
- [ ] Zero `ERROR`-level log entries during all nominal test runs

```bash
# Confirm logging output during tests:
cmake --build build --target charging_station_tests
cd build && ctest -R charging_station --output-on-failure 2>&1 | grep "\[DEBUG\]\|\[TRACE\]"
```

### Phase 5 — Simulation Integration

- [ ] Fleet REST endpoints: `POST /api/fleet/charging/request` + `GET /api/fleet/charging/status`

### Phase 6 — Frontend Visualization

- [ ] Charging station map overlay; queue depth indicator; charge completion countdown

### Phase 7 — Docs Polish

- [ ] Update README.md
- [ ] Move this file to `repo-plans/modules/done/charging_station.md`
