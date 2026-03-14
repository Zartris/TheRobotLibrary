# M24 — Fleet Operations

**Status:** Not Started  
**Dependencies:** M12 (`BatteryManagement`, `FleetMonitor`, `TaskAllocation`, VDA 5050 modules completed; fleet infrastructure available).  
**Scope:** Single-module milestone. `ChargingStationManager` adds station registry, priority charge queue, and completion prediction to the fleet management layer established in M12.

---

## Goal

M12 built out the fleet layer: task allocation, fleet monitoring, battery level tracking, and VDA 5050 messaging. One capability is missing: assigning robots to physical charging stations and managing queues when more robots need charging than stations are available. `ChargingStationManager` closes this gap. It is intentionally a focused solo milestone — the fleet domain has no natural P5–P10 partner, and forcing an unrelated pairing would reduce clarity.

---

## Modules

### fleet_management/charging_station

Charging station registry and assignment manager. Tracks which stations are available, maintains a priority queue of robots awaiting charging (ordered by battery urgency), and provides predictive charge completion estimates based on a simple linear charge model.

- [ ] `include/charging_station/charging_station_manager.hpp` — `ChargingStationManager`
- [ ] `include/charging_station/charging_station_types.hpp` — `ChargingStation` (id `StationId`, position `Pose2D`, `max_power_kw`, status enum `IDLE / OCCUPIED / FAULT`); `ChargeRequest` (robot_id, battery_level_pct, urgency_flag); `AssignmentResult` (station_id, estimated_arrival_s, estimated_completion_s)
- [ ] `src/charging_station_manager.cpp`:
  - `registerStation(ChargingStation)` → void
  - `setFault(StationId)` / `clearFault(StationId)` — station health management
  - `requestCharging(ChargeRequest)` → `std::expected<AssignmentResult, std::string>` — assigns idle station if available; else enqueues; priority = urgency_flag first, then ascending battery_level_pct
  - `releaseStation(StationId)` → assigns next in queue if any
  - `predictCompletion(StationId)` → `std::optional<double>` — `t = capacity_kwh × (1 − soc) / power_kw` (linear model)
  - `getStatus()` → summary of all stations + queue depth
- [ ] `tests/test_charging_station_manager.cpp`:
  - 2 stations, 3 robots requesting simultaneously: first 2 assigned immediately; 3rd queued
  - Priority ordering: robot with `urgency = true` jumps ahead of lower-battery robot without urgency flag
  - `releaseStation()`: queued robot assigned automatically; assignment result returned to caller
  - Fault station: `FAULT` station not assigned; robots redirected to healthy stations
  - `predictCompletion()`: linear model matches hand-calculated time for known capacity / power / soc
  - Empty registry: `requestCharging()` → error via `std::expected`
  - No stations available + queue limit exceeded → documented cap behaviour (return error)
- [ ] Phase 4.5: `ILogger`, assignment decisions + queue depth at `DEBUG`, completion prediction compute time at `TRACE`
- [ ] Sim: fleet REST endpoints `POST /api/fleet/charging/request` + `GET /api/fleet/charging/status`
- [ ] Frontend: charging station map overlay; queue depth indicator; charge completion countdown

---

## Deliverables

- [ ] `fleet_management/charging_station` module: interface, implementation, tests
- [ ] 3-robot / 2-station assignment + queuing verified
- [ ] Priority ordering: urgency flag pre-empts battery-level sort
- [ ] Release-and-reassign: queued robot automatically assigned when station freed
- [ ] Fault handling: FAULT stations excluded from assignment
- [ ] All modules pass Phase 4.5 — Observability gate

---

## Exit Criteria

1. 3-robot / 2-station scenario: correct assignment + queuing verified
2. Priority ordering: urgency flag pre-empts battery-level sort
3. Release-and-reassign: queued robot automatically assigned when station freed
4. Fault handling: FAULT stations excluded from assignment
5. Completion prediction matches hand-calculated linear model
6. All unit tests pass, CI green
7. All modules pass Phase 4.5 — Observability gate (state transitions at `DEBUG`, metrics at `TRACE`)

---

## NOT IN

Dynamic power allocation (variable charging rate), multi-robot convoy routing to stations, predictive charging scheduling, physical charger hardware interface, energy pricing model.
