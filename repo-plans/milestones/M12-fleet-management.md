# M12 — Fleet Management

**Status:** Not Started  
**Dependencies:** M8 (N-robot simulation infrastructure)  
**Scope:** Fleet-level coordination: VDA 5050 protocol, task assignment, fleet monitoring, battery management. All modules hot-swappable via REST.

---

## Goal

Multiple robots operate as a **coordinated fleet** rather than independent agents. A fleet
management system assigns VDA 5050 tasks, monitors overall fleet health, and handles
battery-level driven recharging decisions. This milestone is the capstone of multi-robot
deployment realism.

---

## Prerequisite: Fleet Scaffold (M0)

The `workspace/robotics/fleet_management/` directory tree and all CMakeLists.txt stubs are
created during M0. The sub-modules are INTERFACE targets (no source code) until M12 begins.

---

## Modules

### vda5050

VDA 5050 v2.0 message types and nlohmann/json serialization. This is a pure data-model
module — no algorithms, just C++ structs matching the VDA 5050 specification.

- [ ] `include/vda5050/order.hpp` — `Order` (orderId, orderUpdateId, nodes, edges, zoneSetId)
- [ ] `include/vda5050/instant_action.hpp` — `InstantAction` (actionId, actionType, actionParameters)
- [ ] `include/vda5050/state.hpp` — `State` (orderId, pose, velocity, battery, operatingMode, errors, loads)
- [ ] `include/vda5050/connection.hpp` — `Connection` (connectionState: ONLINE/OFFLINE/CONNECTIONBROKEN)
- [ ] `include/vda5050/visualization.hpp` — `Visualization` (high-rate pose update)
- [ ] JSON serialization for all types via nlohmann/json `to_json` / `from_json`
- [ ] `tests/test_vda5050_serialization.cpp`:
  - Round-trip: serialize → JSON string → deserialize → equal struct
  - Order with 3 nodes + 2 edges: serializes to spec-compliant JSON
  - State with errors list: all error fields preserved
  - Unknown JSON fields are ignored (forward-compatible parsing)

### task_allocation

Assign incoming VDA 5050 Orders to robots based on fleet state.

- [ ] `include/task_allocation/i_task_allocator.hpp` — `ITaskAllocator`
- [ ] `include/task_allocation/greedy_allocator.hpp` — `GreedyAllocator : ITaskAllocator` (nearest available robot)
- [ ] `include/task_allocation/auction_allocator.hpp` — `AuctionAllocator : ITaskAllocator` (Contract Net Protocol)
- [ ] `src/greedy_allocator.cpp`, `src/auction_allocator.cpp`
- [ ] `tests/test_greedy_allocator.cpp`:
  - 3 robots, 1 task: task assigned to nearest robot
  - Robot in error state: excluded from allocation
  - All robots occupied: returns `AllocationError::NoAvailableRobot`
- [ ] `tests/test_auction_allocator.cpp`:
  - 3 robots with different costs: lowest-cost robot wins
  - Tie: deterministic tiebreaker (lowest ID)
  - All robots at max capacity: returns error

### fleet_monitor

Aggregate per-robot VDA 5050 State updates into a unified `FleetState` snapshot.

- [ ] `include/fleet_monitor/i_fleet_monitor.hpp` — `IFleetMonitor`
- [ ] `include/fleet_monitor/fleet_monitor.hpp` — `FleetMonitor : IFleetMonitor`
- [ ] `include/fleet_monitor/fleet_state.hpp` — `FleetState`, `RobotSummary`
- [ ] `src/fleet_monitor.cpp`
- [ ] `tests/test_fleet_monitor.cpp`:
  - Update 3 robots, `getFleetState()` contains all 3
  - Robot not updated for > timeout threshold: marked as LOST
  - State snapshot is consistent (no partial updates visible to consumers)
  - Thread-safe: concurrent updates + reads don't race (lock-based)

### battery_management

Monitor battery levels and route robots to chargers when needed.

- [ ] `include/battery_management/i_battery_manager.hpp` — `IBatteryManager`
- [ ] `include/battery_management/threshold_battery_manager.hpp` — `ThresholdBatteryManager : IBatteryManager`
- [ ] `include/battery_management/charger_station.hpp` — `ChargerStation` (id, pose, available)
- [ ] `src/threshold_battery_manager.cpp`
- [ ] `tests/test_threshold_battery_manager.cpp`:
  - SoC = 19% (<20% threshold): `needsCharging()` returns true
  - SoC = 21%: `needsCharging()` returns false
  - 3 charger stations, 1 occupied: `routeToCharger()` returns nearest free charger
  - No free chargers: returns `BatteryError::NoAvailableCharger`

---

## Sim & REST Integration

- [ ] Extend simulation WorldModel: add charger station locations to scenario JSON
- [ ] Extend scenario JSON: `"chargers": [{"id": "C1", "pose": {...}}]`
- [ ] New REST endpoints:

| Method | Endpoint | Description |
|--------|----------|-------------|
| POST | `/api/fleet/task` | Submit VDA 5050 Order (JSON body) |
| GET | `/api/fleet/state` | Return full `FleetState` as JSON |
| POST | `/api/fleet/instant_action` | Submit `InstantAction` to a specific robot |
| GET | `/api/fleet/allocator` | Get current task allocator type |
| PUT | `/api/fleet/allocator` | Switch allocator `{"type": "greedy"\|"auction"}` |

- [ ] WebSocket state extended: add `"fleet": { "robots": [...], "chargers": [...] }` field to existing state message
- [ ] Backend tick: after each sim step, call `FleetMonitor::update()` with each robot's VDA 5050 state; call `BatteryManager::needsCharging()` for each robot

---

## Frontend Integration

- [ ] Native frontend: fleet panel showing per-robot state table (ID, pose, SoC, mode, current order)
- [ ] Native frontend: charger station icons on grid map
- [ ] Native frontend: highlight robot being routed to charger
- [ ] Mini-demo: 3 robots assigned tasks via REST; one runs low on battery and routes to charger automatically

---

## Deliverables

- [ ] `vda5050` module: all 5 message types, JSON serialization, tests
- [ ] `task_allocation` module: 2 allocator implementations, tests
- [ ] `fleet_monitor` module: thread-safe monitor, tests
- [ ] `battery_management` module: threshold policy, tests
- [ ] Fleet REST API (`/api/fleet/*`)
- [ ] WebSocket fleet state extension
- [ ] Frontend fleet panel
- [ ] At least 1 fleet scenario JSON with 3+ robots + 2+ charger stations

## Exit Criteria

1. 3 robots assigned tasks via `POST /api/fleet/task` and complete them
2. Low-SoC robot automatically routed to charger
3. Task allocator hot-swap (greedy ↔ auction) via REST without crash
4. VDA 5050 JSON round-trip test passes for all 5 message types
5. All unit tests pass, CI green
6. All modules pass Phase 4.5 — Observability gate (state transitions logged, metrics at TRACE)

## NOT IN

MQTT transport layer, multi-fleet management, real AGV/AMR hardware integration, advanced
predictive battery policies (deadline-aware), vehicle type heterogeneity constraints.
