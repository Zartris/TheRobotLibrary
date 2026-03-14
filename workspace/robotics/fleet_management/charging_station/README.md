# charging_station

Charging station registry and assignment manager for the fleet layer. Tracks station availability, maintains a priority queue of robots awaiting charging (ordered by urgency flag and battery level), and estimates charge completion time via a linear charge model.

**Milestone:** M24 — Fleet Operations  
**Status:** Scaffold only — awaiting implementation

## Features

- `ChargingStationManager` — central authority for station assignment
- Station registry: `registerStation()`, `setFault()`, `clearFault()`
- Priority assignment: `requestCharging()` → assigns idle station or enqueues; urgency flag takes precedence over battery level
- Release-and-reassign: `releaseStation()` automatically assigns next queued robot
- Completion prediction: `t = capacity_kwh × (1 − soc) / power_kw` (linear model)
- Status summary: `getStatus()` → all stations + queue depth

## Dependencies

- `common` (logging, `Pose2D`)
