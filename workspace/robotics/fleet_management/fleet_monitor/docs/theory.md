# Fleet Monitor — Theory

## Purpose

The fleet monitor aggregates per-robot VDA 5050 `State` messages into a single `FleetState` snapshot accessible to the FMS, task allocator, battery manager, and REST API. It is the single source of truth about the fleet at any given time.

## Data Model

```
FleetState
├── timestamp                    (seconds since epoch)
├── robots: Map<RobotId, RobotInfo>
│   ├── state: vda5050::State    (latest state message)
│   ├── lastSeen: timestamp
│   └── online: bool             (true if within heartbeat timeout)
└── chargers: Map<ChargerId, ChargerInfo>
    ├── occupied: bool
    └── robotId: optional<RobotId>
```

## `IFleetMonitor` Interface

```cpp
class IFleetMonitor {
public:
    virtual ~IFleetMonitor() = default;
    virtual void update(RobotId id, const vda5050::State& state) = 0;
    virtual void setOffline(RobotId id) = 0;
    virtual FleetState getFleetState() const = 0;
    virtual std::optional<vda5050::State> getRobotState(RobotId id) const = 0;
};
```

## Heartbeat Handling

A robot is considered offline if no `State` message is received for longer than:
$$t_{\text{timeout}} = 3 \times t_{\text{heartbeat\_period}}$$

The default `heartbeat_period` is 1 second (configurable). Offline robots are excluded from task allocation bids.

## Thread Safety

`IFleetMonitor` implementations must be thread-safe. The REST API thread reads `getFleetState()` concurrently with the simulation update thread calling `update()`. Use a reader-writer lock (`std::shared_mutex`) to avoid blocking reads for the common case.

## References

- VDA 5050 `State` message spec — see `vda5050/docs/theory.md`
- `task_allocation/docs/theory.md` — consumes FleetState
- `battery_management/docs/theory.md` — consumes FleetState
