# Fleet Monitor

`IFleetMonitor` aggregates per-robot `vda5050::State` updates into a unified `FleetState`
snapshot. The task allocator, battery manager, and simulation API all consume `FleetState`
rather than querying individual robots.

## Interface

```cpp
#include <fleet_monitor/i_fleet_monitor.hpp>

class IFleetMonitor {
public:
    virtual void update(RobotId id, const vda5050::State&) = 0;
    virtual FleetState getFleetState() const = 0;
    virtual ~IFleetMonitor() = default;
};
```

## FleetState

`FleetState` contains: a map of `RobotId → RobotSummary`, where `RobotSummary` includes
current pose, battery level, operating mode, assigned order ID, and error list.
