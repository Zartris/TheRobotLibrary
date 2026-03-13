# Battery Management

`IBatteryManager` monitors per-robot state-of-charge and decides when and where a robot
should charge.

## Interface

```cpp
#include <battery_management/i_battery_manager.hpp>

class IBatteryManager {
public:
    virtual bool needsCharging(RobotId, const vda5050::State&) const = 0;
    virtual std::expected<Pose2D, BatteryError>
    routeToCharger(RobotId, const FleetState&) const = 0;
    virtual ~IBatteryManager() = default;
};
```

## Charging Policy

The default `ThresholdBatteryManager` triggers recharging when SoC drops below a
configurable threshold (default: 20%). The nearest available charger station from the
`FleetState` map is selected. Future implementations may use predictive policies based
on planned mission distance vs. remaining charge.
