# Battery Management — Theory

## Purpose

The battery manager monitors per-robot battery state (from VDA 5050 `State` messages) and decides when a robot should route to a charger instead of accepting new tasks.

## Decision Model

A robot is flagged for recharging when:

$$\text{batteryCharge} < \theta_{\text{low}}$$

where $\theta_{\text{low}}$ is a configurable low-battery threshold (default 20%).

A robot is cleared for tasks once it has charged back above a configurable high threshold:

$$\text{batteryCharge} \geq \theta_{\text{high}}$$

This hysteresis prevents oscillation around the threshold. Default $\theta_{\text{high}} = 80\%$.

## Charger Assignment

When a robot needs recharging, the battery manager selects the nearest available charger from `FleetState.chargers`. If all chargers are occupied, the robot is placed in a recharge queue ordered by urgency (lowest battery first).

## `IBatteryManager` Interface

```cpp
class IBatteryManager {
public:
    virtual ~IBatteryManager() = default;
    // Returns true and sets chargerId if robot should route to charge
    virtual bool needsCharging(RobotId id,
                               const vda5050::State& state,
                               ChargerId& outChargerId) const = 0;
    // Returns true once robot is sufficiently charged to resume tasks
    virtual bool isFullyCharged(RobotId id,
                                const vda5050::State& state) const = 0;
    virtual void setThresholds(double low, double high) = 0;
};
```

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `low_threshold` | 0.20 | Battery fraction below which robot routes to charger |
| `high_threshold` | 0.80 | Battery fraction above which robot resumes normal ops |
| `charger_speed` | 0.5/s | Simulated charge rate (fraction per second, for simulation) |

## References

- VDA 5050 `State.batteryState` fields — see `vda5050/docs/theory.md`
- `fleet_monitor/docs/theory.md` — provides FleetState and charger occupancy
