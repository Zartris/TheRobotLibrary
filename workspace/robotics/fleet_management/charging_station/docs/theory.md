# Charging Station Theory

## Assignment Problem

The charging station assignment problem is: given $S$ stations and $R$ robots requesting charging, assign each robot to a station to minimize waiting time subject to capacity constraints (each station serves one robot at a time).

## Priority Queue Ordering

Charge requests are ordered by:

1. **Urgency flag** (descending): `urgency = true` robots jump to the front of the queue regardless of battery level
2. **Battery level** (ascending): among equally-urgent robots, lower battery level gets priority

This ordering is implemented as a max-priority queue (std::priority_queue) with a custom comparator.

## Linear Charge Model

Estimated charge completion time for a robot at state-of-charge $\text{soc} \in [0, 1]$:

$$t_{complete} = \frac{C_{\text{kWh}} \cdot (1 - \text{soc})}{P_{\text{kW}}}$$

where $C_{\text{kWh}}$ is the battery capacity and $P_{\text{kW}}$ is the station's charging power. This is a constant-current approximation; actual Li-ion charging has a CC-CV profile, but the linear model is sufficient for fleet scheduling estimates.

## Station Health States

| Status | Description |
|--------|-------------|
| `IDLE` | Available for assignment |
| `OCCUPIED` | Currently charging a robot |
| `FAULT` | Maintenance fault; excluded from assignment |

State transitions: `IDLE → OCCUPIED` on `requestCharging()`, `OCCUPIED → IDLE` on `releaseStation()`, any state → `FAULT` on `setFault()`, `FAULT → IDLE` on `clearFault()`.

## References

- Liu & Santos, "Optimal Scheduling for Electric Vehicle Charging Stations," IEEE Transactions on Smart Grid, 2012
