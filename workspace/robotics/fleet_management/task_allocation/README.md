# Task Allocation

`ITaskAllocator` and concrete implementations for assigning VDA 5050 `Order` tasks to
robots in a fleet.

## Implementations

| Class | Strategy | Best For |
|-------|----------|---------|
| `GreedyAllocator` | Assign to nearest available robot | Homogeneous fleets, simple scenarios |
| `AuctionAllocator` | Contract Net Protocol bidding | Heterogeneous fleets, load balancing |

## Interface

```cpp
#include <task_allocation/i_task_allocator.hpp>

class ITaskAllocator {
public:
    virtual std::expected<RobotId, AllocationError>
    assignTask(const FleetState&, const vda5050::Order&) = 0;
    virtual ~ITaskAllocator() = default;
};
```
