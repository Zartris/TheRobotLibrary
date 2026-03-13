# Task Allocation — Theory

## Problem Definition

Given a set of $N$ robots and $M$ pending tasks (VDA 5050 Orders), assign each task to exactly one robot such that an objective function (total travel distance, makespan, priority-weighted cost) is minimised while respecting robot capability, availability, and battery constraints.

## Algorithms

### Greedy Nearest-Robot Allocator

Assigns each task to the available robot with the smallest Euclidean distance to the task's first node. $O(N \cdot M)$ per allocation cycle. Optimal for single-task, homogeneous fleets.

### Auction-Based Allocator (Contract Net Protocol)

Implements the Contract Net Protocol (CNP):
1. **Announce:** FMS broadcasts task to all eligible robots.
2. **Bid:** Each robot computes and returns a cost estimate (time to reach + execution time).
3. **Award:** FMS selects lowest-bid robot and sends the VDA 5050 Order.

CNP is distributed-compatible and handles heterogeneous fleets well. It does not guarantee global optimality but runs in $O(N)$ communication rounds.

### Hungarian Method (future)

For batch allocation of $M$ tasks to $N$ robots, the Hungarian algorithm solves the assignment problem in $O(N^3)$ and guarantees a globally cost-optimal matching.

## `ITaskAllocator` Interface

```cpp
class ITaskAllocator {
public:
    virtual ~ITaskAllocator() = default;
    // Assign task to best robot; returns assigned RobotId or nullopt if no robot available
    virtual std::optional<RobotId>
        assignTask(const FleetState& fleet, const vda5050::Order& task) = 0;
    virtual std::string allocatorType() const = 0;
};
```

## References

- Smith (1980). "The Contract Net Protocol" — IEEE Transactions on Computers
- Kuhn (1955). "The Hungarian Method" — Naval Research Logistics Quarterly
- VDA 5050 Order specification — see `vda5050/docs/theory.md`
