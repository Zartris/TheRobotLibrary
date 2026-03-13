# Fleet Management — Theory

## Overview

Fleet management addresses the coordination layer **above** individual robot autonomy.
A single robot can navigate, localize, and avoid obstacles independently. A fleet of
robots additionally needs: task distribution, centralized or decentralized state awareness,
resource arbitration (chargers, dock stations), and compliance with industry protocols.

## VDA 5050

VDA 5050 is an open standardized interface specification (v2.0, 2022) for communication
between **fleet management systems (FMS)** and **automated guided vehicles (AGVs)** /
**autonomous mobile robots (AMRs)**. It defines a set of MQTT-based message types:

| Message | Direction | Purpose |
|---------|-----------|---------|
| `Order` | FMS → Robot | Assign a sequence of nodes and edges to traverse |
| `InstantAction` | FMS → Robot | Immediate commands (pause, cancel, pick, drop) |
| `State` | Robot → FMS | Current robot state (position, battery, load, errors) |
| `Connection` | Robot → FMS | Heartbeat/connection status |
| `Visualization` | Robot → FMS | High-rate position stream for visualization |

In this library, VDA 5050 types are implemented as C++ structs with nlohmann/json
serialization. The transport layer (MQTT) is not implemented — structs are exchanged
via the existing REST/WebSocket API.

## Task Allocation Strategies

**Greedy (nearest-robot) allocation:**
- For each incoming task, assign to the robot with the shortest estimated travel distance.
- O(N·M) per allocation (N robots, M tasks). Simple and predictable.

**Auction-based allocation:**
- Robots bid on tasks based on their estimated cost (distance + current load).
- Central allocator awards task to lowest bidder. Produces near-optimal assignments for
  heterogeneous robot types or task-specific capabilities.
- Based on the Contract Net Protocol (FIPA/CNET).

## Fleet Monitor

The fleet monitor aggregates per-robot state updates into a single `FleetState` snapshot.
Consumers (task allocator, battery manager, frontend) query `FleetState` rather than
polling individual robots, decoupling the coordination logic from update timing.

## Battery Management

Battery management tracks state-of-charge (SoC) per robot and:
1. Triggers a **recharge decision** when SoC drops below a configurable threshold.
2. Selects the **nearest available charger station** from the map.
3. Issues a VDA 5050 `Order` to route the robot to the charger.

The policy is configurable: threshold-based (simple), predictive-remaining-distance
(intermediate), or deadline-aware (advanced).

## References

- VDA 5050 v2.0 Specification, Verband der Automobilindustrie, 2022.
- Koenig & Howard, "Design and Use Paradigms for Gazebo, An Open-Source Multi-Robot
  Simulator," *IEEE/RSJ IROS*, 2004. (Fleet sim design patterns)
- Smith, "The Contract Net Protocol," *IEEE Transactions on Computers*, 1980.
