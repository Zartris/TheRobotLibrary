# Fleet Management

Modules for coordinating fleets of robots in real-world deployment scenarios. Fleet
management sits above the per-robot autonomy stack (planning, control, perception) and
handles task-level coordination, protocol compliance, and fleet-wide resource management.

## Architecture

Fleet management is a **pure library domain** — it has no dependency on simulation or
frontends. The simulation server links fleet_management modules during M12 to expose
fleet management via the existing Crow REST/WS server.

```
                  ┌─────────────────────────────────┐
                  │      Simulation Backend          │
                  │  ┌──────────────────────────┐    │
                  │  │  /api/fleet/* endpoints  │    │
                  │  └──────────┬───────────────┘    │
                  └─────────────┼───────────────────-┘
                                │ uses
         ┌──────────────────────┼──────────────────────┐
         │       fleet_management library               │
         │  ┌───────────┐  ┌────────────────┐          │
         │  │  vda5050  │  │ task_allocation │          │
         │  └───────────┘  └────────────────┘          │
         │  ┌──────────────┐  ┌──────────────────────┐  │
         │  │ fleet_monitor│  │ battery_management   │  │
         │  └──────────────┘  └──────────────────────┘  │
         └──────────────────────────────────────────────┘
```

## Sub-modules

| Sub-module             | Description                                                      |
|------------------------|------------------------------------------------------------------|
| `vda5050/`             | VDA 5050 v2.0 message types + nlohmann/json serialization        |
| `task_allocation/`     | `ITaskAllocator` + greedy/auction allocators                     |
| `fleet_monitor/`       | `IFleetMonitor` — aggregates per-robot state into `FleetState`   |
| `battery_management/`  | `IBatteryManager` — charge level tracking, route-to-charger      |

## Theory

See `docs/theory.md` for fleet coordination strategies and VDA 5050 protocol overview.
