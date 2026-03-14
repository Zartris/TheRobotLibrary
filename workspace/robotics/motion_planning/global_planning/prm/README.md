# prm

Probabilistic Roadmap Planner (PRM). Multi-query sampling-based global planner: the
roadmap is built once (offline or at startup) and reused across many planning queries.
Local planner uses straight-line collision checking against `OccupancyGrid`. Query phase
uses Dijkstra / A* on the roadmap graph.

## Algorithm

- **Type:** Sampling-based global planner (multi-query)
- **Interface:** `PRMPlanner : IGlobalPlanner`
- **Config:** `PRMConfig` — max_nodes, k_nearest, max_edge_length, num_samples

## Dependencies

- `common` (IGlobalPlanner interface, OccupancyGrid, logging)
- `Eigen3`

## Usage

```cpp
#include <prm/prm_planner.hpp>
#include <prm/prm_config.hpp>

PRMConfig cfg;
cfg.max_nodes      = 500;
cfg.k_nearest      = 10;
cfg.max_edge_length = 2.0; // metres
cfg.num_samples    = 500;

PRMPlanner planner(cfg, common::getLogger("prm"));
planner.buildRoadmap(occupancy_grid);          // once

auto path1 = planner.plan(start1, goal1);      // reuses roadmap
auto path2 = planner.plan(start2, goal2);      // no rebuild
```

## Milestone

Part of **M16 — Planning Upgrades II**.  
See `repo-plans/modules/prm.md` for full task checklist.
