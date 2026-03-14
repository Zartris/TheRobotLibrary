# lattice_planner

State lattice global planner. Pre-computes kinematically-feasible motion primitives (straight + curved arcs) over a discrete `(x, y, θ)` grid and searches the resulting graph with A* or Dijkstra. Well-suited for structured environments (roads, corridors) and Ackermann vehicles.

**Milestone:** M23 — Lattice Planning & Semantic Vision  
**Status:** Scaffold only — awaiting implementation

## Features

- `LatticePlanner : IGlobalPlanner` — graph-based planner over discrete state lattice
- Pre-computed motion primitives per heading: straight + left/right arcs of varying radius
- Collision checking via `OccupancyGrid` for each primitive
- A* or Dijkstra graph search (configurable via `use_astar` flag)
- Heading continuity guaranteed at primitive joining nodes
- Returns `std::nullopt` when no path exists

## Dependencies

- `common` (logging, types, `IGlobalPlanner`, `OccupancyGrid`)
- Eigen3 (vector math)
