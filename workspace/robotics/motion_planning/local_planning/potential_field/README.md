# potential_field

Artificial potential field local planner. Combines an attractive parabolic/conic field toward the goal with repulsive inverse-square fields from obstacles. Includes local minima escape via random perturbation when progress stalls.

**Milestone:** M20 — Planning Upgrades III  
**Status:** Scaffold only — awaiting implementation

## Features

- `PotentialFieldPlanner : ILocalPlanner` — drop-in local planner
- Attractive force: parabolic within `d_switch`, conic beyond (prevents oscillation near goal)
- Repulsive force: inverse-square from each obstacle within influence radius `d_0`
- Local minima escape: random perturbation after N consecutive stall steps
- Obstacle distance via `OccupancyGrid` nearest-obstacle lookup

## Dependencies

- `common` (logging, types)
- Eigen3 (2D vector math)
