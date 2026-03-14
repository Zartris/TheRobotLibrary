# Lattice Planner Theory

## State Lattice

A state lattice discretizes the configuration space $\mathcal{C} = (x, y, \theta)$ into a regular grid. Each node is a tuple $(g_x, g_y, h_{idx})$ where $g_x, g_y$ are grid cell indices and $h_{idx}$ is a discretized heading index.

## Motion Primitives

For each heading, a set of **motion primitives** is pre-computed: kinematically-feasible arcs (straight + left/right curves of varying radius) that start at one lattice node orientation and end at a neighbouring node orientation. Each primitive is a sequence of `Pose2D` waypoints with an associated traversal cost.

Pre-computation ensures:
- All primitives satisfy kinematic constraints (curvature bounds)
- Heading continuity: the end heading of one primitive exactly matches the start heading of the next at the joining node

## Graph Search

After collision-checking each primitive against the `OccupancyGrid`, the valid graph is searched with A* (heuristic: 2D Euclidean distance to goal) or Dijkstra (uniform cost).

**Advantages over RRT for structured environments:**
- Deterministic (reproducible paths)
- Kinematically consistent trajectories
- Faster in environments with road/corridor topology

## References

- Pivtoraiko et al., "Differentially Constrained Mobile Robot Motion Planning in State Lattices," JFR 2009
- McNaughton et al., "Motion Planning for Autonomous Driving with a Conformal Spatiotemporal Lattice," ICRA 2011
