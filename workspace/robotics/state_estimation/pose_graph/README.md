# pose_graph

Lightweight Gauss-Newton / Levenberg-Marquardt pose graph optimizer for SE2 (2D SLAM).
Provides the reusable loop closure backend that both `lidar_slam` and `visual_slam` can
optionally adopt after M14 completion.

## Algorithm

- **Type:** Graph-based SLAM backend / pose graph optimizer
- **Nodes:** `PoseNode` — SE2 robot poses
- **Edges:** `PoseEdge` — relative transform constraints with information matrix Ω = Σ⁻¹
- **Optimizer:** `PoseGraphOptimizer` — Gauss-Newton with optional Levenberg-Marquardt damping

## Dependencies

- `common` (logging)
- `Eigen3` (SparseLU for large graph solve)

## Usage

```cpp
#include <pose_graph/pose_graph.hpp>
#include <pose_graph/pose_graph_optimizer.hpp>

PoseGraph graph;
graph.addNode({0, Pose2D{0,0,0}});       // anchor node
graph.addNode({1, Pose2D{1,0,0}});
graph.addEdge({0, 1, relative_T, Omega});
graph.addEdge({3, 0, loop_T, Omega});   // loop closure

PoseGraphOptimizer opt(common::getLogger("pose_graph"));
auto result = opt.optimize(graph, /*maxIter=*/50);
```

## Milestone

Part of **M14 — Advanced State Estimation II**.  
See `repo-plans/modules/pose_graph.md` for full task checklist.
