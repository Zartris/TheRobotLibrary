# factor_graph

General factor graph optimizer. Supports heterogeneous variable nodes (pose, landmark, IMU bias) and arbitrary factor types (odometry, observation, prior), solved via Gauss-Newton with Eigen sparse Cholesky (`SimplicialLDLT`). Designed as a lightweight Eigen-only replacement for g2o/Ceres/GTSAM at small-to-medium graph sizes.

**Milestone:** M21 — Estimation & Test Foundations  
**Status:** Scaffold only — awaiting implementation

## Features

- `FactorGraph` — type-safe variable and factor registry
- Variable types: `PoseVariable` (SE2), `LandmarkVariable` (x, y), `BiasVariable`
- Factor types: `OdometryFactor`, `ObservationFactor` (bearing-range), `PriorFactor`
- Gauss-Newton solver: `J^T Ω J Δx = −J^T Ω r` solved via Eigen `SimplicialLDLT`
- Gauge freedom: fix first variable or via explicit `setFixed(id)`
- Post-M21: `PoseGraph (M14)` can optionally be re-implemented as a thin wrapper

## Dependencies

- `common` (logging, types)
- Eigen3 (`SimplicialLDLT`, sparse matrix support)
