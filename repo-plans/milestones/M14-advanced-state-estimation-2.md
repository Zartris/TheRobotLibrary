# M14 — Advanced State Estimation II

**Status:** Not Started  
**Dependencies:** M2 (hardening; stable common types), M5 (state estimation interfaces and `IStateEstimator`).  
**Scope:** UKF as a drop-in nonlinear upgrade from EKF, plus a reusable Gauss-Newton / LM pose graph optimizer as the SLAM loop closure backend. No cross-dependency between the two modules within M14.

---

## Goal

Fill the estimation gap between EKF and particle filter by adding the Unscented Kalman
Filter (UKF), and provide a reusable, lightweight pose graph optimizer that both
`lidar_slam` and `visual_slam` (M6.5) can adopt for loop closure once M14 is complete.
Both modules are independently implementable within M14.

---

## Modules

### state_estimation/ukf

Unscented Kalman Filter with Merwe scaled sigma points. Accepts process and measurement
functions as `std::function` — same generic style as EKF. Natural upgrade path for strongly
nonlinear systems (SE3 poses, IMU integration).

- [ ] `include/ukf/ukf.hpp` — `UKF : IStateEstimator`, templated on state dimension
- [ ] `include/ukf/ukf_config.hpp` — `UKFConfig` (α, β, κ sigma-point parameters, Q, R)
- [ ] `src/ukf.cpp` — sigma-point propagation, unscented transform for predict + update
- [ ] Generic process function: `std::function<VectorXd(const VectorXd&, const VectorXd&)>`
- [ ] Generic measurement function: `std::function<VectorXd(const VectorXd&)>`
- [ ] `tests/test_ukf.cpp`:
  - Linear system: UKF matches KF solution exactly (unscented transform is exact for linear)
  - Nonlinear pendulum: UKF converges faster than EKF from same initial condition
  - Sigma-point count: always `2n+1` points; weights sum to 1
  - Degenerate covariance: `P` stays positive semi-definite throughout
- [ ] Phase 4.5: `ILogger`, sigma-point spread and residual norm at `DEBUG`, propagation time at `TRACE`
- [ ] Sim: registerable as alternative state estimator alongside EKF
- [ ] ImGui panel: render state covariance ellipse (same panel as EKF)

### state_estimation/pose_graph

Lightweight Gauss-Newton / Levenberg-Marquardt pose graph optimizer. SE2 nodes (robot poses)
+ relative-transform edges (with covariance). Provides the reusable loop closure backend
that both `lidar_slam` and `visual_slam` (M6.5) can optionally adopt after M14 completion.

- [ ] `include/pose_graph/pose_graph.hpp` — `PoseGraph`, `PoseNode` (SE2), `PoseEdge` (relative transform + Ω = Σ⁻¹)
- [ ] `include/pose_graph/pose_graph_optimizer.hpp` — `PoseGraphOptimizer` (GN / LM)
- [ ] `src/pose_graph.cpp` + `src/pose_graph_optimizer.cpp`
- [ ] APIs: `addNode()`, `addEdge()`, `optimize(maxIter)` → `OptimizationResult`
- [ ] Fix gauge freedom: anchor node 0 during optimization
- [ ] `tests/test_pose_graph.cpp`:
  - 4-node square loop (known relative transforms + one loop closure edge) → optimize → node positions within tolerance of ground truth
  - Chain of 10 nodes with no loop closure → optimizer converges without drift (noise = 0)
  - Adding loop closure edge → trajectory correction > 20% improvement in cumulative error
  - Large graph (100 nodes, 5 loop closures) → solver finishes in < 500 ms (Eigen SparseLU)
  - Jacobian numerical check: analytic Jacobian matches finite-difference approximation
- [ ] Phase 4.5: `ILogger`, iteration count + residual norm at `DEBUG`, solve time per iteration at `TRACE`

---

## Deliverables

- [ ] `state_estimation/ukf` module: interface, implementation, tests
- [ ] `state_estimation/pose_graph` module: interface, implementation, tests
- [ ] UKF registerable as alternative estimator in simulation
- [ ] UKF covariance ellipse rendered in simulation app ImGui panel
- [ ] Pose graph optimizer ready for optional adoption by lidar_slam / visual_slam
- [ ] All modules pass Phase 4.5 — Observability gate

---

## Exit Criteria

1. UKF matches KF on linear system, outperforms EKF on nonlinear benchmark (measured convergence steps)
2. Pose graph 4-node square loop converges to < 0.05 m position error
3. Adding loop closure edge → cumulative trajectory drift reduced measurably
4. 100-node graph optimization completes in < 500 ms
5. All unit tests pass, CI green
6. All modules pass Phase 4.5 — Observability gate

---

## NOT IN

SE3 pose graph (→ future M18+), g2o / GTSAM wrappers, IMU factor nodes, landmark nodes
(factor_graph P5 covers the general case), UKF with non-additive noise, augmented-state
UKF-SLAM.
