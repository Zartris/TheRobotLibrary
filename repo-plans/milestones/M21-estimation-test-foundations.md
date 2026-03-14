# M21 — Estimation & Test Foundations

**Status:** Not Started  
**Dependencies:** M14 (`pose_graph` GN/LM optimizer as structural predecessor; `common` types) for `factor_graph`. `common/noise_models` has zero external dependencies.  
**Scope:** Two infrastructure/utility modules. `factor_graph` is the general graph estimator that generalizes `pose_graph` (M14); `noise_models` provides reproducible seeded noise generation for testing across all modules. Both are "Foundations II" — they improve rigor throughout the library without being prerequisites for most other modules.

---

## Goal

M14's `pose_graph` is a specialized SE2-pose-only optimizer. `factor_graph` generalizes this to support heterogeneous variable types (poses, landmarks, IMU biases) and arbitrary factor types (odometry, observation, prior) — the pattern underlying g2o, GTSAM, and Ceres. Once available, SLAM variants (M6.5), VIO (M15), and future perception modules can optionally adopt it as a consistent backend. `noise_models` solves a persistent test-quality problem: all modules currently inject noise ad hoc with non-seeded RNGs, making tests non-reproducible. Promoting noise injection to a first-class `common/` utility is a small effort with system-wide payoff.

**Note — Q3/Q5 conflict resolution:** The Q3 recommendation placed `noise_models` with `charging_station`; the Q5 recommendation placed it with `factor_graph`. These are mutually exclusive. M21 follows Q5: domain cohesion (two infrastructure utilities) produces a more teachable milestone. `charging_station` becomes the solo M24 fleet milestone.

---

## Modules

### state_estimation/factor_graph

General factor graph optimizer using a Gauss-Newton iterative solver with sparse Cholesky (Eigen `SimplicialLDLT`). Supports heterogeneous variable nodes and arbitrarily typed factor nodes. Lightweight Eigen-only replacement for g2o/Ceres/GTSAM at small graph sizes.

- [ ] `include/factor_graph/factor_graph.hpp` — `FactorGraph`; `addVariable<N>(id)` → `Variable<N>&`; `addFactor(factor)` → void; `optimize(maxIter, tol)` → `OptimizationResult`
- [ ] `include/factor_graph/variable.hpp` — `Variable<int N>` (id, `Eigen::VectorXd` value, fixed flag); `PoseVariable` (SE2: x, y, θ), `LandmarkVariable` (x, y), `BiasVariable` (accel_bias, gyro_bias)
- [ ] `include/factor_graph/factor.hpp` — `IFactor` interface: `residual()`, `jacobian()`, `informationMatrix()`; `OdometryFactor` (binary pose-pose), `ObservationFactor` (pose-landmark bearing-range), `PriorFactor` (unary, any variable type)
- [ ] `src/factor_graph.cpp` — GN iteration: assemble sparse Jacobian `J` + residual `r`; normal equations `J^T Ω J Δx = −J^T Ω r`; Eigen `SimplicialLDLT` solve; update variables; stop on `‖Δx‖ < tol`
- [ ] Gauge freedom: first non-fixed variable held fixed OR explicit `setFixed(id)` call
- [ ] `tests/test_factor_graph.cpp`:
  - 2-node SE2 chain + `OdometryFactor`: optimizer converges; both poses match ground truth within 1e-6
  - 4-node SE2 loop (3 odometry + 1 loop closure): closed-loop poses converge; cumulative error < 0.05 m
  - Pose + landmark + `ObservationFactor`: landmark converges from corrupted initial guess within 10 iterations
  - GN convergence: `‖Δx‖` decreasing monotonically on noise-free problem
  - Jacobian check: analytic Jacobian matches finite-difference (< 1e-5 max element error)
  - Large graph (100 SE2 nodes, 10 loop closures): `optimize()` completes in < 1 s
- [ ] Phase 4.5: `ILogger`, iteration count + final residual norm at `DEBUG`, per-iteration solve time at `TRACE`
- [ ] Integration note: `PoseGraph (M14)` can be re-implemented as a thin wrapper over `FactorGraph` post-M21; not a requirement of M21 itself

### common/noise_models

Seeded, reproducible noise injection primitives. All types are templated on scalar type `T` and accept an explicit RNG seed for deterministic test sequences. **Header-only INTERFACE library.**

- [ ] `include/common/noise_models.hpp` — `GaussianNoise<T>` (configurable `σ`, `std::mt19937` seeded RNG); `UniformNoise<T>` (configurable `[lo, hi]`); `OutlierInjector<T>` (base noise model + outlier probability `p` + outlier range)
- [ ] Batch API: `sampleN(n)` → `std::vector<T>`; `addTo(Eigen::VectorXd&)` → in-place noise injection
- [ ] `tests/test_noise_models.cpp`:
  - Gaussian mean: 10000 samples → `|mean| < 0.01·σ`; variance: `|var − σ²| < 0.05·σ²`
  - Uniform coverage: bucket histogram of 1000 samples → all buckets within 20% of uniform expectation
  - Seeded reproducibility: two `GaussianNoise<double>` with same seed → identical `sampleN(100)` sequence
  - Different seeds → sequences differ at first differing sample
  - `OutlierInjector`: 1000 samples with `p = 0.1` → outlier count in `[70, 130]`
  - `addTo(Eigen::VectorXd&)`: adds correct noise dimension-wise; does not modify vector size
- [ ] Phase 4.5: `ILogger`, seed value at `DEBUG` on construction; not logged per-sample

---

## Deliverables

- [ ] `state_estimation/factor_graph` module: interface, implementation, tests
- [ ] `common/noise_models` module: header-only interface, tests
- [ ] Factor graph 4-node loop closure verified
- [ ] Analytic Jacobians verified against finite-difference
- [ ] Gaussian noise mean/variance convergence verified over 10000 samples
- [ ] Same seed → identical noise sequence verified
- [ ] All modules pass Phase 4.5 — Observability gate

---

## Exit Criteria

1. Factor graph 4-node SE2 loop closure converges to < 0.05 m position error
2. Analytic Jacobians of all factor types match finite-difference (< 1e-5 max element error)
3. 100-node sparse factor graph optimizes in < 1 s on CI hardware
4. Gaussian noise mean/variance converges to configured parameters over 10000 samples
5. Same seed → identical noise sequence (reproducibility verified in tests)
6. All unit tests pass, CI green
7. All modules pass Phase 4.5 — Observability gate (state transitions at `DEBUG`, metrics at `TRACE`)

---

## NOT IN

SE3 variable types, robust cost functions (Huber, Cauchy), marginalization / Schur complement, GTSAM/g2o file format import, online incremental updates (iSAM2 style), pink/brown correlated noise models.
