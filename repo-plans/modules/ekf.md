# Module: ekf

**Milestone:** M1 — Minimum Viable Robot (Sub-phase M1-E)  
**Status:** Not Started  
**Depends on:** common (Eigen via common)

---

### Phase 1 — Interface Design

- [ ] `include/ekf/ekf2d.hpp` — `EKF2D : IStateEstimator` with Config struct (Q, R matrices)

### Phase 2 — Failing Tests (Red)

- [ ] `tests/CMakeLists.txt`
- [ ] `tests/test_ekf2d.cpp`:
  - Predict with zero twist → pose unchanged, covariance grows
  - Predict with known twist for 1s → expected pose (Euler integration)
  - Update with perfect measurement → covariance shrinks, pose corrected
  - Predict→update cycle converges to true pose with noisy inputs

### Phase 3 — Implementation (Green)

- [ ] `src/ekf2d.cpp` — motion model Jacobian, sensor model Jacobian, predict/update

### Phase 4 — Passing Tests

- [ ] All tests pass

### Phase 5 — Simulation Integration

- [ ] EKF receives odometry (predict) and landmark/scan data (update) each tick (M1-K)
- [ ] Selectable via `PUT /api/robot/estimator {"type": "ekf"}`

### Phase 6 — Frontend Visualization

- [ ] Covariance ellipse rendered around estimated robot pose (M1-L)

### Phase 7 — Docs Polish

- [ ] Update README.md with EKF usage examples
- [ ] Add implementation notes to docs/theory.md
- [ ] Move this file to `repo-plans/modules/done/ekf.md`
