# M5 — State Estimation Upgrades

**Status:** Not Started  
**Dependencies:** M2 (stable IStateEstimator interface)  
**Scope:** Particle filter (MCL) as alternative to EKF. Swappable at runtime.

---

## Goal

Two state estimators available — EKF (from M1) and particle filter — both implementing `IStateEstimator`. User picks at runtime. Demonstrates the value of the swappable interface architecture for estimation.

---

## Module: particle_filter

Monte Carlo Localization (MCL) for known-map localization.

- [ ] `include/particle_filter/particle_filter.hpp` — `ParticleFilter : IStateEstimator`
- [ ] `src/particle_filter.cpp`
- [ ] Motion model: sample-based odometry model (probabilistic robotics Ch. 5)
- [ ] Sensor model: likelihood field model (precompute distance transform of map)
- [ ] Resampling: systematic resampling (low variance)
- [ ] `tests/test_particle_filter.cpp`:
  - Particles initialized uniformly → converge to true pose after 10 updates
  - Zero-motion predict → particles stay put (with noise spread)
  - Perfect measurement → particles cluster at true pose
  - Kidnapped robot (teleport) → particles eventually recover (given enough updates)
  - Configurable particle count affects convergence speed

---

## Sim & Frontend Integration

- [ ] Estimator hot-swap: `PUT /api/robot/estimator {"type": "particle_filter"}`
- [ ] WebSocket state includes particle data when particle_filter active (sampled subset for bandwidth)
- [ ] Frontend: particle cloud visualization (small dots with heading ticks)
- [ ] Frontend: toggle between EKF covariance ellipse and particle cloud
- [ ] Mini-demo: side-by-side EKF vs particle filter comparison on same scenario

---

## Deliverables

- [ ] particle_filter module: interface, implementation, tests
- [ ] Estimator hot-swap in sim
- [ ] Frontend particle cloud rendering
- [ ] Demo: EKF vs MCL comparison

## Exit Criteria

1. Particle filter localizes robot with ≤ 0.2m mean error in known-map scenario
2. Swap EKF ↔ particle_filter mid-run without crash
3. All unit tests pass
4. Visual difference clear in frontend (ellipse vs particle cloud)

## NOT IN

SLAM, adaptive particle count (AMCL), multi-hypothesis tracking. Keep it simple.
