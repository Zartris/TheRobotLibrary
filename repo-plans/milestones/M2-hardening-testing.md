# M2 — Hardening & Testing

**Status:** Not Started  
**Dependencies:** M1  
**Scope:** Comprehensive test coverage, integration tests, bug fixes. No new algorithms.

---

## Goal

Make the M1 codebase rock-solid before adding new features. Stabilize interfaces so future milestones don't cause breaking changes.

---

## Deliverables

### Test Coverage

- [ ] ≥ 80% line coverage on all robotics modules (lcov/gcov)
- [ ] CI publishes coverage report (Codecov or similar)
- [ ] Coverage threshold enforced in CI (fail if < 80%)

### Integration Tests

- [ ] ≥ 3 integration test scenarios:
  - Open room (easy — wide spaces, few obstacles)
  - Maze (narrow corridors, many turns)
  - L-shaped room (dead-end recovery, replanning)
- [ ] Each scenario: robot navigates from start to goal within time limit
- [ ] Regression test: module swap mid-run doesn't crash

### Bug Fixes

- [ ] All bugs discovered during M1 integration fixed
- [ ] No known bugs in issue tracker

### Interface Stabilization

- [ ] Review all `common/interfaces/` — adjust if M1 revealed friction
- [ ] **Resolve SLAM interface:** finalize `ISlamEstimator` (extend `IStateEstimator` with `getMap()`/`getLandmarks()`)
- [ ] **Forward-compatible perception context:** update `ILocalPlanner` to accept a `PerceptionContext` struct (`{scan, grid, tracked_obstacles}`) where `tracked_obstacles` is initially empty — avoids breaking changes when M4 adds obstacle tracking
- [ ] **Perception swappability:** decide whether `IFeatureExtractor`/`IObstacleDetector` interfaces are needed for M4
- [ ] Document interface contracts (pre/post conditions) in header comments
- [ ] After M2, interfaces are **frozen** — no breaking changes without a deprecation cycle

---

## Exit Criteria

1. Coverage ≥ 80% on all robotics modules
2. All 3+ integration scenarios pass in CI
3. Zero known bugs
4. Interfaces documented and frozen

---

## NOT IN

New algorithms, new modules, performance optimization, frontend changes.
