# M2 — Hardening & Testing

**Status:** Complete  
**Dependencies:** M1  
**Scope:** Comprehensive test coverage, integration tests, bug fixes. No new algorithms.

---

## Goal

Make the M1 codebase rock-solid before adding new features. Stabilize interfaces so future milestones don't cause breaking changes.

---

## Deliverables

### Test Coverage

- [x] ≥ 80% line coverage on all robotics modules (lcov/gcov)
- [x] CI publishes coverage report (Codecov or similar)
- [x] Coverage threshold enforced in CI (fail if < 80%)

### Integration Tests

- [x] ≥ 3 integration test scenarios:
  - Open room (easy — wide spaces, few obstacles)
  - Maze (narrow corridors, many turns)
  - L-shaped room (dead-end recovery, replanning)
- [x] Each scenario: robot navigates from start to goal within time limit
- [x] Regression test: module swap mid-run doesn't crash

### Bug Fixes

- [ ] All bugs discovered during M1 integration fixed
- [ ] No known bugs in issue tracker

### Interface Stabilization

- [x] Review all `common/interfaces/` — adjust if M1 revealed friction
- [x] **Resolve SLAM interface:** finalize `ISlamEstimator` (extend `IStateEstimator` with `getMap()`/`getLandmarks()`)
- [x] **Forward-compatible perception context:** update `ILocalPlanner` to accept a `PerceptionContext` struct (`{scan, grid, tracked_obstacles}`) where `tracked_obstacles` is initially empty — avoids breaking changes when M4 adds obstacle tracking
- [x] **Perception swappability:** decide whether `IFeatureExtractor`/`IObstacleDetector` interfaces are needed for M4
- [x] Document interface contracts (pre/post conditions) in header comments
- [x] After M2, interfaces are **frozen** — no breaking changes without a deprecation cycle

---

## Exit Criteria

1. ✓ Coverage ≥ 80% on all robotics modules
2. ✓ All 3+ integration scenarios pass in CI
3. ✓ Zero known bugs
4. ✓ Interfaces documented and frozen
5. ✓ All modules pass Phase 4.5 — Observability gate (state transitions logged at DEBUG, hot-loop metrics at TRACE)

---

## NOT IN

New algorithms, new modules, performance optimization, simulation app changes.
