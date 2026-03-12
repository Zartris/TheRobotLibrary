# Module: common

**Milestone:** M1 — Minimum Viable Robot (Sub-phase M1-A)  
**Status:** Not Started  
**Depends on:** —

---

### Phase 1 — Interface Design

- [ ] `include/common/types.hpp` — `Pose2D`, `Point2D`, `Twist`, `Path`
- [ ] `include/common/transform2d.hpp` — `Transform2D` (compose, inverse, transform point)
- [ ] `include/common/occupancy_grid.hpp` — `OccupancyGrid` (width, height, resolution, origin, cells)
- [ ] `include/common/laser_scan.hpp` — `LaserScan` (angle_min/max/inc, ranges, intensities)
- [ ] `include/common/geometry.hpp` — `angleWrap`, `distance`, `normalizeAngle`, `lerp`
- [ ] `include/common/interfaces/i_controller.hpp`
- [ ] `include/common/interfaces/i_global_planner.hpp`
- [ ] `include/common/interfaces/i_local_planner.hpp`
- [ ] `include/common/interfaces/i_state_estimator.hpp`
- [ ] `include/common/interfaces/i_velocity_profiler.hpp`
- [ ] `include/common/kinematics/i_kinematic_model.hpp` — `IKinematicModel`: `step()`, `getControlLimits()`, `toTwist()`, `fromTwist()`
- [ ] `include/common/kinematics/differential_drive.hpp` — `DifferentialDrive : IKinematicModel`

> **Map types live here.** `OccupancyGrid` is the M1 map representation. Later milestones may add `PointCloudMap`, `FeatureMap`, etc. — all defined in common so modules share one representation.

### Phase 2 — Failing Tests (Red)

- [ ] `tests/CMakeLists.txt` — Catch2 target
- [ ] `tests/test_types.cpp` — construction, equality, basic ops
- [ ] `tests/test_transform2d.cpp` — compose, inverse, transform point
- [ ] `tests/test_geometry.cpp` — angle_wrap (π, -π, >2π), distance, normalize
- [ ] `tests/test_differential_drive.cpp` — twist → pose, wheel speeds ↔ twist round-trip

### Phase 3 — Implementation (Green)

- [ ] `src/transform2d.cpp`
- [ ] `src/geometry.cpp`
- [ ] `src/kinematics/differential_drive.cpp`

### Phase 4 — Passing Tests

- [ ] All tests pass
- [ ] Interfaces compile as INTERFACE targets

### Phase 5 — Simulation Integration

_N/A — common is a dependency, not wired directly into sim loop._

### Phase 6 — Frontend Visualization

_N/A — common has no visual representation._

### Phase 7 — Docs Polish

- [ ] Update `README.md` with type documentation + usage examples
- [ ] Move this file to `repo-plans/modules/done/common.md`
