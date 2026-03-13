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

#### Sub-module: `common/transforms` (INTERFACE, header-only)

> Pre-scaffolded at `workspace/robotics/common/transforms/`. CMake wiring already present.
> SE2/SE3/SO3 Lie-group types extending the existing `Transform2D` for 3D use cases (M6+).

- [ ] `include/transforms/transforms.hpp` — `SE2` (x, y, θ; compose, inverse, transformPoint, interpolate, toMatrix/fromMatrix), `SE3` (translation + quaternion; compose, inverse, transformPoint, toSE2, toMatrix/fromMatrix), `SO3` (quaternion; compose, inverse, rotate, toMatrix, fromAxisAngle, fromRPY/toRPY) — all header-only
- [ ] `tests/transforms_tests.cpp` — SE2 compose + inverse round-trip, SE3 compose + inverse, SO3 fromRPY/toRPY round-trip, SE3::toSE2 projection, all within 1e-9 tolerance
- [ ] Verify `add_subdirectory(transforms)` is present in `workspace/robotics/common/CMakeLists.txt` (already done — confirm only)

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

### Phase 4.5 — Observability

> **This phase gates module completion.** Both human developers and AI agents must be able
> to verify correct behavior through logs and metrics — not just frontend visuals.

- [ ] `ILogger` injected into module constructor via `common::getLogger("common")` (mockable in tests)
- [ ] All state transitions logged at `DEBUG` level (init, reset, mode changes, error paths)
- [ ] Hot-loop performance metrics logged at `TRACE` level (cycle time per iteration in µs, iteration count)
- [ ] At least one test asserts expected log lines appear via stdout capture
- [ ] Zero `ERROR`-level log entries during all nominal test runs

```bash
# Confirm logging output during tests:
cmake --build build --target common_tests
cd build && ctest -R common --output-on-failure 2>&1 | grep "\[DEBUG\]\|\[TRACE\]"
```

### Phase 5 — Simulation Integration

_N/A — common is a dependency, not wired directly into sim loop._

### Phase 6 — Frontend Visualization

_N/A — common has no visual representation._

### Phase 7 — Docs Polish

- [ ] Update `README.md` with type documentation + usage examples
- [ ] Move this file to `repo-plans/modules/done/common.md`

---

## M6 Addition: `common/camera.hpp`

**Milestone:** M6 — Visual Perception Building Blocks
**Tracked here** because `camera.hpp` is a header added to the `common` library target.

- [ ] `include/common/camera.hpp` — `CameraIntrinsics` (fx, fy, cx, cy, width, height; `project()` → `std::optional<Eigen::Vector2d>`, `unproject()` → `Eigen::Vector3d`, `K()` → `Eigen::Matrix3d`), `CameraFrame` (timestamp `double`, width, height, data `std::vector<uint8_t>` row-major grayscale)
- [ ] Tests: `project` round-trip; behind-camera returns `std::nullopt`; `K()` diagonal matches intrinsics; `CameraFrame::valid()` on empty data returns false
