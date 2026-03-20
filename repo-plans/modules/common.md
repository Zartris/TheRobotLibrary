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
> to verify correct behavior through logs and metrics — not just visual inspection.

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

---

## M3.5 Addition: `common/robot/` — Physical Parameter Types

**Milestone:** M3.5 — Vehicle Dynamics
**Tracked here** because these are header-only additions to the `common` library target.

- [ ] `include/common/robot/vehicle_params.hpp` — `VehicleParams` (mass, yaw inertia Iz, CoG position lf/lr, wheel_radius, track_width), `TireParams` (cornering stiffness Cf/Cr, max friction mu), `MotorParams` (stall_torque, no_load_speed, gear_ratio, efficiency), `WheelConfig` (radius, width, position relative to CoG)
- [ ] Tests: construction with defaults; VehicleParams validates positive mass/inertia

## M3.5 Addition: `common/kinematics/` — IDynamicModel Interface

**Milestone:** M3.5 — Vehicle Dynamics
**Tracked here** because `IDynamicModel` is added alongside existing `IKinematicModel` in `common/kinematics/`.

- [ ] `include/common/kinematics/i_dynamic_model.hpp` — `IDynamicModel`: `step(DynamicState, VehicleInput, TerrainProperties, dt) → DynamicState`, `getParams() → VehicleParams`, `setParams(VehicleParams)`; `DynamicState` (pose SE2, vx, vy, omega, acceleration, `DynamicDiagnostics`); `VehicleInput` (longitudinal force, steering angle, braking force)
- [ ] Tests: DynamicState default construction; VehicleInput zero-init

## M3.5 Addition: `common/environment/` — Terrain Types

**Milestone:** M3.5 — Vehicle Dynamics
**Tracked here** because these are header-only additions to the `common` library target.

- [ ] `include/common/environment/terrain.hpp` — `TerrainProperties` (mu, rolling_resistance, slope_angle), `SlipDetector::detect(commanded_vel, measured_vel) → SlipEvent`, `SlipEvent` (detected bool, slip_ratio, severity)
- [ ] Tests: SlipDetector with matching velocities → no slip; divergent velocities → slip detected; TerrainProperties default mu=1.0

---

## MuJoCo Architecture Addition: `common/types_3d/` — 3D Geometric Types

**Milestone:** M1 (MuJoCo architecture)
**Tracked here** because these are header-only additions to the `common` library target, required by the simulation bridge layer.

- [ ] `include/common/types_3d/pose3d.hpp` — `Pose3D` (position `Eigen::Vector3d`, orientation `Eigen::Quaterniond`; `identity()`, `inverse()`, `compose()`) in `robotlib` namespace
- [ ] `include/common/types_3d/transform3d.hpp` — `Transform3D` (SE3: `Eigen::Isometry3d` wrapper; `compose()`, `inverse()`, `transformPoint()`, `toMatrix()`, `fromMatrix()`) in `robotlib` namespace
- [ ] `include/common/types_3d/quaternion.hpp` — `Quaternion` utilities: `fromRPY()`, `toRPY()`, `fromAxisAngle()`, `normalize()`, `slerp()` — all header-only in `robotlib` namespace
- [ ] `include/common/types_3d/twist3d.hpp` — `Twist3D` (linear `Eigen::Vector3d`, angular `Eigen::Vector3d`) in `robotlib` namespace
- [ ] `include/common/types_3d/wrench.hpp` — `Wrench` (force `Eigen::Vector3d`, torque `Eigen::Vector3d`) in `robotlib` namespace
- [ ] `include/common/types_3d/terrain_pose.hpp` — `TerrainPose` (Pose3D on terrain surface + surface normal `Eigen::Vector3d` + slope angle) in `robotlib` namespace
- [ ] Tests: quaternion round-trip `fromRPY/toRPY`; `Transform3D` compose + inverse within 1e-9; `Pose3D::inverse()` identity check; `Twist3D` zero-init

---

## MuJoCo Architecture Addition: `common/sensors/` — Sensor Reading Types

**Milestone:** M1 (MuJoCo architecture)
**Tracked here** because these are header-only sensor data types shared between simulation bridge and robotics modules.

- [ ] `include/common/sensors/lidar_scan.hpp` — `LidarScan` (timestamp `double`, angle_min, angle_max, angle_increment, range_min, range_max, `std::vector<float> ranges`, `std::vector<float> intensities`) in `robotlib` namespace
- [ ] `include/common/sensors/imu_reading.hpp` — `ImuReading` (timestamp `double`, linear_acceleration `Eigen::Vector3d`, angular_velocity `Eigen::Vector3d`, orientation `Eigen::Quaterniond`) in `robotlib` namespace
- [ ] `include/common/sensors/camera_frame.hpp` — `CameraFrame` (timestamp `double`, width `int`, height `int`, data `std::vector<uint8_t>` row-major, intrinsics `CameraIntrinsics`; `valid() → bool`) in `robotlib` namespace
- [ ] `include/common/sensors/depth_frame.hpp` — `DepthFrame` (timestamp `double`, width `int`, height `int`, data `std::vector<float>` row-major metres, intrinsics `CameraIntrinsics`; `valid() → bool`) in `robotlib` namespace
- [ ] `include/common/sensors/force_torque_sensor.hpp` — `ForceTorqueSensor` (timestamp `double`, force `Eigen::Vector3d`, torque `Eigen::Vector3d`) in `robotlib` namespace
- [ ] Tests: `CameraFrame::valid()` on empty data returns false; `DepthFrame::valid()` on zero dimensions returns false; `ImuReading` zero-init construction

---

## MuJoCo Architecture Addition: `common/transforms/` — 3D Conversion Utilities

**Milestone:** M1 (MuJoCo architecture)
**Tracked here** as additions to the existing `common/transforms/` sub-module.

- [ ] `include/transforms/transforms.hpp` additions: `Pose3D ↔ Pose2D` projection (`toSE2()`, `fromSE2()`), `Euler ↔ Quaternion` (`eulerToQuat()`, `quatToEuler()` — ZYX convention), `Transform3D::toSE2()` dropping z/roll/pitch
- [ ] Tests: `Pose3D → Pose2D` drops z and extracts yaw correctly; `eulerToQuat / quatToEuler` round-trip within 1e-9; `fromSE2` sets z=0 and identity roll/pitch
