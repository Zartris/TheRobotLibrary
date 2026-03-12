# M1 — Minimum Viable Robot

**Status:** Not Started  
**Dependencies:** M0  
**Scope:** The simplest working version of EVERY layer needed for an end-to-end autonomous robot navigating from A to B in a 2D room.

---

## Goal

A robot navigates from start to goal in a 2D room: perceiving via lidar, localizing via EKF, planning via A* + DWA, controlled via PID. All visible in an ImGui frontend. This is the **foundation** — every future milestone upgrades one module at a time, testable because the full loop already works.

---

## What's IN

| Domain | Module | Scope |
|--------|--------|-------|
| Common | `common` | Pose2D, Twist, Transform2D, OccupancyGrid, LaserScan, Path, geometry utils, kinematics models, map types, swappable interfaces |
| Perception | `ray_casting` | Bresenham ray cast on OccupancyGrid |
| Perception | `occupancy_grid` | Binary grid with log-odds update from laser scans |
| Perception | `lidar_processing` | Range clipping + optional median filter |
| State Est. | `ekf` | 2D EKF: odometry prediction + landmark update |
| Control | `pid` | Discrete PID with anti-windup for heading/speed |
| Planning | `astar` | A* on occupancy grid, octile heuristic, 8-connected |
| Planning | `dwa` | Basic DWA: velocity sampling, trajectory rollout, obstacle/goal scoring |
| Planning | `velocity_profiling` | Trapezoidal velocity profile along a path |
| Simulation | `simulation` | World model, **pluggable kinematics** (diff-drive default), simulated lidar, sim loop, Crow WS+REST, scenario loader |
| Frontend | `native` | SDL2+ImGui, WS state receiver, REST commander, 2D renderer |

## What's NOT IN

pure_pursuit, mpc, particle_filter, SLAM (ekf_slam, lidar_slam), dijkstra, rrt, spline_fitting, teb, time_optimal, multi-robot (orca, priority_planning, cbs, dmpc, mader), web frontend, obstacle_detection, RANSAC, inflation layer. No performance optimization. No advanced error handling.

---

## Sub-Phases

M1 is too large for a single pass. It's split into ordered sub-phases, each independently testable:

```
M1-A: Common Foundation
 └→ M1-B: Ray Casting
 └→ M1-C: Occupancy Grid  (uses ray_casting for cell marking)
 └→ M1-D: Lidar Processing
 └→ M1-E: EKF
 └→ M1-F: PID Controller
 └→ M1-G: A* Planner
 └→ M1-H: DWA Local Planner
 └→ M1-I: Velocity Profiling
 └→ M1-J: Simulation Server  (uses ray_casting, links all modules)
 └→ M1-K: Full Pipeline Integration
 └→ M1-L: Native Frontend
```

**Ordering rules:**
- M1-A must come first (everything depends on common)
- **Group 1** (parallel, depend only on common): M1-B, M1-D, M1-E, M1-F, M1-G, M1-H, M1-I
- **Group 2** (needs M1-B): M1-C (occupancy_grid uses ray_casting for cell marking)
- M1-J needs M1-B at minimum (simulated lidar uses ray_casting)
- M1-K wires everything together (needs all of M1-A through M1-J)
- M1-L is last (needs running sim server)

---

### M1-A: Common Foundation

Core types, math utilities, and abstract interfaces.

- [ ] `include/common/types.hpp` — `Pose2D`, `Point2D`, `Twist`, `Path` (vector of Pose2D)
- [ ] `include/common/transform2d.hpp` — `Transform2D` (compose, inverse, transform point)
- [ ] `include/common/occupancy_grid.hpp` — `OccupancyGrid` data type (width, height, resolution, origin, cell data)
- [ ] `include/common/laser_scan.hpp` — `LaserScan` (angle_min, angle_max, angle_increment, ranges, intensities)
- [ ] `include/common/geometry.hpp` — `angleWrap`, `distance`, `normalizeAngle`, `lerp`, `pointInPolygon`
- [ ] `include/common/interfaces/i_controller.hpp` — `IController`
- [ ] `include/common/interfaces/i_global_planner.hpp` — `IGlobalPlanner`
- [ ] `include/common/interfaces/i_local_planner.hpp` — `ILocalPlanner`
- [ ] `include/common/interfaces/i_state_estimator.hpp` — `IStateEstimator`
- [ ] `include/common/interfaces/i_velocity_profiler.hpp` — `IVelocityProfiler`
- [ ] `include/common/interfaces/i_slam_estimator.hpp` — `ISlamEstimator` extends `IStateEstimator` with `getMap()`/`getLandmarks()` (unused until M6, but frozen in M2)
- [ ] `include/common/kinematics/i_kinematic_model.hpp` — `IKinematicModel`: `step(state, control, dt)`, `getControlLimits()`, `toTwist(control)`, `fromTwist(twist)`
- [ ] `include/common/kinematics/differential_drive.hpp` — `DifferentialDrive : IKinematicModel` (wheel radius, track width)
- [ ] `src/kinematics/differential_drive.cpp`
- [ ] `tests/test_differential_drive.cpp` — known twist → expected pose, wheel speeds ↔ twist round-trip
- [ ] `src/transform2d.cpp`, `src/geometry.cpp`
- [ ] `tests/test_types.cpp` — construction, equality, basic ops
- [ ] `tests/test_transform2d.cpp` — compose, inverse, transform point
- [ ] `tests/test_geometry.cpp` — angle_wrap edge cases (π, -π, >2π), distance, normalize

**Interfaces** (defined in common, implemented by each domain module):

```cpp
// IKinematicModel: step(state, control, dt) → new state; toTwist/fromTwist conversions
// IController: current pose + target → Twist command
// IGlobalPlanner: start + goal + grid → optional<Path>
// ILocalPlanner: pose + velocity + path + scan + grid → Twist
// IStateEstimator: predict(twist, dt), update(sensor_data), getPose(), getCovariance()
// IVelocityProfiler: path + constraints → TimedPath
```

> **Map types** (`OccupancyGrid`, and later `PointCloudMap`, `FeatureMap`) live in common.
> Modules that need a map import from `common/` — no module defines its own map type.

**Exit:** All common tests pass. Interfaces compile.

---

### M1-B: Ray Casting

Bresenham ray traversal on OccupancyGrid.

- [ ] `include/ray_casting/ray_caster.hpp` — `castRay()`, `castScan()`
- [ ] `src/ray_caster.cpp`
- [ ] `tests/test_ray_caster.cpp`:
  - Ray into empty grid → max_range, no hit
  - Ray toward wall at known distance → correct range ± 1 cell
  - Diagonal ray at 45° → expected hit
  - `castScan` with 3 rays → correct LaserScan structure

**Exit:** All ray_casting tests pass.

---

### M1-C: Occupancy Grid Module

Log-odds grid updated from laser scans via inverse sensor model.

- [ ] `include/occupancy_grid/occupancy_grid_map.hpp` — `OccupancyGridMap` class
- [ ] `src/occupancy_grid_map.cpp`
- [ ] `tests/test_occupancy_grid_map.cpp`:
  - Empty grid → all cells Unknown
  - Single ray update → cells along ray Free, endpoint Occupied
  - Multiple updates → log-odds accumulate
  - Coordinate conversion round-trip: `toGrid(toWorld(c)) == c`

**Exit:** All occupancy_grid tests pass.

---

### M1-D: Lidar Processing

Basic scan filtering.

- [ ] `include/lidar_processing/scan_filter.hpp` — `FilterConfig`, `filterScan()`
- [ ] `src/scan_filter.cpp`
- [ ] `tests/test_scan_filter.cpp`:
  - Range clipping (below min, above max → NaN)
  - Median filter with window=3 → known output
  - Passthrough on clean scan → identical
  - All-NaN input → all-NaN output

**Exit:** All lidar_processing tests pass.

---

### M1-E: EKF

2D Extended Kalman Filter. State = [x, y, θ].

- [ ] `include/ekf/ekf2d.hpp` — `EKF2D : IStateEstimator`
- [ ] `src/ekf2d.cpp`
- [ ] `tests/test_ekf2d.cpp`:
  - Predict with zero twist → pose unchanged, covariance grows
  - Predict with known twist for 1s → expected pose
  - Update with perfect measurement → covariance shrinks
  - Predict→update cycle converges to true pose with noisy inputs

**Exit:** All ekf tests pass.

---

### M1-F: PID Controller

Discrete PID with anti-windup. Wraps two PIDs (heading + speed) into `IController`.

- [ ] `include/pid/pid_controller.hpp` — `PIDController`, `HeadingSpeedController : IController`
- [ ] `src/pid_controller.cpp`
- [ ] `tests/test_pid_controller.cpp`:
  - P-only: output proportional to error
  - PI: steady-state error → zero
  - PID: derivative damps overshoot
  - Anti-windup: integral doesn't explode under saturation
  - Reset clears state

**Exit:** All pid tests pass.

---

### M1-G: A* Planner

A* grid search. 8-connected, octile heuristic.

- [ ] `include/astar/astar_planner.hpp` — `AStarPlanner : IGlobalPlanner`
- [ ] `src/astar_planner.cpp`
- [ ] `tests/test_astar_planner.cpp`:
  - Straight line in empty grid → direct path
  - L-shaped obstacle → path goes around
  - No path possible → `std::nullopt`
  - Start == goal → single-point path
  - Start/goal in occupied cell → `std::nullopt`

**Exit:** All astar tests pass.

---

### M1-H: DWA Local Planner

Dynamic Window Approach: velocity sampling, trajectory rollout, scoring.

- [ ] `include/dwa/dwa_planner.hpp` — `DWAPlanner : ILocalPlanner`
- [ ] `src/dwa_planner.cpp`
- [ ] `tests/test_dwa_planner.cpp`:
  - Open space with goal ahead → forward velocity
  - Obstacle ahead → turning velocity
  - Boxed in → zero twist
  - Respects dynamic window (acceleration limits)

**Exit:** All dwa tests pass.

---

### M1-I: Velocity Profiling

Trapezoidal velocity profile along a path.

- [ ] `include/velocity_profiling/trapezoidal_profiler.hpp` — `TrapezoidalProfiler : IVelocityProfiler`
- [ ] `src/trapezoidal_profiler.cpp`
- [ ] `tests/test_trapezoidal_profiler.cpp`:
  - Long straight path → accelerate-cruise-decelerate
  - Short path → triangular profile (no cruise)
  - Single-point path → zero velocity
  - Sharp turns → velocity reduced

**Exit:** All velocity_profiling tests pass.

---

### M1-J: Simulation Server

World model, diff-drive kinematics, simulated lidar, Crow WS+REST, scenario loader.

- [ ] `include/simulation/world.hpp` — `WorldModel` (grid map, robot state, landmarks)
- [ ] `include/simulation/robot.hpp` — `Robot` (pose, velocity, uses `IKinematicModel` for `step()`)
- [ ] `include/simulation/sim_loop.hpp` — `SimLoop` (fixed-timestep tick)
- [ ] `include/simulation/api_server.hpp` — Crow routes + WS broadcast
- [ ] `include/simulation/scenario_loader.hpp` — JSON → world config
- [ ] `src/main.cpp`, `src/world.cpp`, `src/sim_loop.cpp`, `src/robot.cpp`, `src/api_server.cpp`, `src/scenario_loader.cpp`
- [ ] At least 1 scenario JSON: simple room with walls + start/goal + landmarks
- [ ] `tests/test_robot.cpp` — kinematic model step (apply twist via `DifferentialDrive` → expected pose)
- [ ] `tests/test_world.cpp` — load scenario, step, state consistency
- [ ] `tests/test_scenario_loader.cpp` — parse JSON → valid config

**Exit:** Sim starts, streams state via WS, accepts REST commands. Unit tests pass.

Smoke test:
```bash
./build/simulation_server --scenario default
curl http://localhost:8080/api/scenario/list
curl -X POST http://localhost:8080/api/sim/start
# Verify WS state stream
```

---

### M1-K: Full Pipeline Integration

Wire all modules into the sim loop. The robot runs autonomously.

- [ ] `include/simulation/robot_pipeline.hpp` — `RobotPipeline` struct with `unique_ptr<IFoo>` for each layer
- [ ] Update `sim_loop.cpp` — autonomous tick: sense → filter → estimate → plan → profile → local plan → control → actuate
- [ ] Update `simulation/CMakeLists.txt` — link all robotics modules
- [ ] REST endpoints for module swapping: `PUT /api/robot/controller`, `/global_planner`, `/local_planner`, `/estimator`
- [ ] `GET /api/robot/pipeline` — returns current module selections
- [ ] `tests/test_pipeline_integration.cpp`:
  - Load "simple_room" → start sim → robot reaches goal within 60 simulated seconds
  - Swap controller via REST → sim continues without crash

**Exit:** Robot autonomously navigates from A to B. Integration test passes.

---

### M1-L: Native Frontend

SDL2 + ImGui desktop app connected to sim via WS + REST.

- [ ] `src/main.cpp` — SDL2+ImGui init, main loop
- [ ] `src/ws_client.cpp` — IXWebSocket state receiver, JSON parsing
- [ ] `src/rest_client.cpp` — cpp-httplib command sender
- [ ] `src/renderer.cpp` — ImDrawList 2D rendering
- [ ] Rendering layers:
  - Grid map (free=white, occupied=black, unknown=gray)
  - Robot (triangle showing heading)
  - Lidar rays (light red)
  - Global planned path (green line)
  - DWA trajectory candidates (thin blue arcs)
  - EKF covariance ellipse (yellow)
- [ ] UI panels: sim control buttons, robot state text, scenario dropdown, module selector dropdowns
- [ ] Tests: WS JSON parsing, REST command serialization (unit-testable data layer)

**Exit:** Frontend connects to sim, displays all visual elements, controls work. Manual visual verification.

---

## Overall M1 Exit Criteria

1. Robot autonomously navigates from start to goal in a 2D room scenario
2. Lidar data generated by ray_casting, filtered by lidar_processing
3. Occupancy grid updated incrementally from lidar scans
4. A* plans global path on the grid
5. DWA generates local collision-free velocity commands
6. PID tracks velocity commands
7. EKF estimates pose from odometry + simulated landmarks
8. All visible in ImGui frontend: grid, robot, lidar, path, EKF ellipse
9. All unit tests pass, CI green
10. Module swapping via REST works (controller at minimum)
