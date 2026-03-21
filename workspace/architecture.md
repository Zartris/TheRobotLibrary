# System Architecture

> **For AI agents:** Read this document in full before writing any code in this repository.
> It defines the component boundaries, naming conventions, and dependency rules that all
> code must follow. Do not deviate from these without updating this document first.

---

## Overview

TheRobotLibrary is built around two tiers combined into a single executable:

1. **Robotics modules** (`workspace/robotics/`) — Self-contained C++ libraries implementing
   individual robotics domains. They depend only on Eigen and `common` — never on MuJoCo,
   simulation, or any rendering library. Copy any module to a real robot project and it
   just works.

2. **Simulation** (`workspace/simulation/`) — A single C++ executable combining MuJoCo
   physics, 3D rendering (GLFW + MuJoCo renderer), and ImGui control panels. It links
   against the robotics modules and wires them into the MuJoCo physics loop via a bridge
   layer. There is no network API — all interaction happens through the integrated UI.

```
+------------------------------------------------------------------+
|               Robotics Modules (workspace/robotics/)              |
|                                                                   |
|  Pure C++20/Eigen libraries. No MuJoCo dependency.               |
|  Copy any module to a real robot project and it just works.       |
|                                                                   |
|  common/  control/  perception/  state_estimation/                |
|  motion_planning/  fleet_management/                              |
+-------------------------------+----------------------------------+
                                | linked directly
+-------------------------------v----------------------------------+
|               Simulation (workspace/simulation/)                  |
|                                                                   |
|  Single executable: MuJoCo physics + GLFW + ImGui UI              |
|                                                                   |
|  +----------+  +-----------+  +----------+  +------------------+  |
|  | MuJoCo   |  | Bridge    |  | GLFW +   |  | ImGui Control    |  |
|  | Physics  |<>| Layer     |<>| MuJoCo   |  | Panels           |  |
|  | (mj_step)|  | (adapter) |  | Render   |  |                  |  |
|  +----------+  +-----------+  +----------+  +------------------+  |
+------------------------------------------------------------------+
```

**Dependency rule (strictly enforced):**
- Robotics modules -> depend on `common`; cross-domain deps prohibited (control must not depend on perception). Intra-domain composition allowed where explicitly designed (e.g., `control/lqg` uses `lqr` + `ekf`). No MuJoCo dependency.
- Simulation -> depends on MuJoCo, robotics modules, ImGui, GLFW

---

## Robotics Modules (`workspace/robotics/`)

### Purpose

Each subdirectory is a standalone C++ library implementing a specific robotics domain.
The design goal is that any module can be copied to a new project and used without any
other part of this repository.

### Standard module layout

Every module follows this identical structure:

```
<module_name>/
+-- CMakeLists.txt          # Produces a static library; no workspace-level deps required
+-- README.md               # What this module does and how to integrate it
+-- include/
|   +-- <module_name>/      # Public headers only (consumers add this to their include path)
+-- src/                    # Implementation files
+-- tests/                  # Unit tests (Catch2)
+-- docs/
    +-- theory.md           # Theory, math, and algorithm explanations for this domain
```

### Module index

Each domain contains sub-modules — copy only what you need.

| Domain              | Sub-module                  | Description                                              |
|---------------------|-----------------------------|----------------------------------------------------------|
| `common`            | *(flat)*                    | `Pose2D`, `Twist`, `Transform2D`, math utils, map types, interfaces |
|                     | `types_3d/`                 | `Pose3D`, `Quaternion`, `Transform3D`, `Twist3D`, `Wrench`, `TerrainPose` |
|                     | `sensors/`                  | `LidarScan`, `ImuReading`, `CameraFrame`, `DepthFrame`, `ForceTorqueSensor` |
|                     | `kinematics/`               | `IKinematicModel`, differential-drive, unicycle, Ackermann, swerve   |
|                     | `logging/`                  | `ILogger`, `SpdlogLogger`, `getLogger()` — observability for all modules |
|                     | `transforms/`               | 2D transforms + 3D conversion utilities (Pose3D<->Pose2D, Euler<->quaternion) |
|                     | `noise_models/`             | `GaussianNoise<T>`, `UniformNoise<T>`, `OutlierInjector<T>` — seeded, reproducible (header-only) |
|                     | `robot/`                    | `VehicleParams`, `MotorParams`, `TireParams`, `WheelConfig` |
|                     | `environment/`              | `TerrainProperties`, `SlipDetector` |
| `control`           | `pid/`                      | Discrete PID with anti-windup and derivative kick fix    |
|                     | `pure_pursuit/`             | Geometric path tracker; adaptive lookahead               |
|                     | `mpc/`                      | Receding-horizon NMPC via acados; diff-drive/Ackermann/swerve       |
|                     | `cbf/`                      | CBF safety filter — decorator wrapping any IController   |
|                     | `adaptive/gain_scheduling/` | Adaptive PID — online gain adjustment from tracking error|
|                     | `adaptive/rls/`             | Recursive Least Squares parameter estimator              |
|                     | `frenet/`                   | Frenet-Serret frame path following controller            |
|                     | `lqr/`                      | Discrete-time infinite-horizon LQR; DARE solver via Eigen |
|                     | `stanley/`                  | Stanley geometric path tracker; heading + speed-norm CTE  |
|                     | `mppi/`                     | Model Predictive Path Integral; N Monte Carlo rollouts, importance-weighted update |
|                     | `feedback_linearization/`   | Input-output linearization for diff-drive (chained form); Lie derivative check |
|                     | `lqg/`                      | LQG controller = LQR feedback + internal EKF observer; output-only measurements |
| `perception`        | `lidar_processing/`         | Scan filtering, DBSCAN segmentation, RANSAC lines        |
|                     | `occupancy_grid/`           | Binary Bayes grid, log-odds update, inflation            |
|                     | `ray_casting/`              | Bresenham / DDA ray traversal, noise injection           |
|                     | `obstacle_detection/`       | Detection pipeline, clustering, Kalman tracking          |
|                     | `feature_extraction/`       | FAST+BRIEF keypoint pipeline, DescriptorMatcher          |
|                     | `visual_odometry/`          | 8-point RANSAC essential matrix, camera-based ego-motion |
|                     | `imu_processing/`           | Complementary filter, bias estimation, IMU pre-integration |
|                     | `place_recognition/`        | Descriptor-based place DB; loop closure detection        |
|                     | `stereo_depth/`             | Block-matching disparity, StereoRectifier, depth map     |
|                     | `depth_camera/`             | RGB-D processing: depth->pointcloud, hole fill, outlier removal, `RgbdCamera` |
|                     | `object_detection_3d/`      | 3D DBSCAN + PCA oriented bounding box; size-based classification (PERSON/CAR/UNKNOWN) |
|                     | `lane_detection/`           | Hough on binary edge image, lane polynomial fitting (degree-2), left/right assignment |
|                     | `semantic_segmentation/`    | `ISemanticSegmenter` + stub + `PluginSemanticSegmenter` (`std::function` DL plugin point) |
| `state_estimation`  | `ekf/`                      | Extended Kalman Filter for diff-drive + range-bearing    |
|                     | `particle_filter/`          | Monte Carlo Localization (MCL / AMCL)                    |
|                     | `ekf_slam/`                 | Augmented-state EKF-SLAM with landmark management        |
|                     | `lidar_slam/`               | ICP/NDT scan matching, pose-graph SLAM                   |
|                     | `visual_slam/`              | Feature-based visual SLAM (ORB features, loop closure)   |
|                     | `ukf/`                      | Unscented Kalman Filter; Merwe scaled sigma points       |
|                     | `pose_graph/`               | Gauss-Newton / LM SE2 pose graph optimizer (loop closure backend) |
|                     | `visual_inertial_odometry/` | Loosely-coupled VIO: IMU pre-integration + VO fusion     |
|                     | `factor_graph/`             | General factor graph: heterogeneous variables + factors, GN + Eigen `SimplicialLDLT` |
| `motion_planning`   | `global_planning/astar/`    | A\* on 2D grid; octile heuristic; weighted A\*           |
|                     | `global_planning/dijkstra/` | Dijkstra; multi-source distance maps                     |
|                     | `global_planning/rrt/`      | RRT and RRT\* for continuous C-space                     |
|                     | `global_planning/prm/`      | Probabilistic Roadmap; multi-query; Dijkstra query phase |
|                     | `global_planning/informed_rrt_star/` | Informed RRT*: prolate hyperspheroid sampling after first solution |
|                     | `global_planning/lattice_planner/`   | Pre-computed motion primitives, state lattice graph, A*/Dijkstra query |
|                     | `local_planning/dwa/`       | Dynamic Window Approach; velocity sampling               |
|                     | `local_planning/potential_field/`    | Attractive + repulsive fields; local minima escape via random perturbation |
|                     | `trajectory_planning/velocity_profiling/` | Trapezoidal / S-curve profiles; curvature-aware speed |
|                     | `trajectory_planning/spline_fitting/`     | Cubic spline, Catmull-Rom, B-spline path smoothing    |
|                     | `trajectory_planning/teb/`               | Timed Elastic Band: joint path + timing optimization  |
|                     | `trajectory_planning/time_optimal/`      | TOPP-RA, minimum-snap QP, bang-bang profiles          |
|                     | `trajectory_planning/polynomial/`        | Min-jerk / min-snap polynomials; Bezier curves (de Casteljau) |
|                     | `multi_robot/orca/`         | VO->RVO->ORCA reactive collision avoidance              |
|                     | `multi_robot/priority_planning/` | Sequential priority-based multi-robot planning   |
|                     | `multi_robot/cbs/`          | Conflict-Based Search (optimal MAPF)                 |
|                     | `multi_robot/dmpc/`         | Distributed MPC with inter-robot trajectory constraints |
|                     | `multi_robot/mader/`        | MADER: decentralized async replanning (MIT ACL)       |
| `fleet_management`  | `vda5050/`                  | VDA 5050 v2.0 message types + nlohmann/json serialization|
|                     | `task_allocation/`          | `ITaskAllocator` + greedy/auction allocators             |
|                     | `fleet_monitor/`            | `IFleetMonitor` — per-robot state aggregation            |
|                     | `battery_management/`       | `IBatteryManager` — charge tracking + route-to-charger   |
|                     | `charging_station/`         | `ChargingStationManager` — station registry, priority charge queue, linear completion prediction |

### Dependency graph between modules

```
common  <-  control
common  <-  perception
common  <-  state_estimation
common  <-  motion_planning
common  <-  fleet_management
```

No module other than `common` may depend on another domain module. If cross-domain shared
data is needed, the type belongs in `common`.

---

## Type System (`common/`)

### SE2 Types (kept, for 2D algorithms)

- `Pose2D` — (x, y, theta)
- `Transform2D` — SE2 rigid body transform
- `Twist` — (linear, angular) velocity

These remain the primary types for 2D algorithms (path planning, 2D controllers, 2D SLAM).
Ground robots use 2D kinematics even in the 3D world.

### SE3 Types (new, in `common/types_3d/`)

- `Pose3D` — position (x, y, z) + orientation (quaternion)
- `Transform3D` — SE3 rigid body transform (rotation matrix + translation)
- `Quaternion` — unit quaternion with multiplication, slerp, Euler/rotation-matrix conversion
- `Twist3D` — 6-DOF velocity (linear xyz + angular xyz)
- `Wrench` — 6-axis force + torque vector

### 2.5D Bridge Types (for ground robots on slopes)

- `TerrainPose` — `Pose2D` + elevation + pitch + roll
- Conversion utilities: `Pose3D` <-> `Pose2D` projection, `Pose3D` -> `TerrainPose`

### Sensor Data Types (in `common/sensors/`)

Hardware-agnostic sensor data types — the same types work with MuJoCo simulation or real
hardware drivers:

- `LidarScan` — array of ranges + angles (2D) or point cloud (3D)
- `ImuReading` — accelerometer + gyroscope + timestamp
- `CameraFrame` — RGB image buffer (`std::vector<uint8_t>`) + width/height/channels + intrinsics (`Eigen::Matrix3d`) + extrinsics (`Transform3D`)
- `DepthFrame` — depth buffer (`std::vector<float>`) + width/height + intrinsics
- `ForceTorqueSensor` — 6-axis wrench reading

### Sub-Module Boundaries

- **`common/types_3d/`** owns all 3D type definitions: Pose3D, Quaternion, Transform3D, Twist3D, Wrench, TerrainPose
- **`common/transforms/`** owns conversion functions between types: 2D<->3D projections, Euler<->quaternion, rotation matrix conversions
- **`common/sensors/`** owns sensor data type definitions
- The existing 2D types remain in their current locations

All types live in the `robotlib` namespace and are Eigen-based. No MuJoCo includes anywhere
in `workspace/robotics/`.

---

## Simulation (`workspace/simulation/`)

### Purpose

The simulation is a single C++ executable that:
- Loads MJCF (XML) model files defining robots, terrain, sensors, and world geometry
- Runs MuJoCo physics with a configurable timestep (default 2ms / 500Hz)
- Renders the 3D scene via MuJoCo's built-in renderer
- Provides ImGui overlay panels for simulation control, module switching, and telemetry
- Can run headless (no window) for batch testing and CI

### MuJoCo Physics Core

- **MJCF loading:** Robot and world parameters are defined in MJCF files. The `ModelAdapter`
  extracts what robotics modules need (`VehicleParams`, sensor configurations) from the
  loaded `mjModel` at startup.
- **Split-step loop:** `mj_step1()` -> module control injection -> `mj_step2()`. This
  allows the robotics pipeline to read sensor data and write control signals between the
  two halves of the physics step.
- **Configurable timestep:** Default 2ms (500Hz). Adjustable for accuracy vs. speed tradeoffs.
- **Headless mode:** Physics runs without a window for CI and batch testing.

### Bridge Layer (`simulation/bridge/`)

The bridge is the **only** code in the repository that includes `<mujoco/mujoco.h>`:

- **SensorAdapter** — reads `mjData` sensor arrays -> `common::LidarScan`, `ImuReading`, `CameraFrame`, etc.
- **StateAdapter** — reads `mjData` qpos/qvel -> `common::Pose3D`, `Pose2D`, `TerrainPose`
- **ActuatorAdapter** — takes `common::Twist` from controllers -> writes `mjData::ctrl[]`
- **ModelAdapter** — reads `mjModel` at startup -> populates `VehicleParams`, `MotorParams`, sensor configs

For real hardware deployment (no MuJoCo), a separate hardware config loader would populate
the same `common/` structs. Modules do not care where params came from.

### Module Pipeline

Pluggable robotics modules wired together at runtime:

```
Sensors (via SensorAdapter)
  -> IStateEstimator (EKF, particle filter, ...)
  -> IGlobalPlanner (A*, RRT, ...)
  -> ILocalPlanner (DWA, potential field, ...)
  -> IController (PID, pure_pursuit, MPC, ...)
  -> ActuatorAdapter -> MuJoCo ctrl[]
```

- Each slot has an interface defined in `common/` or its domain
- Runtime switching via ImGui dropdown panels
- Pipeline reads sensor data from bridge, computes commands, feeds back through bridge

### App Shell (GLFW + MuJoCo Render + ImGui)

Single executable combining:

- **GLFW** window creation + OpenGL context
- **MuJoCo** `mjr_render()` for 3D scene in the main viewport
- **ImGui** overlay panels (docking branch, GLFW + OpenGL3 backend):
  - Simulation control (play/pause/step/reset/speed)
  - Module pipeline selector (dropdowns for controller, planner, estimator, kinematics)
  - Parameter tuning (PID gains, lookahead distance, etc.)
  - Telemetry plots (velocity, tracking error, sensor data)
  - Scenario loader (load different MJCF worlds)

### Threading Model

```
Physics Thread                    Main Thread (OpenGL)
+--------------------------+      +-------------------------+
| loop:                    |      | loop:                   |
|   mj_step1(m, d)        |      |   lock(mtx)             |
|   bridge.readSensors(d)  |      |   mj_copyData(d_render, |
|   pipeline.run()         |      |                m, d)    |
|   bridge.writeCtrl(d)    |      |   unlock(mtx)           |
|   mj_step2(m, d)        |      |   mjr_render(d_render)  |
|   lock(mtx)              |      |   ImGui panels          |
|   notify render          |      |   glfwSwapBuffers()     |
|   unlock(mtx)            |      +-------------------------+
+--------------------------+
```

Key design choices:
- **Module pipeline runs on the physics thread** between `mj_step1()` and `mj_step2()`.
  Fast modules (PID, EKF, DWA) complete well within the 2ms budget.
- **Slow planners (RRT\*, MPC, A\*) run asynchronously**: launched on a separate planning
  thread, producing results consumed on the next tick via a thread-safe buffer. The
  controller uses the last-known-good plan until a new one arrives.
- **Render thread gets a copy** of `mjData` via `mj_copyData()` under a mutex. This is the
  standard MuJoCo pattern — `mjData` is not thread-safe, so the render thread never touches
  the physics `mjData` directly.
- **ImGui state changes** (module switching, parameter tuning) are queued and applied on the
  physics thread at the start of the next tick.

### Scenario System

- MJCF files live in `simulation/scenarios/`
- Each scenario defines: terrain + robot(s) + obstacles + sensor configuration
- Loaded at startup via command line or switched at runtime via ImGui
- Multi-robot scenarios are native (multiple bodies in one MJCF)

### Simulation directory layout

```
simulation/
+-- CMakeLists.txt              # Single executable, links MuJoCo + ImGui + modules
+-- include/simulation/
|   +-- bridge/                 # SensorAdapter, StateAdapter, ActuatorAdapter, ModelAdapter
|   +-- pipeline/               # Module pipeline wiring + runtime switching
|   +-- app/                    # GLFW window, ImGui panels, render loop
+-- src/
|   +-- main.cpp                # Entry point: load MJCF, create window, run
|   +-- bridge/                 # Bridge implementations
|   +-- pipeline/               # Pipeline implementations
|   +-- app/                    # App shell, ImGui panels, render
|   +-- scenario_loader/        # Scenario loading logic
+-- scenarios/                  # MJCF model files
+-- tests/                      # Integration tests (headless MuJoCo)
+-- docs/
    +-- design.md
```

---

## C++ Conventions

- **Standard:** C++20 minimum. Use standard library features aggressively (`std::ranges`,
  `std::span`, `std::optional`, `std::expected`). Use C++ modules where toolchain support
  is stable; otherwise use `#pragma once` header guards.
- **Build system:** CMake 3.20+. Each module has its own standalone `CMakeLists.txt`.
  A root `workspace/CMakeLists.txt` wires all submodules together.
- **Naming:**
  - Files and directories: `snake_case`
  - Types (classes, structs, enums): `PascalCase`
  - Methods and functions: `camelCase`
  - Member variables: `m_camelCase`
  - Constants and macros: `UPPER_SNAKE_CASE`
- **Memory management:** No raw owning pointers. Use `std::unique_ptr`, `std::shared_ptr`,
  or value types. Prefer value semantics where possible.
- **Error handling:** Use `std::expected<T, E>` for recoverable errors in library code.
  Reserve exceptions for programmer errors (precondition violations).
- **Testing:** Every module has a `tests/` directory. Tests use **Catch2**.
  New code should not decrease coverage.

---

## Adding a New Robotics Module

1. Create `workspace/robotics/<module_name>/` following the standard layout above.
2. Write `CMakeLists.txt` producing a static library named `<module_name>`.
3. Write `README.md` explaining what the module provides and how to integrate it.
4. Write `docs/theory.md` covering the theory, math, and algorithms involved.
5. Add `add_subdirectory(robotics/<module_name>)` to `workspace/CMakeLists.txt`.
6. Only depend on `common`. Document any external third-party dependencies in `CMakeLists.txt`
   with a comment explaining why they are needed.

---

## Directory Structure Reference

```
workspace/
+-- architecture.md            <- This file (read before writing code)
+-- CMakeLists.txt             <- Root build: wires all submodules together
+-- robotics/
|   +-- README.md
|   +-- common/                <- Shared types and math primitives
|   |   +-- types_3d/          <- Pose3D, Transform3D, Quaternion, Twist3D, Wrench, TerrainPose
|   |   +-- sensors/           <- LidarScan, ImuReading, CameraFrame, DepthFrame, ForceTorqueSensor
|   |   +-- kinematics/        <- IKinematicModel, differential-drive, unicycle, Ackermann, swerve
|   |   +-- robot/             <- VehicleParams, MotorParams, TireParams, WheelConfig
|   |   +-- environment/       <- TerrainProperties, SlipDetector
|   |   +-- logging/           <- ILogger, SpdlogLogger
|   |   +-- transforms/        <- 2D transforms + 3D conversion utilities
|   |   +-- noise_models/      <- GaussianNoise, UniformNoise, OutlierInjector
|   +-- control/
|   |   +-- pid/
|   |   +-- pure_pursuit/
|   |   +-- mpc/
|   |   +-- cbf/
|   |   +-- adaptive/
|   |   +-- frenet/
|   |   +-- lqr/
|   |   +-- stanley/
|   |   +-- mppi/
|   |   +-- feedback_linearization/
|   |   +-- lqg/
|   +-- perception/
|   |   +-- lidar_processing/
|   |   +-- occupancy_grid/
|   |   +-- ray_casting/
|   |   +-- obstacle_detection/
|   |   +-- feature_extraction/
|   |   +-- visual_odometry/
|   |   +-- imu_processing/
|   |   +-- place_recognition/
|   |   +-- stereo_depth/
|   |   +-- depth_camera/
|   |   +-- object_detection_3d/
|   |   +-- lane_detection/
|   |   +-- semantic_segmentation/
|   +-- state_estimation/
|   |   +-- ekf/
|   |   +-- particle_filter/
|   |   +-- ekf_slam/
|   |   +-- lidar_slam/
|   |   +-- visual_slam/
|   |   +-- ukf/
|   |   +-- pose_graph/
|   |   +-- visual_inertial_odometry/
|   |   +-- factor_graph/
|   +-- motion_planning/
|   |   +-- global_planning/
|   |   |   +-- astar/
|   |   |   +-- dijkstra/
|   |   |   +-- rrt/
|   |   |   +-- prm/
|   |   |   +-- informed_rrt_star/
|   |   |   +-- lattice_planner/
|   |   +-- local_planning/
|   |   |   +-- dwa/
|   |   |   +-- potential_field/
|   |   +-- trajectory_planning/
|   |   |   +-- velocity_profiling/
|   |   |   +-- spline_fitting/
|   |   |   +-- teb/
|   |   |   +-- time_optimal/
|   |   |   +-- polynomial/
|   |   +-- multi_robot/
|   |       +-- orca/
|   |       +-- priority_planning/
|   |       +-- cbs/
|   |       +-- dmpc/
|   |       +-- mader/
|   +-- fleet_management/
|       +-- vda5050/
|       +-- task_allocation/
|       +-- fleet_monitor/
|       +-- battery_management/
|       +-- charging_station/
+-- simulation/                <- MuJoCo simulation + integrated visualization
|   +-- CMakeLists.txt
|   +-- include/simulation/
|   |   +-- bridge/
|   |   +-- pipeline/
|   |   +-- app/
|   +-- src/
|   |   +-- main.cpp
|   |   +-- bridge/
|   |   +-- pipeline/
|   |   +-- app/
|   |   +-- scenario_loader/
|   +-- scenarios/             <- MJCF model files
|   +-- tests/
|   +-- docs/
+-- cmake/
    +-- deps.cmake             <- FetchContent: MuJoCo, Eigen, Catch2, spdlog, ImGui, etc.
```

---

## Building

### Prerequisites

- CMake >= 3.20
- A C++20-capable compiler: GCC 12+, Clang 15+, or MSVC 2022+
- Git (FetchContent downloads deps on first configure)
- OpenGL development headers (`libgl1-mesa-dev` on Debian/Ubuntu)
- For `trajectory_planning/teb` (when implementing): `libsuitesparse-dev`

MuJoCo and GLFW are both fetched via FetchContent in `deps.cmake`. GLFW is fetched explicitly because `MUJOCO_BUILD_SIMULATE=OFF` prevents MuJoCo from providing it.

### Full workspace build

```bash
# Configure (first run downloads all FetchContent dependencies -- allow a few minutes)
cmake -B build -S workspace -DCMAKE_BUILD_TYPE=Release

# Build everything
cmake --build build -j$(nproc)

# Run all tests
ctest --test-dir build --output-on-failure
```

### Debug build

```bash
cmake -B build-debug -S workspace -DCMAKE_BUILD_TYPE=Debug
cmake --build build-debug -j$(nproc)
ctest --test-dir build-debug --output-on-failure
```

### Build a single module

Each leaf module is a standalone CMake project. To build just `pid`:

```bash
cmake -B build-pid -S workspace/robotics/control/pid \
      -DCMAKE_BUILD_TYPE=Release
cmake --build build-pid -j$(nproc)
```

> **Note:** Standalone builds require `common` (and any other direct dependencies) to also
> be built and their install prefix added to `CMAKE_PREFIX_PATH`, or the workspace root
> must be used instead.

### IDE / clangd support

`compile_commands.json` is generated by default.
Point clangd at the build directory:

```bash
# In VS Code: set "clangd.arguments": ["--compile-commands-dir=build"]
# Or symlink at workspace root:
ln -s build/compile_commands.json workspace/compile_commands.json
```

### Third-party dependency management

All dependencies are fetched via CMake's `FetchContent` in `workspace/cmake/deps.cmake`.
No vcpkg manifest, Conan recipe, or system packages are required (except OpenGL headers).
To add a new dependency:

1. Add a `FetchContent_Declare` block in `cmake/deps.cmake`, guarded with
   `if(NOT TARGET <target-name>)`.
2. Reference the target in the relevant module's `CMakeLists.txt`.
3. Update this document and the module's `README.md`.
