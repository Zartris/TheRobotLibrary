# Design Spec: 3D MuJoCo Architecture Refactor

**Date:** 2026-03-20
**Status:** Draft
**Scope:** Full architecture redesign — 2D custom sim to 3D MuJoCo-based simulation

---

## 1. Motivation

TheRobotLibrary was originally designed as a 2D simulation with a three-tier architecture: robotics modules, a Crow HTTP/WebSocket simulation server, and separate native (ImGui) and web (React) frontends connected over a network API.

This redesign replaces that with a simpler, more capable architecture:

- **MuJoCo** as the physics backend — fast, accurate 3D rigid-body dynamics with contact, friction, terrain, and sensors out of the box.
- **Single executable** combining MuJoCo physics, 3D rendering, and ImGui control panels — no network layer, no separate frontends.
- **3D from the start** — ground robots on slopes with terrain interaction, architected for future expansion to drones, manipulators, and legged robots.
- **Robotics modules remain pure C++/Eigen** — no MuJoCo dependency in any module. The simulation bridge is the only code that touches MuJoCo.

---

## 2. Architecture Overview

### 2.1 Two-Tier Structure

```
+-----------------------------------------------------------------+
|              Robotics Modules (workspace/robotics/)              |
|                                                                  |
|  Pure C++20/Eigen libraries. No MuJoCo dependency.              |
|  Copy any module to a real robot project and it just works.      |
|                                                                  |
|  common/  control/  perception/  state_estimation/               |
|  motion_planning/  fleet_management/                             |
+-------------------------------+---------------------------------+
                                | linked directly
+-------------------------------v---------------------------------+
|              Simulation (workspace/simulation/)                  |
|                                                                  |
|  Single executable: MuJoCo physics + GLFW + ImGui UI            |
|                                                                  |
|  +----------+  +-----------+  +----------+  +----------------+  |
|  | MuJoCo   |  | Bridge    |  | GLFW +   |  | ImGui Control  |  |
|  | Physics  |<>| Layer     |<>| MuJoCo   |  | Panels         |  |
|  | (mj_step)|  | (adapter) |  | Render   |  |                |  |
|  +----------+  +-----------+  +----------+  +----------------+  |
+-----------------------------------------------------------------+
```

### 2.2 Dependency Rule

- **Robotics modules** -> depend only on `common` (Eigen-based, no MuJoCo)
- **Simulation** -> depends on MuJoCo, robotics modules, ImGui, GLFW

### 2.3 What Was Removed

- **Frontends tier** (`workspace/frontends/`) -- visualization is integrated into the simulation executable
- **Crow** HTTP/WebSocket server -- no network API
- **SDL2** -- replaced by GLFW (MuJoCo's native windowing)
- **IXWebSocket, cpp-httplib** -- no network layer needed

---

## 3. `common/` Type System

### 3.1 Coordinate Levels

The type system supports three coordinate levels for different use cases:

**SE2 types (kept, for 2D algorithms):**
- `Pose2D` -- (x, y, theta)
- `Transform2D` -- SE2 rigid body transform
- `Twist` -- (linear, angular) velocity

**SE3 types (new, for 3D and as canonical simulation type):**
- `Pose3D` -- position (x, y, z) + orientation (quaternion)
- `Transform3D` -- SE3 rigid body transform (rotation matrix + translation)
- `Quaternion` -- unit quaternion with multiplication, slerp, Euler/rotation-matrix conversion
- `Twist3D` -- 6-DOF velocity (linear xyz + angular xyz)
- `Wrench` -- 6-axis force + torque vector

All types live in the `robotlib` namespace to avoid collisions with other robotics libraries (Pinocchio, RBDL, etc.) that use similar names.

**2.5D bridge types (for ground robots on slopes):**
- `TerrainPose` -- `Pose2D` + elevation + pitch + roll
- Conversion utilities: `Pose3D` <-> `Pose2D` projection, `Pose3D` -> `TerrainPose`

### 3.1.1 Sub-Module Boundaries

- **`common/types_3d/`** owns all 3D type definitions: Pose3D, Quaternion, Transform3D, Twist3D, Wrench, TerrainPose
- **`common/transforms/`** owns conversion functions between types: 2D<->3D projections, Euler<->quaternion, rotation matrix conversions
- The existing 2D types remain in their current locations

### 3.2 Sensor Data Types

New sub-module `common/sensors/` defining hardware-agnostic sensor data:

- `LidarScan` -- array of ranges + angles (2D) or point cloud (3D)
- `ImuReading` -- accelerometer + gyroscope + timestamp
- `CameraFrame` -- RGB image buffer (`std::vector<uint8_t>`) + width/height/channels + intrinsics (`Eigen::Matrix3d`) + extrinsics (`Transform3D`). Raw buffer (not Eigen matrix) because image data is row-major and byte-typed, which doesn't map cleanly to Eigen's column-major layout.
- `DepthFrame` -- depth buffer (`std::vector<float>`) + width/height + intrinsics
- `ForceTorqueSensor` -- 6-axis wrench reading

`CameraFrame` and `DepthFrame` are the raw sensor data types in `common/sensors/`. The `perception/depth_camera/RgbdCamera` module consumes these types and provides higher-level processing (pointcloud conversion, hole filling, outlier removal).

All types are header-only with no MuJoCo includes. The simulation bridge converts `mjData` fields into these types. MuJoCo's offscreen renderer produces bottom-up RGB framebuffers which the bridge flips to top-down before populating `CameraFrame`. The same types would be populated from real hardware drivers on a physical robot.

### 3.3 Existing Sub-Modules (unchanged)

- `kinematics/` -- IKinematicModel, IDynamicModel. Note: current interfaces are 2D (Pose2D, Twist). 3D kinematic interfaces (for drones, manipulators) will be addressed in a future spec when those modules are designed. For now, the 2D interfaces remain — ground robots use 2D kinematics even in the 3D world.
- `robot/` -- VehicleParams, MotorParams, TireParams, WheelConfig. May need expansion as MJCF models grow in complexity (joint limits, actuator dynamics, tendon properties). The `ModelAdapter` extracts only what current `common/robot/` types support; new types are added as modules require them.
- `environment/` -- TerrainProperties, SlipDetector
- `logging/` -- ILogger, SpdlogLogger
- `transforms/` -- existing + 3D conversion utilities
- `noise_models/` -- GaussianNoise, UniformNoise, OutlierInjector

---

## 4. Simulation Tier

### 4.1 MuJoCo Physics Core

- Loads MJCF (XML) model files defining robots, terrain, sensors, and world
- Runs split-step loop: `mj_step1()` -> module control injection -> `mj_step2()`
- Configurable timestep (default 2ms / 500Hz)
- Can run headless (no window) for batch testing and CI
- Physics runs on a dedicated thread, decoupled from rendering

#### 4.1.1 Threading Model

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
- **Module pipeline runs on the physics thread** between `mj_step1()` and `mj_step2()`. Fast modules (PID, EKF, DWA) complete well within the 2ms budget.
- **Slow planners (RRT*, MPC, A*) run asynchronously**: they are launched on a separate planning thread and produce results that the pipeline consumes on the next tick via a thread-safe buffer. The controller uses the last-known-good plan until a new one arrives.
- **Render thread gets a copy** of `mjData` via `mj_copyData()` under a mutex. This is the standard MuJoCo pattern — `mjData` is not thread-safe, so the render thread never touches the physics `mjData` directly.
- **ImGui state changes** (module switching, parameter tuning) are queued and applied on the physics thread at the start of the next tick.

### 4.2 Bridge Layer (`simulation/bridge/`)

The bridge is the only code that includes `<mujoco/mujoco.h>`:

- **SensorAdapter** -- reads `mjData` sensor arrays -> `common::LidarScan`, `ImuReading`, `CameraFrame`, etc.
- **StateAdapter** -- reads `mjData` qpos/qvel -> `common::Pose3D`, `Pose2D`, `TerrainPose`
- **ActuatorAdapter** -- takes `common::Twist` from controllers -> writes `mjData::ctrl[]`
- **ModelAdapter** -- reads `mjModel` at startup -> populates `VehicleParams`, `MotorParams`, sensor configs

### 4.3 MJCF as Source of Truth

Robot and world parameters are defined in MJCF files. The `ModelAdapter` extracts what robotics modules need (`VehicleParams`, sensor configurations, kinematic structure) from the loaded `mjModel` at startup. Modules never read MJCF directly.

For real hardware deployment (no MuJoCo), a separate hardware config loader would populate the same `common/` structs. Modules don't care where params came from.

### 4.4 Module Pipeline

Pluggable robotics modules wired together at runtime:

```
Sensors (via SensorAdapter)
  -> IStateEstimator (EKF, particle filter, ...)
  -> IGlobalPlanner (A*, RRT, ...)
  -> ILocalPlanner (DWA, potential field, ...)
  -> IController (PID, pure_pursuit, MPC, ...)
  -> ActuatorAdapter -> MuJoCo ctrl[]
```

- Runtime switching via ImGui dropdowns
- Each slot has an interface defined in `common/` or its domain
- Pipeline reads sensor data from bridge, computes commands, feeds back through bridge

### 4.5 App Shell (GLFW + MuJoCo Render + ImGui)

Single executable combining:

- **GLFW** window creation + OpenGL context
- **MuJoCo** `mjr_render()` for 3D scene in the main viewport
- **ImGui** overlay panels (GLFW + OpenGL3 backend):
  - Simulation control (play/pause/step/reset/speed)
  - Module pipeline selector (dropdowns for controller, planner, estimator, kinematics)
  - Parameter tuning (PID gains, lookahead distance, etc.)
  - Telemetry plots (velocity, tracking error, sensor data)
  - Scenario loader (load different MJCF worlds)
- **Threading:** physics on dedicated thread, render + UI on main thread (OpenGL requirement). See Section 4.1.1 for the full threading model.
- **Implementation starting point:** MuJoCo's own `simulate.cc` sample application demonstrates GLFW + MuJoCo rendering + UI integration. The app shell should reference this pattern but replace mjUI with ImGui.
- **ImGui docking:** Use the ImGui docking branch (`imgui-docking`) to allow panels to be docked, resized, and rearranged without occluding the 3D viewport.

### 4.6 Scenario System

- MJCF files live in `simulation/scenarios/`
- Each scenario defines: terrain + robot(s) + obstacles + sensor configuration
- Loaded at startup via command line or switched at runtime via ImGui
- Multi-robot scenarios are native (multiple bodies in one MJCF)

---

## 5. Directory Structure

```
workspace/
+-- architecture.md              <- Rewritten for 2-tier MuJoCo architecture
+-- CMakeLists.txt               <- Remove frontends, update simulation deps
+-- cmake/deps.cmake             <- Add MuJoCo/GLFW, remove Crow/SDL2/IXWebSocket/httplib
+-- robotics/
|   +-- common/
|   |   +-- types_3d/            <- Pose3D, Transform3D, Quaternion, Twist3D, Wrench
|   |   +-- sensors/             <- LidarScan, ImuReading, CameraFrame, DepthFrame, ForceTorqueSensor
|   |   +-- kinematics/          <- Existing + future 3D kinematic models
|   |   +-- robot/               <- Existing VehicleParams etc.
|   |   +-- environment/         <- Existing TerrainProperties etc.
|   |   +-- logging/             <- Unchanged
|   |   +-- transforms/          <- Existing + 3D conversion utilities
|   |   +-- noise_models/        <- Unchanged
|   +-- control/                 <- Unchanged
|   +-- perception/              <- Unchanged
|   +-- state_estimation/        <- Unchanged
|   +-- motion_planning/         <- Unchanged
|   +-- fleet_management/        <- Unchanged
+-- simulation/
|   +-- CMakeLists.txt           <- Single executable, links MuJoCo + ImGui + modules
|   +-- include/simulation/
|   |   +-- bridge/              <- SensorAdapter, StateAdapter, ActuatorAdapter, ModelAdapter
|   |   +-- pipeline/            <- Module pipeline wiring + runtime switching
|   |   +-- app/                 <- GLFW window, ImGui panels, render loop
|   +-- src/
|   |   +-- main.cpp             <- Entry point: load MJCF, create window, run
|   |   +-- bridge/              <- Bridge implementations
|   |   +-- pipeline/            <- Pipeline implementations
|   |   +-- app/                 <- App shell, ImGui panels, render
|   |   +-- scenario_loader/     <- Scenario loading logic
|   +-- scenarios/               <- MJCF model files
|   +-- tests/                   <- Integration tests (headless MuJoCo)
|   +-- docs/
|       +-- design.md
+-- frontends/                   <- DELETED entirely
```

---

## 6. Dependency Changes

### 6.1 Added

| Dependency | Purpose |
|------------|---------|
| MuJoCo (latest 3.x stable) | Physics engine |
| GLFW | Windowing (may come transitively via MuJoCo) |

**MuJoCo integration strategy:** MuJoCo's CMake build pulls its own transitive dependencies (abseil-cpp, lodepng, tinyxml2, ccd, qhull). The preferred approach is FetchContent from the MuJoCo GitHub repo. If FetchContent causes target conflicts with the parent build, fall back to `ExternalProject_Add` (builds MuJoCo in isolation) or system install with `find_package(mujoco)`. This should be validated early in M0.

### 6.2 Kept

| Dependency | Purpose |
|------------|---------|
| Eigen 3.4 | Linear algebra (all modules) |
| Catch2 v3 | Unit testing |
| spdlog | Logging backend for common/logging/ (observability gate requirement) |
| nlohmann/json | JSON serialization (parameter configs, telemetry export, VDA 5050 messages) |
| OSQP | QP solver (MPC, DMPC, multi-robot) |
| ImGui (docking branch) | UI panels (rebuilt with GLFW+OpenGL3 backend instead of SDL2) |

### 6.3 Removed

| Dependency | Reason |
|------------|--------|
| Crow | No network API layer |
| SDL2 | Replaced by GLFW |
| IXWebSocket | No WebSocket clients |
| cpp-httplib | No HTTP clients |

### 6.4 ImGui Backend Change

- Old: `imgui_impl_sdl2.cpp` + `imgui_impl_opengl3.cpp`
- New: `imgui_impl_glfw.cpp` + `imgui_impl_opengl3.cpp`

---

## 7. Milestone Impact

### 7.1 Updated Milestones

**M0 -- Dev Infrastructure**
- Devcontainer needs MuJoCo + GLFW + OpenGL deps
- CI builds simulation with MuJoCo headless (no GPU required for physics-only tests)
- Remove Crow/WebSocket scaffolding

**M1 -- Minimum Viable Robot**
- Core deliverable: MuJoCo sim with bridge + GLFW/ImGui app (replaces Crow server + native frontend)
- MJCF model for differential drive robot on flat ground
- Bridge: MuJoCo sensors -> LidarScan, MuJoCo qpos -> Pose2D
- ImGui panels: play/pause, cmd_vel, module selector
- New in common/: Pose3D, sensor types, conversion utilities
- Modules unchanged: PID, A*, DWA, EKF, lidar, occupancy grid

**M2 -- Hardening & Testing**
- Headless MuJoCo enables fast CI integration tests
- Interface freeze includes bridge adapters + sensor types

**M3 -- Control Upgrades**
- Unchanged conceptually (pure_pursuit, MPC, CBF)
- MPC (acados) unaffected -- generates C code, no MuJoCo coupling

**M3.5 -- Vehicle Dynamics**
- Enhanced: MuJoCo natively handles contact, friction, terrain
- Bicycle model validates against MuJoCo's own dynamics
- Terrain interaction uses MuJoCo heightfield meshes + `TerrainPose`

**M4 -- Perception Upgrades**
- MuJoCo provides simulated sensors (rangefinders, contact, cameras) via bridge
- Richer sensor data from 3D world, projected to module formats

**M5 -- State Estimation**
- Unchanged -- modules consume `common/` sensor types

**M6 -- Visual Perception**
- Major simplification: MuJoCo renders camera images via offscreen `mjr_render()`
- No need for custom "2.5D sim renderer"
- `CameraFrame` and `DepthFrame` types feed feature_extraction, visual_odometry

**M6.5 -- SLAM**
- Unchanged conceptually, benefits from richer sensor simulation

**M7 -- Advanced Planning**
- Unchanged

**M8 -- Multi-Robot**
- Simplified: MuJoCo handles N-body physics natively (multiple robots in one MJCF)
- No need for custom multi-robot sim infrastructure

**M10 -- Polish & Showcase**
- Redefined: no deployable web demo
- Focus on docs, recorded demos, packaging
- ImGui app is the showcase
- Updated dependency: M10 <- M8 (M9 removed from dependency chain)

**M11-M24 -- Advanced modules**
- Conceptually unchanged -- pure robotics algorithms
- Benefit from MuJoCo's richer physics without module changes
- Notable: M12 (Fleet Management) depends on M8 (Multi-Robot), which is simplified by MuJoCo's native N-body support. Fleet simulation scenarios use multi-robot MJCF models.

### 7.2 Removed Milestones

**M9 -- Web Frontend**
- Removed. No React/Canvas/WebSocket app.
- WASM export noted as a future possibility but not a milestone.
- Rationale: no network API, no frontends tier. If web deployment is desired in the future, MuJoCo WASM is the path, but it must not constrain module design for real robot deployment.

### 7.3 New Work Absorbed into M1

The following replaces what was previously spread across M1 + M9 + frontends scaffolding:

- GLFW + MuJoCo + ImGui app shell
- Bridge layer (SensorAdapter, StateAdapter, ActuatorAdapter, ModelAdapter)
- First MJCF scenario (differential drive on flat ground)
- 3D types in `common/` (Pose3D, Quaternion, Transform3D, sensor types)
- Conversion utilities (Pose3D <-> Pose2D, TerrainPose)

---

## 8. Design Decisions Summary

| Decision | Choice | Rationale |
|----------|--------|-----------|
| Physics engine | MuJoCo | Fast, accurate, 3D native, headless capable, MJCF models |
| 3D scope | Ground robots first, architected for 6-DOF expansion | Avoids rewriting all modules while enabling future growth |
| Network layer | None (removed Crow) | Unnecessary without separate frontends; WASM is the future web path |
| Visualization | MuJoCo render + ImGui overlay in single executable | MuJoCo handles 3D; ImGui handles custom UI; no separate frontend |
| Windowing | GLFW (replaces SDL2) | MuJoCo's native windowing library |
| Robot description | MJCF is source of truth | Mature format; bridge extracts `common/` structs from `mjModel` |
| Module isolation | No MuJoCo includes in `workspace/robotics/` | Modules stay portable -- copy to real robot project and build |
| Type system | Dual 2D + 3D with 2.5D bridge types | 2D algorithms stay 2D; 3D available for future modules; TerrainPose bridges ground-robot slope awareness |
| Sensor types | `common/sensors/` with hardware-agnostic types | Same types work with MuJoCo sim or real hardware drivers |

---

## 9. Future Possibilities (Not In Scope)

- **MuJoCo WASM** -- compile simulation for browser-based portfolio demo. Only pursue if it doesn't constrain module C++ for real robot deployment.
- **Network API** -- if remote visualization or multi-client access is ever needed, a thin WebSocket layer can be added around the simulation. The bridge layer's clean interfaces make this straightforward.
- **ROS 2 bridge** -- the same bridge pattern (adapters converting between MuJoCo and `common/` types) could be extended to publish/subscribe ROS 2 topics for real robot integration.

---

## 10. Superseded Items

This redesign supersedes the following:

- **Deferred PR review items** referencing Crow, WebSocket, the native frontend renderer, or the web frontend are no longer applicable. The memory file `project_deferred_review_items.md` should be cleaned up accordingly.
- **WebSocket state message schema** (Section "WebSocket -- state streaming" in old architecture.md) -- replaced by the bridge layer's direct type population.
- **REST API specification** (all `/api/*` endpoints in old architecture.md) -- removed entirely. Module switching and simulation control are handled via ImGui panels.
- **Frontends README files** and scaffolding CMakeLists -- deleted.
