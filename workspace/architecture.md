# System Architecture

> **For AI agents:** Read this document in full before writing any code in this repository.
> It defines the component boundaries, communication protocols, naming conventions, and
> dependency rules that all code must follow. Do not deviate from these without updating
> this document first.

---

## Overview

TheRobotLibrary is built around three clearly separated components:

1. **Robotics modules** (`workspace/robotics/`) — Self-contained C++ libraries implementing
   individual robotics domains. They have no dependency on the simulation or frontends and
   can be used in any C++ project independently.

2. **Simulation backend** (`workspace/simulation/`) — A standalone C++ server that runs the
   simulated 2D world. It uses the robotics modules internally and exposes its state and
   control interface via WebSocket and REST HTTP.

3. **Frontends** (`workspace/frontends/`) — User interfaces that connect to the simulation
   over the network. The native ImGui frontend and the web frontend are entirely independent
   codebases. They share nothing except the API contract with the simulation server.

```
┌──────────────────────────────────────────────────────────────────────┐
│                            Frontends                                 │
│                                                                      │
│   ┌───────────────────────┐        ┌─────────────────────────────┐   │
│   │  Native (ImGui / C++) │        │   Web (TypeScript / React)  │   │
│   └───────────┬───────────┘        └──────────────┬──────────────┘   │
│               │                                   │                  │
└───────────────┼───────────────────────────────────┼──────────────────┘
                │  WebSocket  (state stream @ 30 Hz)│
                │  REST HTTP  (commands)            │
                ▼                                   ▼
┌──────────────────────────────────────────────────────────────────────┐
│                     Simulation Backend (C++)                         │
│                                                                      │
│  ┌─────────────────┐  ┌────────────────┐  ┌───────────────────────┐  │
│  │   World model   │  │  Sim loop      │  │  WebSocket + REST API │  │
│  └────────────┬────┘  └───────┬────────┘  └───────────────────────┘  │
│               └───────────────┘                                      │
│                       uses ▼                                         │
└──────────────────────────────────────────────────────────────────────┘
                             │
┌──────────────────────────────────────────────────────────────────────┐
│               Robotics Modules  (workspace/robotics/)                │
│                                                                      │
│  ┌──────────┐  ┌───────────┐  ┌──────────────────┐  ┌───────────┐    │
│  │  common  │  │  control  │  │ state_estimation │  │ planning  │    │
│  └──────────┘  └───────────┘  └──────────────────┘  └───────────┘    │
└──────────────────────────────────────────────────────────────────────┘
```

**Dependency rule (strictly enforced):**
- Robotics modules → no dependency on simulation or frontends
- Simulation → may depend on robotics modules, must not depend on frontends
- Frontends → communicate with simulation via API only; no direct library links

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
├── CMakeLists.txt          # Produces a static library; no workspace-level deps required
├── README.md               # What this module does and how to integrate it
├── include/
│   └── <module_name>/      # Public headers only (consumers add this to their include path)
├── src/                    # Implementation files
├── tests/                  # Unit tests (Catch2 or GoogleTest)
└── docs/
    └── theory.md           # Theory, math, and algorithm explanations for this domain
```

### Module index

Each domain contains sub-modules — copy only what you need.

| Domain              | Sub-module                  | Description                                              |
|---------------------|-----------------------------|----------------------------------------------------------|
| `common`            | *(flat)*                    | `Pose2D`, `Twist`, `Transform2D`, math utils, interfaces |
| `control`           | `pid/`                      | Discrete PID with anti-windup and derivative kick fix    |
|                     | `pure_pursuit/`             | Geometric path tracker; adaptive lookahead               |
|                     | `mpc/`                      | Receding-horizon QP controller for diff-drive            |
| `perception`        | `lidar_processing/`         | Scan filtering, DBSCAN segmentation, RANSAC lines        |
|                     | `occupancy_grid/`           | Binary Bayes grid, log-odds update, inflation            |
|                     | `ray_casting/`              | Bresenham / DDA ray traversal, noise injection           |
|                     | `obstacle_detection/`       | Detection pipeline, clustering, Kalman tracking          |
| `state_estimation`  | `ekf/`                      | Extended Kalman Filter for diff-drive + range-bearing    |
|                     | `particle_filter/`          | Monte Carlo Localization (MCL / AMCL)                    |
|                     | `ekf_slam/`                 | Augmented-state EKF-SLAM with landmark management        |
|                     | `lidar_slam/`               | ICP/NDT scan matching, pose-graph SLAM                   |
| `motion_planning`   | `global_planning/astar/`    | A\* on 2D grid; octile heuristic; weighted A\*           |
|                     | `global_planning/dijkstra/` | Dijkstra; multi-source distance maps                     |
|                     | `global_planning/rrt/`      | RRT and RRT\* for continuous C-space                     |
|                     | `local_planning/dwa/`       | Dynamic Window Approach; velocity sampling               |
|                     | `trajectory_planning/velocity_profiling/` | Trapezoidal / S-curve profiles; curvature-aware speed |
|                     | `trajectory_planning/spline_fitting/`     | Cubic spline, Catmull-Rom, B-spline path smoothing    |
|                     | `trajectory_planning/teb/`               | Timed Elastic Band: joint path + timing optimization  |
|                     | `trajectory_planning/time_optimal/`      | TOPP-RA, minimum-snap QP, bang-bang profiles          |
|                     | `multi_robot/orca/`         | VO→RVO→ORCA reactive collision avoidance              |
|                     | `multi_robot/priority_planning/` | Sequential priority-based multi-robot planning   |
|                     | `multi_robot/cbs/`          | Conflict-Based Search (optimal MAPF)                 |
|                     | `multi_robot/dmpc/`         | Distributed MPC with inter-robot trajectory constraints |
|                     | `multi_robot/mader/`        | MADER: decentralized async replanning (MIT ACL)       |

### Dependency graph between modules

```
common  ←  control
common  ←  perception
common  ←  state_estimation
common  ←  motion_planning
```

No module other than `common` may depend on another domain module. If cross-domain shared
data is needed, the type belongs in `common`.

---

## Simulation Backend (`workspace/simulation/`)

### Purpose

The simulation backend is a C++ application that:
- Maintains the simulated world state (robot pose, map, obstacles, sensor readings)
- Runs the simulation loop at a fixed timestep (configurable, default 50 Hz internally)
- Streams world state to connected clients via WebSocket at 30 Hz
- Accepts commands from clients via REST HTTP

The C++ server library used is **[Crow](https://crowcpp.org/)** — header-only, handles both
HTTP REST and WebSocket on the same server instance cleanly.

### WebSocket — state streaming

- **Endpoint:** `ws://<host>:8080/state`
- The server pushes a JSON state message every simulation tick
- Clients subscribe and receive; they do not send data over this channel

State message schema:
```json
{
  "t": 1234567890.123,
  "robot": {
    "pose": { "x": 1.2, "y": 3.4, "theta": 0.5 },
    "velocity": { "linear": 0.3, "angular": 0.1 }
  },
  "sensors": {
    "lidar": [0.5, 0.7, 1.2, 2.0, 1.8]
  },
  "map": {
    "width": 100,
    "height": 100,
    "resolution": 0.05,
    "data": "..."
  }
}
```

### REST HTTP — commands and configuration

- **Base URL:** `http://<host>:8080/api/`
- All request and response bodies are JSON

| Method | Endpoint               | Description                              |
|--------|------------------------|------------------------------------------|
| POST   | `/api/sim/start`       | Start or resume the simulation           |
| POST   | `/api/sim/stop`        | Pause the simulation                     |
| POST   | `/api/sim/reset`       | Reset world to initial state             |
| PUT    | `/api/sim/speed`       | Set simulation speed multiplier          |
| POST   | `/api/robot/cmd_vel`   | Send velocity command `{linear, angular}`|
| GET    | `/api/scenario/list`   | List available scenario names            |
| POST   | `/api/scenario/load`   | Load a scenario `{"name": "..."}`        |

### Simulation directory layout

```
simulation/
├── CMakeLists.txt
├── README.md
├── include/simulation/     # Public API headers (used by simulation internals)
├── src/                    # World model, sim loop, Crow server, scenario loader
├── tests/                  # Integration and unit tests
└── docs/
    └── design.md           # Simulation design decisions and extension guide
```

---

## Frontends (`workspace/frontends/`)

> **Critical rule:** Frontends communicate with the simulation backend exclusively through
> the WebSocket and REST API. They must not link against any simulation or robotics module
> libraries. This ensures each frontend remains independently deployable and independently
> testable.

### Native frontend (`workspace/frontends/native/`)

- **Language:** C++20
- **GUI framework:** ImGui with SDL2 + OpenGL backend (or GLFW + OpenGL)
- Connects to the simulation WebSocket for continuous state updates
- Sends commands via REST using a lightweight HTTP client (e.g. cpp-httplib)
- Renders the 2D robot world with ImGui's `ImDrawList` API
- Suitable for local development, debugging, and visualization on the host machine

### Web frontend (`workspace/frontends/web/`)

- **Language:** TypeScript
- **Framework:** React
- **Visualization:** HTML5 Canvas (2D rendering), upgradeable to WebGL / Three.js for 3D
- Connects to the simulation WebSocket with the browser-native `WebSocket` API
- Sends commands via `fetch()` REST calls
- Designed to be embedded on a personal homepage or portfolio site as a live demo

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
- **Testing:** Every module has a `tests/` directory. Tests use **Catch2** or **GoogleTest**.
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
├── architecture.md            ← This file (read before writing code)
├── CMakeLists.txt             ← Root build: wires all submodules together
├── robotics/
│   ├── README.md
│   ├── common/                ← Shared types and math primitives
│   ├── control/
│   │   ├── pid/
│   │   ├── pure_pursuit/
│   │   └── mpc/
│   ├── perception/
│   │   ├── lidar_processing/
│   │   ├── occupancy_grid/
│   │   ├── ray_casting/
│   │   └── obstacle_detection/
│   ├── state_estimation/
│   │   ├── ekf/
│   │   ├── particle_filter/
│   │   ├── ekf_slam/
│   │   └── lidar_slam/
│   └── motion_planning/
│       ├── global_planning/
│       │   ├── astar/
│       │   ├── dijkstra/
│       │   └── rrt/
│       ├── local_planning/
│       │   └── dwa/
│       ├── trajectory_planning/
│       │   ├── velocity_profiling/
│       │   ├── spline_fitting/
│       │   ├── teb/
│       │   └── time_optimal/
│       └── multi_robot/
│           ├── orca/
│           ├── priority_planning/
│           ├── cbs/
│           ├── dmpc/
│           └── mader/
├── simulation/                ← Simulation backend server
│   ├── README.md
│   ├── include/simulation/
│   ├── src/
│   ├── tests/
│   └── docs/
└── frontends/
    ├── README.md
    ├── native/                ← ImGui C++ desktop app
    │   ├── README.md
    │   └── src/
    └── web/                   ← TypeScript/React web app
        ├── README.md
        └── src/
```

---

## Building

### Prerequisites

- CMake ≥ 3.20
- A C++20-capable compiler: GCC 12+, Clang 15+, or MSVC 2022+
- Git (FetchContent downloads deps on first configure)
- For `frontends/native`: OpenGL development headers (`libgl1-mesa-dev` on Debian/Ubuntu)
- For `trajectory_planning/teb` (when implementing): `libsuitesparse-dev`

### Full workspace build

```bash
# Configure (first run downloads all FetchContent dependencies — allow a few minutes)
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
No vcpkg manifest, Conan recipe, or system packages are required (except OpenGL headers for
the native frontend).  
To add a new dependency:

1. Add a `FetchContent_Declare` block in `cmake/deps.cmake`, guarded with
   `if(NOT TARGET <target-name>)`.
2. Reference the target in the relevant module's `CMakeLists.txt`.
3. Update this document and the module's `README.md`.
