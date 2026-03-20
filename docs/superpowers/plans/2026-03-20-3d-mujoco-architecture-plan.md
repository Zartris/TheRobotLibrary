# 3D MuJoCo Architecture Refactor — Implementation Plan

> **For agentic workers:** REQUIRED: Use superpowers:subagent-driven-development (if subagents available) or superpowers:executing-plans to implement this plan. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Update all planning documents, architecture docs, CMake files, and scaffolding to reflect the new 2-tier MuJoCo-based architecture, removing the old Crow/WebSocket/frontends infrastructure.

**Architecture:** This is a documentation/planning refactor, not a code implementation. The project has no implemented code yet — only scaffolding (CMakeLists, READMEs) and planning docs (milestones, module task files). All references to Crow, WebSocket, REST API, SDL2, separate frontends must be replaced with the MuJoCo + bridge + ImGui integrated architecture.

**Tech Stack:** Markdown, CMake. No code compilation in this plan.

**Spec:** `docs/superpowers/specs/2026-03-20-3d-mujoco-architecture-design.md`

---

## Chunk 1: Core Infrastructure Updates

### Task 1: Update `workspace/cmake/deps.cmake`

**Files:**
- Modify: `workspace/cmake/deps.cmake`

- [ ] **Step 1: Read current deps.cmake**

Read `workspace/cmake/deps.cmake` to understand current structure.

- [ ] **Step 2: Remove Crow dependency block**

Remove the entire Crow FetchContent block (lines ~63-75):
```cmake
# Crow v1.2 — C++ HTTP + WebSocket server (simulation backend)
```

- [ ] **Step 3: Remove SDL2 dependency block**

Remove the entire SDL2 FetchContent block (lines ~119-131):
```cmake
# SDL2 release-2.30.0 — window / input / OpenGL context (frontends/native)
```

- [ ] **Step 4: Remove cpp-httplib dependency block**

Remove the entire cpp-httplib FetchContent block (lines ~93-103):
```cmake
# cpp-httplib v0.15.3 — header-only HTTP/HTTPS client (frontends/native)
```

- [ ] **Step 5: Remove IXWebSocket dependency block**

Remove the entire IXWebSocket FetchContent block (lines ~108-116):
```cmake
# IXWebSocket v11.4.4 — WebSocket client (frontends/native)
```

- [ ] **Step 6: Update ImGui block — change SDL2 backend to GLFW backend**

Replace the ImGui FetchContent block. Change the backend files from `imgui_impl_sdl2.cpp` to `imgui_impl_glfw.cpp`. Replace `SDL2::SDL2` link with a GLFW target. Use the docking branch. Update the comment to reference simulation app instead of frontends/native.

**Important ordering:** The MuJoCo block (Step 7) MUST appear before the ImGui block in deps.cmake because ImGui links against `glfw` which comes transitively via MuJoCo. The final order in deps.cmake should be: Eigen -> Catch2 -> nlohmann_json -> OSQP -> **MuJoCo** -> **ImGui**.

**ImGui tag:** The docking branch does not use versioned tags like `v1.91.8-docking`. Use `GIT_TAG docking` to track the docking branch head. For reproducibility, the executor should pin to a specific commit hash from the docking branch once validated. Note this in a CMake comment.

New ImGui block (placed AFTER the MuJoCo block):
```cmake
# ---------------------------------------------------------------------------
# Dear ImGui (docking branch) — immediate-mode GUI (simulation app)
# ImGui has no official CMakeLists; we create a STATIC target manually.
# NOTE: Must appear after MuJoCo in this file — ImGui links glfw which
# comes transitively via MuJoCo's FetchContent.
# Pin to a specific docking branch commit hash for reproducibility once validated.
# ---------------------------------------------------------------------------
if(NOT TARGET imgui::imgui)
    FetchContent_Declare(imgui
        GIT_REPOSITORY https://github.com/ocornut/imgui.git
        GIT_TAG        docking
        GIT_SHALLOW    TRUE
    )
    FetchContent_MakeAvailable(imgui)

    find_package(OpenGL REQUIRED)

    add_library(imgui STATIC
        ${imgui_SOURCE_DIR}/imgui.cpp
        ${imgui_SOURCE_DIR}/imgui_draw.cpp
        ${imgui_SOURCE_DIR}/imgui_tables.cpp
        ${imgui_SOURCE_DIR}/imgui_widgets.cpp
        # GLFW + OpenGL3 back-end
        ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
        ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
    )
    target_include_directories(imgui PUBLIC
        ${imgui_SOURCE_DIR}
        ${imgui_SOURCE_DIR}/backends
    )
    target_link_libraries(imgui PUBLIC glfw OpenGL::GL)
    add_library(imgui::imgui ALIAS imgui)
endif()
```

- [ ] **Step 7: Add MuJoCo dependency block**

Add after the OSQP block (BEFORE ImGui — see ordering note in Step 6):
```cmake
# ---------------------------------------------------------------------------
# MuJoCo (latest 3.x stable) — physics engine (simulation backend)
# MuJoCo brings its own transitive deps: abseil, lodepng, tinyxml2, ccd, qhull.
# If FetchContent causes target conflicts, fall back to ExternalProject_Add
# or system install with find_package(mujoco). Validate in M0.
# GLFW comes transitively via MuJoCo.
# ---------------------------------------------------------------------------
if(NOT TARGET mujoco)
    FetchContent_Declare(mujoco
        GIT_REPOSITORY https://github.com/google-deepmind/mujoco.git
        GIT_TAG        3.3.2
        GIT_SHALLOW    TRUE
    )
    set(MUJOCO_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
    set(MUJOCO_BUILD_SIMULATE OFF CACHE BOOL "" FORCE)
    set(MUJOCO_BUILD_TESTS    OFF CACHE BOOL "" FORCE)
    set(MUJOCO_BUILD_PYTHON   OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(mujoco)
    # NOTE: MuJoCo FetchContent exports target as 'mujoco' (no namespace).
    # When using find_package(mujoco) it exports 'mujoco::mujoco'.
    # Verify the actual target name during M0 validation.
endif()
```

- [ ] **Step 8: Update header comments**

Update the file header comments to reflect the new dependency set. Remove references to Crow, SDL2, IXWebSocket, cpp-httplib. Add MuJoCo to the Used by: comments for relevant deps.

- [ ] **Step 9: Commit**

```bash
git add workspace/cmake/deps.cmake
git commit -m "build: update deps for MuJoCo architecture — add MuJoCo, remove Crow/SDL2/IXWebSocket/httplib, switch ImGui to GLFW+docking"
```

---

### Task 2: Update `workspace/CMakeLists.txt`

**Files:**
- Modify: `workspace/CMakeLists.txt`

- [ ] **Step 1: Read current workspace CMakeLists.txt**

Read `workspace/CMakeLists.txt` to understand current structure.

- [ ] **Step 2: Remove frontends subdirectory inclusion**

Remove the line `add_subdirectory(frontends)` (line ~29). The frontends tier no longer exists.

- [ ] **Step 3: Verify simulation subdirectory is still included**

Ensure `add_subdirectory(simulation)` is present. The simulation tier remains but with different dependencies.

- [ ] **Step 4: Commit**

```bash
git add workspace/CMakeLists.txt
git commit -m "build: remove frontends subdirectory from workspace build"
```

---

### Task 3: Delete frontends tier

**Files:**
- Delete: `workspace/frontends/` (entire directory)
- Delete: `repo-plans/modules/native_frontend.md`

- [ ] **Step 1: Delete the frontends directory**

```bash
rm -rf workspace/frontends/
```

- [ ] **Step 2: Delete native_frontend module task file**

```bash
rm repo-plans/modules/native_frontend.md
```

- [ ] **Step 3: Commit**

```bash
git rm -r workspace/frontends/ && git rm repo-plans/modules/native_frontend.md
git commit -m "refactor: remove frontends tier — visualization integrated into simulation"
```

---

### Task 4: Update `workspace/simulation/CMakeLists.txt`

**Files:**
- Modify: `workspace/simulation/CMakeLists.txt`

- [ ] **Step 1: Read current simulation CMakeLists.txt**

Read `workspace/simulation/CMakeLists.txt`.

- [ ] **Step 2: Rewrite CMakeLists.txt for MuJoCo architecture**

Replace the entire contents with:
```cmake
cmake_minimum_required(VERSION 3.20)
project(simulation VERSION 0.1.0 LANGUAGES CXX)

# -- Third-party dependencies -------------------------------------------------
# In-tree builds: workspace/cmake/deps.cmake already declared these.
# Standalone: build from workspace root so deps.cmake is included automatically.
# NOTE: MuJoCo target may be 'mujoco' (FetchContent) or 'mujoco::mujoco'
# (find_package). Verify during M0 validation and update accordingly.
if(NOT TARGET mujoco)
    message(FATAL_ERROR
        "[simulation] Target 'mujoco' not found.\n"
        "Build from the workspace root so deps.cmake is included automatically.")
endif()
if(NOT TARGET imgui::imgui)
    message(FATAL_ERROR
        "[simulation] Target 'imgui::imgui' not found.\n"
        "Build from the workspace root so deps.cmake is included automatically.")
endif()
# ------------------------------------------------------------------------------

find_package(OpenGL REQUIRED)

# Collect sources — executable grows as features are added
file(GLOB_RECURSE SOURCES src/*.cpp)

if(NOT SOURCES)
    message(STATUS "[simulation] No source files yet — skipping executable target")
    return()
endif()

add_executable(simulation_app ${SOURCES})

target_include_directories(simulation_app PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(simulation_app PRIVATE
    # Physics engine (target name may be 'mujoco' or 'mujoco::mujoco' — verify in M0)
    mujoco

    # GUI overlay (GLFW + OpenGL3 backend, GLFW comes via MuJoCo)
    imgui::imgui
    OpenGL::GL

    # JSON (scenario configs, parameter export, telemetry)
    # nlohmann_json::nlohmann_json

    # Robotics modules consumed by the simulation at runtime.
    # Add as needed when implementing scenarios:
    # robotlib::common
    # robotlib::ekf
    # robotlib::occupancy_grid
    # robotlib::pid
    # robotlib::astar
    # robotlib::dwa
)

target_compile_features(simulation_app PRIVATE cxx_std_20)

if(BUILD_TESTING AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/CMakeLists.txt")
    add_subdirectory(tests)
endif()
```

- [ ] **Step 3: Commit**

```bash
git add workspace/simulation/CMakeLists.txt
git commit -m "build: rewrite simulation CMakeLists for MuJoCo + ImGui (replaces Crow)"
```

---

### Task 5: Update `workspace/simulation/README.md`

**Files:**
- Modify: `workspace/simulation/README.md`

- [ ] **Step 1: Read current simulation README.md**

Read `workspace/simulation/README.md`.

- [ ] **Step 2: Rewrite README for MuJoCo architecture**

Replace the entire contents with:
```markdown
# Simulation

A standalone C++ application that runs the 3D simulated world using MuJoCo physics
and provides an integrated ImGui control panel for visualization and interaction.

**Important:** The simulation is the single source of truth for world state. It is a
single executable — there are no separate frontends or network APIs. Visualization
(MuJoCo 3D rendering) and controls (ImGui panels) are integrated into the same process.

---

## Responsibilities

- Loading MJCF model files (robots, terrain, sensors, world scenarios)
- Running MuJoCo physics at a configurable timestep (default 2ms / 500Hz)
- Bridging MuJoCo state to `common/` types for robotics modules
- Running the module pipeline (estimator -> planner -> controller) each physics tick
- Rendering the 3D scene via MuJoCo's OpenGL renderer
- Providing ImGui overlay panels for simulation control and module configuration
- Supporting headless mode (no window) for batch testing and CI

---

## Technology

- **Physics:** [MuJoCo](https://mujoco.org/) — fast, accurate rigid-body dynamics with contact
- **Windowing:** GLFW (comes transitively via MuJoCo)
- **3D Rendering:** MuJoCo's built-in OpenGL renderer (`mjr_render()`)
- **UI Overlay:** [Dear ImGui](https://github.com/ocornut/imgui) (docking branch, GLFW+OpenGL3 backend)
- **Build:** CMake, produces an executable `simulation_app`
- **Dependencies:** MuJoCo, ImGui, and the robotics modules from
  `workspace/robotics/` (linked directly)

---

## Threading Model

- **Physics thread:** runs `mj_step1()` -> module pipeline -> `mj_step2()` in a tight loop
- **Main thread (OpenGL):** copies `mjData` under mutex, renders 3D scene, draws ImGui panels
- **Async planning thread:** slow planners (RRT*, MPC) run asynchronously and deliver results
  to the pipeline via a thread-safe buffer

---

## Directory Layout

```
simulation/
+-- CMakeLists.txt
+-- README.md               <- This file
+-- include/simulation/
|   +-- bridge/             # SensorAdapter, StateAdapter, ActuatorAdapter, ModelAdapter
|   +-- pipeline/           # Module pipeline wiring + runtime switching
|   +-- app/                # GLFW window, ImGui panels, render loop
+-- src/
|   +-- main.cpp            # Entry point: load MJCF, create window, run
|   +-- bridge/             # Bridge implementations
|   +-- pipeline/           # Pipeline implementations
|   +-- app/                # App shell, ImGui panels, render
|   +-- scenario_loader/    # Scenario loading logic
+-- scenarios/              # MJCF model files
+-- tests/                  # Integration tests (headless MuJoCo)
+-- docs/
    +-- design.md
```

---

## Running

```bash
# Build from workspace root
cmake -B build -S workspace -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)

# Run with default scenario
./build/simulation/simulation_app

# Run with a specific scenario
./build/simulation/simulation_app --scenario scenarios/diff_drive_flat.xml

# Run headless (no window, for CI/testing)
./build/simulation/simulation_app --headless --scenario scenarios/diff_drive_flat.xml
```

- [ ] **Step 3: Commit**

```bash
git add workspace/simulation/README.md
git commit -m "docs: rewrite simulation README for MuJoCo + ImGui architecture"
```

---

### Task 6: Rewrite `workspace/architecture.md`

**Files:**
- Modify: `workspace/architecture.md`

- [ ] **Step 1: Read current architecture.md**

Read `workspace/architecture.md` in full.

- [ ] **Step 2: Rewrite architecture.md**

This is a full rewrite. The new document must cover:

1. **Overview** — Two-tier structure (robotics modules + simulation). No frontends tier. Single executable.
2. **Architecture diagram** — Replace the 3-tier ASCII diagram with the 2-tier diagram from the spec (Section 2.1).
3. **Dependency rule** — Simplified: modules depend only on common; simulation depends on MuJoCo + modules.
4. **Robotics Modules section** — Keep module index table, standard module layout, dependency graph. These are unchanged.
5. **Simulation Backend section** — Complete rewrite:
   - MuJoCo physics core (MJCF loading, split-step loop, headless mode)
   - Bridge layer (SensorAdapter, StateAdapter, ActuatorAdapter, ModelAdapter)
   - Module pipeline (pluggable modules, runtime switching via ImGui)
   - App shell (GLFW + MuJoCo render + ImGui overlay, threading model from spec Section 4.1.1)
   - Scenario system (MJCF files in simulation/scenarios/)
   - Remove ALL references to: Crow, WebSocket, REST API, `/api/*` endpoints, state message JSON schema, 30Hz streaming
6. **Frontends section** — DELETE entirely. No native frontend, no web frontend.
7. **Type system** — Add a new section documenting the 3-level coordinate types (SE2, SE3, 2.5D) and sensor data types from spec Section 3.
8. **C++ Conventions** — Keep as-is.
9. **Building section** — Update prerequisites (remove OpenGL dev headers note for frontends, add MuJoCo note). Update build commands if needed.
10. **Adding a New Module** — Keep as-is.
11. **Directory Structure** — Update to remove `frontends/` subtree. Update simulation subtree to match new layout.

Key replacements throughout:
- "Crow HTTP + WebSocket server" -> "MuJoCo physics + GLFW + ImGui"
- "WebSocket state stream" -> "bridge layer"
- "REST API" -> "ImGui panels"
- "simulation_server" -> "simulation_app"
- "/api/robot/controller" etc. -> "ImGui module selector"
- "2D" world/robot references -> "3D" where appropriate
- "SDL2" -> "GLFW"
- Remove all WebSocket/REST endpoint tables
- Remove all JSON schema examples

- [ ] **Step 3: Commit**

```bash
git add workspace/architecture.md
git commit -m "docs: rewrite architecture.md for 2-tier MuJoCo architecture"
```

---

### Task 7: Update `CLAUDE.md`

**Files:**
- Modify: `CLAUDE.md` (repo root)

- [ ] **Step 1: Read current CLAUDE.md**

Read `/repo/CLAUDE.md` in full.

- [ ] **Step 2: Update Architecture section**

Replace the three-tier description with the two-tier description. Remove Tier 3 (Frontends). Update Tier 2 (Simulation) to describe MuJoCo + bridge + ImGui. Update the dependency rule.

- [ ] **Step 3: Update Build Commands section**

- Keep cmake build commands (unchanged)
- Update "Run all tests" if simulation test binary name changes
- Remove any frontend build references
- Note that `simulation_app` is the main executable

- [ ] **Step 4: Update Gotchas section**

- Remove or update any gotcha referencing frontends, WebSocket, Crow, or REST API
- Add gotcha: "MuJoCo's `mjData` is not thread-safe — the render thread uses `mj_copyData()`"
- Add gotcha: "MJCF files in `simulation/scenarios/` are the source of truth for robot parameters"

- [ ] **Step 5: Update Key Reference Files section**

- Remove frontend README references
- Update simulation description

- [ ] **Step 6: Update deps.cmake description**

Update the deps.cmake reference to mention MuJoCo instead of Crow.

- [ ] **Step 7: Commit**

```bash
git add CLAUDE.md
git commit -m "docs: update CLAUDE.md for MuJoCo architecture"
```

---

### Task 8: Clean up deferred review items memory

**Files:**
- Modify: `/home/zartris/.claude/projects/-repo/memory/project_deferred_review_items.md`
- Modify: `/home/zartris/.claude/projects/-repo/memory/MEMORY.md`

- [ ] **Step 1: Read deferred review items**

Read the memory file to see what items exist.

- [ ] **Step 2: Remove or update superseded items**

Remove any items referencing Crow, WebSocket, native frontend renderer, REST API, or the web frontend. These are superseded by the architecture redesign (per spec Section 10).

- [ ] **Step 3: Commit memory changes (if tracked) or just save**

Memory files are not in the git repo, so just save them.

---

## Chunk 2: Simulation & Common Module Plans

### Task 9: Rewrite `repo-plans/modules/simulation.md`

**Files:**
- Modify: `repo-plans/modules/simulation.md`

- [ ] **Step 1: Read current simulation.md**

Read `repo-plans/modules/simulation.md`.

- [ ] **Step 2: Rewrite for MuJoCo architecture**

The simulation module plan must be completely rewritten. The old plan describes a Crow HTTP/WebSocket server. The new plan describes:

**Module:** simulation
**Milestone:** M1
**Status:** Not Started
**Dependencies:** common (for types), MuJoCo (physics), ImGui (UI)

**7-phase structure (per module-task-template.md):**

Phase 1 — Scaffold:
- `simulation/include/simulation/bridge/sensor_adapter.hpp`
- `simulation/include/simulation/bridge/state_adapter.hpp`
- `simulation/include/simulation/bridge/actuator_adapter.hpp`
- `simulation/include/simulation/bridge/model_adapter.hpp`
- `simulation/include/simulation/pipeline/module_pipeline.hpp`
- `simulation/include/simulation/app/app_shell.hpp`
- `simulation/include/simulation/app/imgui_panels.hpp`
- `simulation/src/main.cpp`
- `simulation/src/bridge/*.cpp`
- `simulation/src/pipeline/*.cpp`
- `simulation/src/app/*.cpp`
- `simulation/src/scenario_loader/scenario_loader.cpp`

Phase 2 — Core implementation:
- MuJoCo model loading from MJCF
- Split-step physics loop (`mj_step1` -> control -> `mj_step2`)
- Bridge adapters: read `mjData` -> `common::` types, write control -> `mjData::ctrl[]`
- Model adapter: extract `VehicleParams` etc. from `mjModel` at startup
- Headless mode support (no window creation)

Phase 3 — Module pipeline:
- Pluggable module slots (IStateEstimator, IGlobalPlanner, ILocalPlanner, IController)
- Runtime switching
- Async planning thread for slow planners
- Thread-safe result buffer

Phase 4 — App shell:
- GLFW window + OpenGL context
- MuJoCo 3D rendering (`mjr_render()`)
- ImGui overlay (GLFW+OpenGL3 backend, docking branch)
- ImGui panels: sim control, module selector, parameter tuning, telemetry plots, scenario loader
- Threading: physics thread + main render thread + mutex-based `mj_copyData()`

Phase 5 — Scenarios:
- First MJCF scenario: differential drive robot on flat ground
- Scenario directory structure: `simulation/scenarios/`

Phase 6 — Testing:
- Headless integration tests (MuJoCo physics only, no window)
- Bridge adapter unit tests (mock `mjData` -> verify correct `common::` type output)
- Pipeline wiring tests

Phase 7 — Observability:
- Integrate `common/logging/` throughout bridge and pipeline

Replace ALL references to Crow, WebSocket, REST, `/api/*`, state JSON schema, 30Hz streaming.

- [ ] **Step 3: Commit**

```bash
git add repo-plans/modules/simulation.md
git commit -m "docs: rewrite simulation module plan for MuJoCo architecture"
```

---

### Task 10: Update `repo-plans/modules/common.md`

**Files:**
- Modify: `repo-plans/modules/common.md`

- [ ] **Step 1: Read current common.md**

Read `repo-plans/modules/common.md`.

- [ ] **Step 2: Add 3D types and sensor types to the plan**

Add new phases/tasks for:
- `common/types_3d/`: Pose3D, Transform3D, Quaternion, Twist3D, Wrench, TerrainPose
- `common/sensors/`: LidarScan, ImuReading, CameraFrame, DepthFrame, ForceTorqueSensor
- `common/transforms/`: 3D conversion utilities (Pose3D <-> Pose2D, Euler <-> quaternion)
- All types in `robotlib` namespace
- All header-only, Eigen-based, no MuJoCo includes
- Tests for quaternion math, SE3 transforms, type conversions

Keep all existing 2D type tasks. The 3D types are additive.

- [ ] **Step 3: Commit**

```bash
git add repo-plans/modules/common.md
git commit -m "docs: add 3D types and sensor types to common module plan"
```

---

### Task 11: Update `repo-plans/module-task-template.md`

**Files:**
- Modify: `repo-plans/module-task-template.md`

- [ ] **Step 1: Read current template**

Read `repo-plans/module-task-template.md`.

- [ ] **Step 2: Update integration phases**

Replace references to:
- "Add module output to WebSocket state message" -> "Add module output to bridge state adapter"
- "Add rendering layer to native frontend (ImGui ImDrawList)" -> "Add visualization to ImGui telemetry panel (if applicable)"
- "(Later -- M9) Mirror rendering in web frontend Canvas 2D" -> Remove this line entirely
- "frontend visuals" -> "visual inspection"
- Phase 6 header: "Frontend Visualization" -> "Visualization" (if present)

- [ ] **Step 3: Commit**

```bash
git add repo-plans/module-task-template.md
git commit -m "docs: update module task template for MuJoCo architecture"
```

---

## Chunk 3: Roadmap & Milestone Updates

### Task 12: Update `repo-plans/README.md` (roadmap)

**Files:**
- Modify: `repo-plans/README.md`

- [ ] **Step 1: Read current roadmap**

Read `repo-plans/README.md`.

- [ ] **Step 2: Update milestone table**

- M1 description: change "common + lidar + grid + A* + DWA + PID + EKF + sim + frontend" -> "common + lidar + grid + A* + DWA + PID + EKF + MuJoCo sim + ImGui"
- M9 row: remove entirely (or mark as "Removed — see architecture redesign spec")
- M10 description: change to "Docs site, examples, packaging, recorded demos"

- [ ] **Step 3: Update dependency graph**

- Remove M9 node and all edges to/from it
- Update M10 dependency: `M10 <- M8` (no longer depends on M9)
- Update the text paragraph below the graph that mentions M9

- [ ] **Step 4: Update Philosophy section if it references frontends**

Check and update any references to "frontends", "web frontend", "native frontend".

- [ ] **Step 5: Commit**

```bash
git add repo-plans/README.md
git commit -m "docs: update roadmap — remove M9 web frontend, update M1/M10, fix dependency graph"
```

---

### Task 13: Update M1 milestone (`M1-minimum-viable-robot.md`)

**Files:**
- Modify: `repo-plans/milestones/M1-minimum-viable-robot.md`

This is the most heavily affected milestone. It requires substantial rewriting.

- [ ] **Step 1: Read current M1 milestone**

Read `repo-plans/milestones/M1-minimum-viable-robot.md` in full.

- [ ] **Step 2: Rewrite simulation sections**

Replace ALL Crow/WebSocket/REST references:
- "Crow WS+REST" -> "MuJoCo physics + bridge"
- `include/simulation/api_server.hpp` -> `include/simulation/bridge/*.hpp`, `include/simulation/app/*.hpp`
- `curl http://localhost:8080/api/*` examples -> "ImGui panel interaction"
- REST endpoint tables -> "Module switching via ImGui dropdown"
- "WebSocket state broadcast at 30Hz" -> "Bridge reads mjData each physics tick"
- "simulation_server" -> "simulation_app"

- [ ] **Step 3: Rewrite frontend section**

Replace the native frontend module (SDL2+ImGui+IXWebSocket+cpp-httplib over network) with the integrated ImGui app:
- Remove "Frontend | native | SDL2+ImGui, WS state receiver, REST commander, 2D renderer" from module table
- Replace with simulation app's ImGui panels as the visualization layer
- Remove all references to IXWebSocket, cpp-httplib, ws_client, rest_client
- 2D renderer -> MuJoCo 3D renderer

- [ ] **Step 4: Remove web frontend from out-of-scope**

Update the out-of-scope/exclusions list: web frontend is no longer "later" — it's removed from the architecture.

- [ ] **Step 5: Update exit criteria**

- "WebSocket connects and streams" -> "Bridge populates common types from mjData"
- "REST commands work" -> "ImGui panels control simulation"
- "Frontend renders" -> "MuJoCo 3D scene renders with ImGui overlay"
- "All visible in ImGui frontend: grid, robot, lidar, path, EKF ellipse" -> "All visible in simulation app: 3D robot, terrain, lidar beams, planned path, EKF state"

- [ ] **Step 6: Add new M1 items from spec Section 7.3**

Add tasks for:
- 3D types in common/ (Pose3D, sensor types, conversions)
- First MJCF scenario (differential drive on flat ground)
- Bridge layer implementation
- GLFW + MuJoCo + ImGui app shell

- [ ] **Step 7: Commit**

```bash
git add repo-plans/milestones/M1-minimum-viable-robot.md
git commit -m "docs: rewrite M1 milestone for MuJoCo + ImGui architecture"
```

---

### Task 14: Delete M9 milestone

**Files:**
- Delete: `repo-plans/milestones/M9-web-frontend.md`

- [ ] **Step 1: Delete the file**

```bash
rm repo-plans/milestones/M9-web-frontend.md
```

- [ ] **Step 2: Commit**

```bash
git add repo-plans/milestones/M9-web-frontend.md
git commit -m "docs: remove M9 web frontend milestone — superseded by architecture redesign"
```

---

### Task 15: Update remaining milestone files (batch)

**Files:**
- Modify: All milestone files listed below

This task covers mechanical updates across many milestone files. The changes follow consistent patterns.

**Pattern A — REST API references:**
Replace any mention of `PUT /api/robot/<module>`, `POST /api/*`, `GET /api/*` with the equivalent ImGui interaction. For example:
- "hot-swap via REST `PUT /api/robot/controller`" -> "hot-swap via ImGui module selector"
- "REST endpoint for planner switching" -> "ImGui dropdown for planner switching"

**Pattern B — WebSocket references:**
Replace any mention of WebSocket state extension or streaming with bridge-based equivalents:
- "extend WebSocket state message with X" -> "bridge StateAdapter exposes X via common types"
- "WebSocket state stream at 30Hz" -> "bridge reads state from mjData each physics tick"

**Pattern C — Frontend rendering references:**
- "rendered in native frontend" -> "rendered in MuJoCo 3D scene / ImGui telemetry panel"
- "Visual difference clear in frontend" -> "Visual difference clear in simulation app"
- "Frontend renders X" -> "Simulation app renders X"

**Pattern D — Crow/server references:**
- "Crow server" -> "simulation app"
- "simulation_server" -> "simulation_app"

**Pattern E — M9/web frontend references:**
- Remove any "Later -- M9" notes
- Remove "web frontend" from any milestone scope

**Pattern F — SDL2 references:**
- "SDL2+ImGui" -> "GLFW+MuJoCo+ImGui"

**Pattern G — Phase 4.5 observability boilerplate:**
Many module files contain the template phrase "not just frontend visuals" in their Phase 4.5 observability section. Replace with:
- "not just frontend visuals" -> "not just visual inspection"

- [ ] **Step 1: Update M0-dev-infrastructure.md**

Read and update:
- Devcontainer/CI deps: replace Crow with MuJoCo + GLFW
- Remove any WebSocket/Crow scaffolding tasks
- Add: validate MuJoCo FetchContent integration

- [ ] **Step 2: Update M2-hardening-testing.md**

Read and update:
- "frontend changes" -> "simulation app changes"
- Integration test references: now use headless MuJoCo
- Interface freeze includes bridge adapters

- [ ] **Step 3: Update M3-control-upgrades.md**

Read and update:
- Lines 11, 28, 53-54, 68, 86: all REST API endpoints -> ImGui selectors
- Line 110: "frontend" -> "simulation app"

- [ ] **Step 4: Update M3.5-vehicle-dynamics.md**

Read and update:
- Line 68: "PUT /api/sim/physics_mode" -> "ImGui physics mode toggle"
- Lines 83, 112-113: WebSocket/REST -> bridge + ImGui
- MuJoCo natively handles terrain physics now — note this enhancement

- [ ] **Step 5: Update M4-perception-upgrades.md**

Read and update. MuJoCo provides richer sensor simulation — note this.

- [ ] **Step 6: Update M5-state-estimation-upgrades.md**

Read and update:
- Lines 35-36: REST/WebSocket -> ImGui/bridge
- Line 55: "frontend" -> "simulation app"

- [ ] **Step 7: Update M6-visual-perception.md**

Read and update:
- Line 85: "CameraFrame streamed over WebSocket at 30Hz" -> "CameraFrame populated by bridge SensorAdapter from MuJoCo offscreen render"
- Note: no need for custom 2.5D renderer — MuJoCo handles camera simulation natively

- [ ] **Step 8: Update M6.5-slam.md**

Read and update:
- Line 61: camera simulation infrastructure uses MuJoCo, not WebSocket streaming

- [ ] **Step 9: Update M7-advanced-planning.md**

Read and update:
- Line 87: REST -> ImGui planner selector

- [ ] **Step 10: Update M8-multi-robot.md**

Read and update:
- Lines 24-25: WebSocket/REST -> bridge/ImGui
- Lines 91-92: frontend -> simulation app
- Note: MuJoCo handles N-body physics natively

- [ ] **Step 11: Update M10-polish-showcase.md**

Read and update:
- Line 60: "Web frontend deployed to GitHub Pages" -> "Recorded demos and documentation published"
- Remove M9 dependency
- Focus on docs, recorded demos, packaging

- [ ] **Step 12: Update M11-advanced-control.md**

Read and update:
- Lines 36, 69: REST API -> ImGui selectors

- [ ] **Step 13: Update M12-fleet-management.md**

Read and update:
- Lines 99-114: REST API fleet endpoints -> ImGui fleet panel
- Lines 112-114: "native frontend fleet panel" -> "ImGui fleet panel in simulation app"

- [ ] **Step 14: Update M13-classical-optimal-control.md**

Read and update:
- Lines 37, 56, 67: REST API -> ImGui

- [ ] **Step 15: Update M14-advanced-state-estimation-2.md**

Read and update:
- Line 66: "frontend" -> "simulation app"

- [ ] **Step 16: Update M15-visual-inertial-odometry.md**

Read and update for any WebSocket/frontend references.

- [ ] **Step 17: Update M16-planning-upgrades-2.md**

Read and update:
- Line 37: REST API -> ImGui

- [ ] **Step 18: Update M17-camera-perception-2.md**

Read and update for any frontend/WebSocket references.

- [ ] **Step 19: Update M18-advanced-nonlinear-control.md**

Read and update:
- Lines 33, 51: REST API -> ImGui

- [ ] **Step 20: Update M19-depth-perception-3d.md**

Read and update:
- Line 53: "frontend" -> "simulation app"

- [ ] **Step 21: Update M20-planning-upgrades-3.md**

Read and update:
- Lines 32, 49: REST API -> ImGui

- [ ] **Step 22: Update M21-estimation-test-foundations.md**

Read and update for any references.

- [ ] **Step 23: Update M22-optimal-feedback-automotive.md**

Read and update:
- Line 31: REST API -> ImGui

- [ ] **Step 24: Update M23-lattice-semantic.md**

Read and update:
- Line 33: REST API -> ImGui

- [ ] **Step 25: Update M24-fleet-operations.md**

Read and update:
- Line 39: fleet REST endpoints -> ImGui fleet panel

- [ ] **Step 26: Commit all milestone updates**

```bash
git add repo-plans/milestones/
git commit -m "docs: update all milestones for MuJoCo architecture — replace REST/WebSocket/frontend refs"
```

---

## Chunk 4: Module Plan Updates

### Task 16: Update module task files with REST/WebSocket/frontend references (batch)

**Files:**
- Modify: All module files listed below

These are mechanical updates. Each module file has a Phase 6 "Simulation Integration" step that typically says something like:
- "Add module output to WebSocket state message"
- "Add REST endpoint for module swapping"
- "Render output in native frontend"

All of these follow the same replacement patterns from Task 15.

**Modules with REST `/api/` references (replace with ImGui selector):**

- [ ] **Step 1: Update pid.md** (line ~51)
- [ ] **Step 2: Update astar.md** (line ~51)
- [ ] **Step 3: Update dwa.md** (line ~50)
- [ ] **Step 4: Update ekf.md** (line ~50)
- [ ] **Step 5: Update prm.md** (line ~51)
- [ ] **Step 6: Update stanley.md** (line ~51)
- [ ] **Step 7: Update lattice_planner.md** (line ~53)
- [ ] **Step 8: Update feedback_linearization.md** (line ~52)
- [ ] **Step 9: Update lqr.md** (line ~51)
- [ ] **Step 10: Update lqg.md** (line ~51)
- [ ] **Step 11: Update mppi.md** (line ~53)
- [ ] **Step 12: Update potential_field.md** (line ~51)
- [ ] **Step 13: Update informed_rrt_star.md** (line ~51)
- [ ] **Step 14: Update ukf.md** (line ~49)
- [ ] **Step 15: Update charging_station.md** (line ~59)
- [ ] **Step 16: Update vehicle_dynamics.md** (line ~85)

**Modules with WebSocket references:**

- [ ] **Step 17: Update visual_slam.md** (line ~81)
- [ ] **Step 18: Update motor_model.md** (line ~78)

**Modules with frontend rendering references:**

- [ ] **Step 19: Update occupancy_grid.md** (line ~53) — "native frontend" -> "simulation app"
- [ ] **Step 20: Update ray_casting.md** (line ~53) — "native frontend" -> "simulation app"
- [ ] **Step 21: Update semantic_segmentation.md** (line ~60) — update frontend reference
- [ ] **Step 22: Update pose_graph.md** (line ~60) — "SLAM frontends" -> "simulation app"

**Modules with "frontend visuals" boilerplate (Phase 4.5 observability — apply Pattern G):**

- [ ] **Step 23: Update lidar_processing.md** — "frontend visuals" -> "visual inspection"
- [ ] **Step 24: Update velocity_profiling.md** — same
- [ ] **Step 25: Update depth_camera.md** — same
- [ ] **Step 26: Update factor_graph.md** — same
- [ ] **Step 27: Update lane_detection.md** — same
- [ ] **Step 28: Update noise_models.md** — same
- [ ] **Step 29: Update object_detection_3d.md** — same
- [ ] **Step 30: Update polynomial.md** — same
- [ ] **Step 31: Update stereo_depth.md** — same
- [ ] **Step 32: Update visual_inertial_odometry.md** — same
- [ ] **Step 33: Update feature_extraction.md** — same (if boilerplate present)
- [ ] **Step 34: Update visual_odometry.md** — same (if boilerplate present)

**Modules with REST/M9 references missed in initial scan:**

- [ ] **Step 35: Update param_estimation.md** — remove "(Later -- M9)" line
- [ ] **Step 36: Update imu_processing.md** — update REST endpoint reference
- [ ] **Step 37: Update place_recognition.md** — update REST endpoint reference

- [ ] **Step 38: Commit all module updates**

```bash
git add repo-plans/modules/
git commit -m "docs: update all module plans for MuJoCo architecture — replace REST/WebSocket/frontend refs"
```

---

### Task 17: Update workspace/robotics/ README and docs files

**Files:**
- Modify: Various README.md and docs/theory.md files under `workspace/robotics/`

Several README and theory docs under `workspace/robotics/` reference the old architecture (frontends, REST, WebSocket). These need updating.

- [ ] **Step 1: Update workspace/robotics/README.md**

Read and update any references to frontends, simulation server, or the 3-tier architecture.

- [ ] **Step 2: Update domain README files**

Read and update if they reference the old architecture:
- `workspace/robotics/control/README.md`
- `workspace/robotics/perception/README.md`
- `workspace/robotics/motion_planning/README.md`
- `workspace/robotics/fleet_management/README.md`

- [ ] **Step 3: Update fleet_management theory docs**

These have substantive REST/WebSocket references:
- `workspace/robotics/fleet_management/docs/theory.md`
- `workspace/robotics/fleet_management/fleet_monitor/docs/theory.md`
- `workspace/robotics/fleet_management/vda5050/docs/theory.md`

Replace REST API endpoint references with bridge/ImGui equivalents. VDA 5050 message types are protocol-level (not our REST API) and should be kept.

- [ ] **Step 4: Update other docs with references**

- `workspace/robotics/control/cbf/README.md`
- `workspace/robotics/common/logging/README.md`
- `workspace/robotics/common/logging/docs/theory.md`

- [ ] **Step 5: Commit**

```bash
git add workspace/robotics/
git commit -m "docs: update robotics module READMEs and theory docs for MuJoCo architecture"
```

---

### Task 18: Update `repo-plans/todos.md` (renumbered from removed web frontend references)

**Files:**
- Modify: `repo-plans/todos.md`

- [ ] **Step 1: Read current todos.md**

Read `repo-plans/todos.md`.

- [ ] **Step 2: Check for any references that need updating**

Look for references to frontends, Crow, WebSocket, REST API. Update or remove as needed. The current content is mostly about game-theory motion planning modules which are unaffected.

- [ ] **Step 3: Commit if changed**

```bash
git add repo-plans/todos.md
git commit -m "docs: update todos for architecture changes (if applicable)"
```

---

### Task 19: Final verification

- [ ] **Step 1: Verify deletions succeeded**

```bash
test ! -d workspace/frontends/ && echo "PASS: frontends/ deleted" || echo "FAIL: frontends/ still exists"
test ! -f repo-plans/modules/native_frontend.md && echo "PASS: native_frontend.md deleted" || echo "FAIL: native_frontend.md still exists"
test ! -f repo-plans/milestones/M9-web-frontend.md && echo "PASS: M9 deleted" || echo "FAIL: M9 still exists"
```

- [ ] **Step 2: Search for any remaining old architecture references**

Run a grep across the entire repo for leftover references:
```bash
grep -r "Crow" workspace/ repo-plans/ CLAUDE.md --include="*.md" --include="*.cmake" --include="*.txt" -l
grep -r "WebSocket" workspace/ repo-plans/ CLAUDE.md --include="*.md" --include="*.cmake" --include="*.txt" -l
grep -r "ws://" workspace/ repo-plans/ CLAUDE.md --include="*.md" --include="*.cmake" --include="*.txt" -l
grep -r "/api/" workspace/ repo-plans/ CLAUDE.md --include="*.md" --include="*.cmake" --include="*.txt" -l
grep -r "IXWebSocket" workspace/ repo-plans/ CLAUDE.md --include="*.md" --include="*.cmake" --include="*.txt" -l
grep -r "cpp-httplib" workspace/ repo-plans/ CLAUDE.md --include="*.md" --include="*.cmake" --include="*.txt" -l
grep -r "SDL2" workspace/ repo-plans/ CLAUDE.md --include="*.md" --include="*.cmake" --include="*.txt" -l
grep -r "simulation_server" workspace/ repo-plans/ CLAUDE.md --include="*.md" --include="*.cmake" --include="*.txt" -l
grep -r "native_frontend" workspace/ repo-plans/ CLAUDE.md --include="*.md" --include="*.cmake" --include="*.txt" -l
grep -r "frontends/" workspace/ repo-plans/ CLAUDE.md --include="*.md" --include="*.cmake" --include="*.txt" -l
grep -r "frontend" workspace/ repo-plans/ CLAUDE.md --include="*.md" --include="*.cmake" --include="*.txt" -l
```

**Exclusions (ignore hits in these locations):**
- `docs/superpowers/specs/` — the spec doc mentions the old architecture for historical context
- `docs/superpowers/plans/` — older design documents (e.g., `2026-03-16-vehicle-dynamics-milestones.md`, `2026-03-13-visual-slam-milestones.md`) are historical artifacts and are not updated by this plan
- `repo-plans/todos.md` — game-theory items are unrelated

- [ ] **Step 3: Fix any remaining references found**

- [ ] **Step 4: Final commit if needed**

```bash
git commit -m "docs: fix remaining old architecture references"
```

- [ ] **Step 5: Verify clean build of CMake configuration**

```bash
cmake -B /tmp/verify-build -S workspace -DCMAKE_BUILD_TYPE=Release 2>&1 | head -50
```

This will likely fail (MuJoCo fetch may take time or have issues), but it should at least parse the CMake files without syntax errors. The actual build validation is M0 work.
