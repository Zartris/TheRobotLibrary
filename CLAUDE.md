# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build Commands

All builds target the `workspace/` directory as the CMake source root.

```bash
# Full workspace build (Release)
cmake -B build -S workspace -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)

# Debug build
cmake -B build-debug -S workspace -DCMAKE_BUILD_TYPE=Debug

# Single module build (e.g., pid)
cmake -B build-pid -S workspace/robotics/control/pid

# Run all tests
ctest --test-dir build --output-on-failure

# Run a single test binary (after build)
./build/robotics/control/pid/tests/pid_tests

# Run the simulation app (after build)
./build/simulation/simulation_app

# Format code (run from workspace/)
clang-format -i <file>
```

**Dependencies** are all managed via CMake FetchContent — no vcpkg or Conan needed. See [workspace/cmake/deps.cmake](workspace/cmake/deps.cmake) for the full list (MuJoCo, Eigen, Catch2, spdlog, ImGui, OSQP, nlohmann/json, etc.).

**Docker dev environment:**
```bash
cd docker
docker compose -f docker-compose.dev.yml up -d
docker exec -it TheRobotLibrary bash
```

## Architecture

TheRobotLibrary is a modular C++ robotics library with two tiers combined into a single executable:

### Tier 1: Robotics Modules (`workspace/robotics/`)
Self-contained, reusable C++ libraries. Modules may only depend on `common` — never on each other or simulation. Pure C++20/Eigen, no MuJoCo dependency. Domains: `common`, `control`, `perception`, `state_estimation`, `motion_planning`, `fleet_management`.

### Tier 2: Simulation (`workspace/simulation/`)
A single executable combining MuJoCo physics, GLFW windowing, MuJoCo 3D rendering, and ImGui control panels. Links against robotics modules via a bridge layer. No network API — all interaction through the integrated UI. Can run headless for CI.

### Dependency Rule
```
Robotics modules -> only depend on common (Eigen-based, no MuJoCo)
Simulation       -> depends on MuJoCo, robotics modules, ImGui, GLFW
```

### Workflow
- **Use superpowers workflow for all new features, modules, and milestones:** Brainstorm -> plan -> execute -> review.
- During brainstorm, always have a recommended answer ready when asking questions — don't leave choices open-ended.
- Specs go in `docs/superpowers/specs/YYYY-MM-DD-<topic>-design.md`
- Plans go in `docs/superpowers/plans/YYYY-MM-DD-<topic>-milestones.md`

## Module Template

Every module follows this layout:
```
<module>/
+-- CMakeLists.txt
+-- README.md
+-- include/<module>/   # public headers only
+-- src/                # implementation files
+-- tests/              # Catch2 unit tests + CMakeLists.txt
+-- docs/
    +-- theory.md       # math/algorithm explanation
```

Each module produces a CMake `STATIC` or `INTERFACE` target named `<module>` with alias `robotlib::<module>`. Tests are conditionally included: `if(BUILD_TESTING AND EXISTS tests/CMakeLists.txt)`.

## Code Conventions

From [workspace/architecture.md](workspace/architecture.md):

| Element | Convention |
|---------|-----------|
| Files/directories | `snake_case` |
| Types (classes, structs, enums) | `PascalCase` |
| Methods/functions | `camelCase` |
| Member variables | `m_camelCase` |
| Constants/macros | `UPPER_SNAKE_CASE` |

- **C++ standard:** C++20 minimum — use `std::ranges`, `std::span`, `std::optional`, `std::expected`
- **Memory:** No raw owning pointers — use `std::unique_ptr`, `std::shared_ptr`, or value types
- **Error handling:** `std::expected<T, E>` for recoverable errors; exceptions only for programmer errors
- **Testing:** Catch2 v3 (`TEST_CASE`, `REQUIRE`, `CHECK`)

## Observability Gate

All modules must use `common/logging/` (`ILogger` + `SpdlogLogger`) before being considered complete. This is the "Phase 4.5" gate in the milestone checklist.

## Claude Code Hooks (already active)

- **Auto-format:** Any `.cpp`/`.hpp`/`.h` edit automatically runs `clang-format` (config: `workspace/.clang-format`)
- **deps.cmake guard:** Edits to `workspace/cmake/deps.cmake` are blocked until explicitly confirmed
- **Co-Authored-By check:** After any `git commit`, warns if AI attribution trailer slipped through
- **Task completion reminder:** Fires at end of each response — re-read tracker before declaring done

## Skills

- `/new-module <domain> <name>` — scaffold a new module from template
- `/module-status [module]` — show implementation status across the workspace

## Gotchas

- **Never add Co-Authored-By trailers to commits.** A `commit-msg` hook blocks AI attribution. Install after cloning:
  ```bash
  ln -sf ../../.github/hooks/commit-msg .git/hooks/commit-msg
  ```
- **Before claiming all tasks done**, re-read the task tracker file and list any items NOT marked complete.
- **`common/` has sub-modules:** `types_3d/` (Pose3D, Quaternion, Transform3D, Twist3D, Wrench, TerrainPose), `sensors/` (LidarScan, ImuReading, CameraFrame, DepthFrame, ForceTorqueSensor), `kinematics/` (IKinematicModel, IDynamicModel), `robot/` (VehicleParams, MotorParams, TireParams, WheelConfig), `environment/` (TerrainProperties, SlipDetector), `logging/`, `transforms/`, `noise_models/`. Header-only types only — no logic.
- **TerrainMap lives in simulation**, not common. Only `TerrainProperties` (the struct) lives in `common/environment/`.
- **Module task files** live in `repo-plans/modules/<module>.md` and move to `repo-plans/modules/done/` when complete.
- **MuJoCo's `mjData` is not thread-safe** — the render thread uses `mj_copyData()` under a mutex. Never access physics `mjData` from the render thread directly.
- **MJCF files in `simulation/scenarios/` are the source of truth for robot parameters.** The `ModelAdapter` extracts `VehicleParams` and sensor configs from the loaded `mjModel` at startup. Modules never read MJCF directly.
- **GLFW comes transitively via MuJoCo** — do not add a separate GLFW FetchContent block in `deps.cmake`.

## Key Reference Files

- [workspace/architecture.md](workspace/architecture.md) — system design, type system, module index (read before adding a new module)
- [workspace/cmake/deps.cmake](workspace/cmake/deps.cmake) — all third-party dependencies (MuJoCo, Eigen, Catch2, OSQP, nlohmann/json, spdlog, ImGui, etc.)
- [repo-plans/README.md](repo-plans/README.md) — M0-M24 milestone roadmap
- [repo-plans/modules/](repo-plans/modules/) — per-module 7-phase task files (one per module, moved to done/ when complete)
- [repo-plans/todos.md](repo-plans/todos.md) — active work items
- [docs/superpowers/specs/](docs/superpowers/specs/) — design specs (brainstorm output)
- [docs/superpowers/plans/](docs/superpowers/plans/) — implementation plans (plan output, executed by subagent-driven-development)
