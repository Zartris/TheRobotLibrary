# TheRobotLibrary — GitHub Copilot Instructions

## Architecture

TheRobotLibrary is a modular C++20 robotics library with three strictly separated tiers:

- **Tier 1 — Robotics modules** (`workspace/robotics/`): Self-contained libraries. May only depend on `common`. Never on each other, simulation, or frontends.
- **Tier 2 — Simulation backend** (`workspace/simulation/`): Crow HTTP/WebSocket server, 50 Hz sim loop, 30 Hz state streaming. May link robotics modules.
- **Tier 3 — Frontends** (`workspace/frontends/`): Native (ImGui+SDL2) and Web (TypeScript/React). Network-only communication — never link against simulation or robotics libraries.

## Code Conventions

| Element | Convention |
|---------|-----------|
| Files/directories | `snake_case` |
| Types (classes, structs, enums) | `PascalCase` |
| Methods/functions | `camelCase` |
| Member variables | `m_camelCase` |
| Constants/macros | `UPPER_SNAKE_CASE` |

- **C++ standard**: C++20 — use `std::ranges`, `std::span`, `std::optional`, `std::expected`
- **Memory**: No raw owning pointers — use `std::unique_ptr`, `std::shared_ptr`, or value types
- **Error handling**: `std::expected<T, E>` for recoverable errors; exceptions only for programmer errors
- **Testing**: Catch2 v3 (`TEST_CASE`, `REQUIRE`, `CHECK`)
- **Logging**: All modules must use `common/logging/` (`ILogger` + `SpdlogLogger`) — this is mandatory before a module is considered complete

### Workflow
- Use superpower workflow: Brainstorm → plan → execute → review for all new features, modules, and milestones.
- During brainstorm, when asking questions always have a recommended answer ready to guide the response and why that is recommended (don't leave it open-ended).

## Module Template

Every module follows this layout:
```
<module>/
├── CMakeLists.txt          # STATIC target named <module>, alias robotlib::<module>
├── README.md
├── include/<module>/       # public headers only
├── src/                    # implementation files
├── tests/                  # Catch2 unit tests + CMakeLists.txt
└── docs/
    └── theory.md           # math/algorithm explanation
```

Tests are conditionally included: `if(BUILD_TESTING AND EXISTS tests/CMakeLists.txt)`

## Build System

- All builds target `workspace/` as the CMake source root
- Dependencies managed via CMake FetchContent — no vcpkg or Conan
- See `workspace/cmake/deps.cmake` for all third-party deps (Eigen, Catch2, Crow, OSQP, nlohmann/json, etc.)
- `cmake/deps.cmake` is the single source of truth for dependencies — do not edit without explicit intent

## Scaffolding a New Module

When asked to create a new robotics module at `workspace/robotics/<domain>/<module-name>/`, always generate all of:
1. `CMakeLists.txt` — STATIC library target `<module-name>` with alias `robotlib::<module-name>`; conditionally include tests/
2. `include/<module-name>/<module-name>.hpp` — public API, following all naming conventions
3. `src/<module-name>.cpp` — implementation including `ILogger` usage (mandatory)
4. `tests/CMakeLists.txt` + `tests/<module-name>_tests.cpp` — Catch2 v3 skeleton
5. `docs/theory.md` — algorithm/math placeholder
6. `README.md` — brief description

Then remind to add `add_subdirectory(<module-name>)` to the parent `CMakeLists.txt`.
