# TheRobotLibrary — OpenCode Instructions

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
- **Logging**: All modules must use `common/logging/` (`ILogger` + `SpdlogLogger`) — mandatory before a module is considered complete

## Build

- CMake source root: `workspace/`
- All deps via FetchContent — no vcpkg or Conan
- `workspace/cmake/deps.cmake` is protected — do not edit without explicit intent
- Format: `clang-format -i --style=file:workspace/.clang-format <file>`

### Workflow
- Use superpower workflow: Brainstorm → plan → execute → review for all new features, modules, and milestones.
- During brainstorm, when asking questions always have a recommended answer ready to guide the response and why that is recommended (don't leave it open-ended).

## Module Template

```
<module>/
├── CMakeLists.txt
├── README.md
├── include/<module>/
├── src/
├── tests/
└── docs/theory.md
```

## Key Reference Files

- `workspace/architecture.md` — full system design and API specs
- `workspace/cmake/deps.cmake` — all third-party dependencies
- `repo-plans/README.md` — M0–M12 milestone roadmap
- `repo-plans/todos.md` — active work items
