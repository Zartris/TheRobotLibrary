---
name: new-module
description: "Scaffold a new robotics module following TheRobotLibrary template. Args: <domain> <module-name>"
---

Scaffold a new robotics module at `workspace/robotics/<domain>/<module-name>/` following the standard template.

## Files to create

### `CMakeLists.txt`
```cmake
cmake_minimum_required(VERSION 3.20)
project(<module-name> VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Pull shared deps if building standalone
get_filename_component(_ws_root "${CMAKE_CURRENT_SOURCE_DIR}/../../../../.." ABSOLUTE)
if(EXISTS "${_ws_root}/cmake/deps.cmake")
    include("${_ws_root}/cmake/deps.cmake")
endif()

add_library(<module-name> STATIC
    src/<module-name>.cpp
)
add_library(robotlib::<module-name> ALIAS <module-name>)

target_include_directories(<module-name>
    PUBLIC  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:include>
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(<module-name>
    PUBLIC robotlib::common
)

target_compile_features(<module-name> PUBLIC cxx_std_20)

if(BUILD_TESTING AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/CMakeLists.txt")
    enable_testing()
    add_subdirectory(tests)
endif()
```

### `include/<module-name>/<module-name>.hpp`
- Public API header
- All types: PascalCase
- All methods: camelCase
- All member variables: m_camelCase
- All constants/macros: UPPER_SNAKE_CASE
- No raw owning pointers — use std::unique_ptr, std::shared_ptr, or value types
- Recoverable errors: std::expected<T, E> (not exceptions)
- Include and use ILogger from `common/logging/`

### `src/<module-name>.cpp`
- Implementation of the public API
- Must include and use `ILogger`/`SpdlogLogger` — this is the **Observability Gate** required before the module is considered complete

### `tests/CMakeLists.txt`
```cmake
include(CTest)
include(Catch2/extras/Catch.cmake OPTIONAL)

add_executable(<module-name>_tests
    <module-name>_tests.cpp
)
target_link_libraries(<module-name>_tests
    PRIVATE robotlib::<module-name> Catch2::Catch2WithMain
)
catch_discover_tests(<module-name>_tests)
```

### `tests/<module-name>_tests.cpp`
Catch2 v3 test skeleton:
```cpp
#include <catch2/catch_test_macros.hpp>
#include <<module-name>/<module-name>.hpp>

TEST_CASE("<ModuleName> basic construction", "[<module-name>]") {
    REQUIRE(true); // replace with real tests
}
```

### `docs/theory.md`
Placeholder explaining the algorithm/math behind the module.

### `README.md`
Brief description of what the module does, its API, and usage example.

## Architectural rules (enforce strictly)
- Module may ONLY depend on `common` — never on other robotics modules, simulation, or frontends
- Files/dirs: snake_case
- Types (classes, structs, enums): PascalCase
- Methods/functions: camelCase
- Member variables: m_camelCase
- Constants/macros: UPPER_SNAKE_CASE
- No raw owning pointers
- std::expected for recoverable errors; exceptions only for programmer errors
- ILogger must be used (Observability Gate)

After scaffolding, remind the user to add the new module's subdirectory to the parent `CMakeLists.txt` with `add_subdirectory(<module-name>)`.
