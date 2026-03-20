cmake_minimum_required(VERSION 3.20)
project(TheRobotLibrary VERSION 0.1.0 LANGUAGES C CXX)

# Several FetchContent dependencies (OSQP v0.6.3, qhull via MuJoCo) use
# cmake_minimum_required(VERSION < 3.5) which CMake 4.x rejects by default.
# This global override lets CMake 4.x configure them as if they require 3.5.
# See: https://cmake.org/cmake/help/latest/variable/CMAKE_POLICY_VERSION_MINIMUM.html
if(CMAKE_VERSION VERSION_GREATER_EQUAL "4.0")
    set(CMAKE_POLICY_VERSION_MINIMUM 3.5)
endif()

# ── C++ standard ────────────────────────────────────────────────────────────
set(CMAKE_CXX_STANDARD          20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS        OFF)

# ── Compile database for clangd / IDE tooling ───────────────────────────────
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ── Default build type ───────────────────────────────────────────────────────
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
        Debug Release RelWithDebInfo MinSizeRel)
endif()

# ── Third-party dependencies (fetched once, shared by all sub-directories) ──
include(cmake/deps.cmake)

# ── Testing ──────────────────────────────────────────────────────────────────
option(BUILD_TESTING "Build unit tests" ON)
include(CTest)

# ── Sub-directories ──────────────────────────────────────────────────────────
add_subdirectory(robotics)
add_subdirectory(simulation)
