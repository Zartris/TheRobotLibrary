cmake_minimum_required(VERSION 3.20)
project(TheRobotLibrary VERSION 0.1.0 LANGUAGES C CXX)

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
add_subdirectory(frontends)
