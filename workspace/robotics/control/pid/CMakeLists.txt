cmake_minimum_required(VERSION 3.20)
project(pid VERSION 0.1.0 LANGUAGES CXX)

# ── Standalone: pull in common (and transitively Eigen) ─────────────────────
if(NOT TARGET common)
    message(FATAL_ERROR
        "[pid] Target 'common' not found.\n"
        "Build pid as part of the workspace, or add common as a subdirectory first.")
endif()

# ── Library target ───────────────────────────────────────────────────────────
file(GLOB_RECURSE SOURCES src/*.cpp)

if(SOURCES)
    add_library(pid STATIC ${SOURCES})
    set(_vis PUBLIC)
else()
    add_library(pid INTERFACE)
    set(_vis INTERFACE)
    message(STATUS "[pid] No source files yet — INTERFACE target")
endif()

target_include_directories(pid ${_vis}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(pid ${_vis} common)
target_compile_features(pid ${_vis} cxx_std_20)

add_library(robotlib::pid ALIAS pid)

# ── Tests ────────────────────────────────────────────────────────────────────
if(BUILD_TESTING AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/CMakeLists.txt")
    add_subdirectory(tests)
endif()
