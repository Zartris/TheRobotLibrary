cmake_minimum_required(VERSION 3.20)
project(dmpc VERSION 0.1.0 LANGUAGES CXX)

if(NOT TARGET common)
    message(FATAL_ERROR "[dmpc] Target 'common' not found. Build as part of the workspace.")
endif()

# ── OSQP standalone guard ────────────────────────────────────────────────────
# When building outside the workspace tree, fetch OSQP here.
# In-tree builds: deps.cmake in the workspace root handles this.
if(NOT TARGET osqp::osqp)
    include(FetchContent)
    FetchContent_Declare(osqp
        GIT_REPOSITORY https://github.com/osqp/osqp.git
        GIT_TAG        v0.6.3
        GIT_SHALLOW    TRUE
    )
    set(OSQP_BUILD_DEMO_EXE OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(osqp)
endif()
# ─────────────────────────────────────────────────────────────────────────────

file(GLOB_RECURSE SOURCES src/*.cpp)

if(SOURCES)
    add_library(dmpc STATIC ${SOURCES})
    set(_vis PUBLIC)
else()
    add_library(dmpc INTERFACE)
    set(_vis INTERFACE)
    message(STATUS "[dmpc] No source files yet — INTERFACE target")
endif()

target_include_directories(dmpc ${_vis}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(dmpc ${_vis}
    common       # Eigen via common
    osqp::osqp   # distributed QP solves per agent
)
target_compile_features(dmpc ${_vis} cxx_std_20)

add_library(robotlib::dmpc ALIAS dmpc)

if(BUILD_TESTING AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/CMakeLists.txt")
    add_subdirectory(tests)
endif()
