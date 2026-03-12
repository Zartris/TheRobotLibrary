cmake_minimum_required(VERSION 3.20)
project(priority_planning VERSION 0.1.0 LANGUAGES CXX)

if(NOT TARGET common)
    message(FATAL_ERROR "[priority_planning] Target 'common' not found. Build as part of the workspace.")
endif()
if(NOT TARGET astar)
    message(FATAL_ERROR "[priority_planning] Target 'astar' not found. Build as part of the workspace.")
endif()

file(GLOB_RECURSE SOURCES src/*.cpp)

if(SOURCES)
    add_library(priority_planning STATIC ${SOURCES})
    set(_vis PUBLIC)
else()
    add_library(priority_planning INTERFACE)
    set(_vis INTERFACE)
    message(STATUS "[priority_planning] No source files yet — INTERFACE target")
endif()

target_include_directories(priority_planning ${_vis}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
# Uses A* as the per-robot single-agent planner (intra-domain dep: both in motion_planning).
target_link_libraries(priority_planning ${_vis}
    common
    astar
)
target_compile_features(priority_planning ${_vis} cxx_std_20)

add_library(robotlib::priority_planning ALIAS priority_planning)

if(BUILD_TESTING AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/CMakeLists.txt")
    add_subdirectory(tests)
endif()
