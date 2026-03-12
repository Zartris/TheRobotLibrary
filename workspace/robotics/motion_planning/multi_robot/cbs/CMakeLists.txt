cmake_minimum_required(VERSION 3.20)
project(cbs VERSION 0.1.0 LANGUAGES CXX)

if(NOT TARGET common)
    message(FATAL_ERROR "[cbs] Target 'common' not found. Build as part of the workspace.")
endif()
if(NOT TARGET astar)
    message(FATAL_ERROR "[cbs] Target 'astar' not found. Build as part of the workspace.")
endif()

file(GLOB_RECURSE SOURCES src/*.cpp)

if(SOURCES)
    add_library(cbs STATIC ${SOURCES})
    set(_vis PUBLIC)
else()
    add_library(cbs INTERFACE)
    set(_vis INTERFACE)
    message(STATUS "[cbs] No source files yet — INTERFACE target")
endif()

target_include_directories(cbs ${_vis}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
# CBS's low-level planner is A* (intra-domain dep: both in motion_planning).
target_link_libraries(cbs ${_vis}
    common
    astar
)
target_compile_features(cbs ${_vis} cxx_std_20)

add_library(robotlib::cbs ALIAS cbs)

if(BUILD_TESTING AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/CMakeLists.txt")
    add_subdirectory(tests)
endif()
