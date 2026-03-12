cmake_minimum_required(VERSION 3.20)
project(time_optimal VERSION 0.1.0 LANGUAGES CXX)

if(NOT TARGET common)
    message(FATAL_ERROR "[time_optimal] Target 'common' not found. Build as part of the workspace.")
endif()
if(NOT TARGET velocity_profiling)
    message(FATAL_ERROR "[time_optimal] Target 'velocity_profiling' not found. Build as part of the workspace.")
endif()
if(NOT TARGET spline_fitting)
    message(FATAL_ERROR "[spline_fitting] Target 'spline_fitting' not found. Build as part of the workspace.")
endif()

file(GLOB_RECURSE SOURCES src/*.cpp)

if(SOURCES)
    add_library(time_optimal STATIC ${SOURCES})
    set(_vis PUBLIC)
else()
    add_library(time_optimal INTERFACE)
    set(_vis INTERFACE)
    message(STATUS "[time_optimal] No source files yet — INTERFACE target")
endif()

target_include_directories(time_optimal ${_vis}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
# Depends on sibling trajectory modules; all within the motion_planning domain.
target_link_libraries(time_optimal ${_vis}
    common
    velocity_profiling
    spline_fitting
)
target_compile_features(time_optimal ${_vis} cxx_std_20)

add_library(robotlib::time_optimal ALIAS time_optimal)

if(BUILD_TESTING AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/CMakeLists.txt")
    add_subdirectory(tests)
endif()
