cmake_minimum_required(VERSION 3.20)
project(mader VERSION 0.1.0 LANGUAGES CXX)

if(NOT TARGET common)
    message(FATAL_ERROR "[mader] Target 'common' not found. Build as part of the workspace.")
endif()
if(NOT TARGET time_optimal)
    message(FATAL_ERROR "[mader] Target 'time_optimal' not found. Build as part of the workspace.")
endif()

file(GLOB_RECURSE SOURCES src/*.cpp)

if(SOURCES)
    add_library(mader STATIC ${SOURCES})
    set(_vis PUBLIC)
else()
    add_library(mader INTERFACE)
    set(_vis INTERFACE)
    message(STATUS "[mader] No source files yet — INTERFACE target")
endif()

target_include_directories(mader ${_vis}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
# MADER uses time-optimal trajectories as its inter-agent trajectory primitive.
# Both time_optimal and velocity_profiling/spline_fitting are transitively available.
target_link_libraries(mader ${_vis}
    common
    time_optimal
)
target_compile_features(mader ${_vis} cxx_std_20)

add_library(robotlib::mader ALIAS mader)

if(BUILD_TESTING AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/CMakeLists.txt")
    add_subdirectory(tests)
endif()
