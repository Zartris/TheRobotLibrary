cmake_minimum_required(VERSION 3.20)
project(velocity_profiling VERSION 0.1.0 LANGUAGES CXX)

if(NOT TARGET common)
    message(FATAL_ERROR "[velocity_profiling] Target 'common' not found. Build as part of the workspace.")
endif()

file(GLOB_RECURSE SOURCES src/*.cpp)

if(SOURCES)
    add_library(velocity_profiling STATIC ${SOURCES})
    set(_vis PUBLIC)
else()
    add_library(velocity_profiling INTERFACE)
    set(_vis INTERFACE)
    message(STATUS "[velocity_profiling] No source files yet — INTERFACE target")
endif()

target_include_directories(velocity_profiling ${_vis}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(velocity_profiling ${_vis} common)   # Eigen via common
target_compile_features(velocity_profiling ${_vis} cxx_std_20)

add_library(robotlib::velocity_profiling ALIAS velocity_profiling)

if(BUILD_TESTING AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/CMakeLists.txt")
    add_subdirectory(tests)
endif()
