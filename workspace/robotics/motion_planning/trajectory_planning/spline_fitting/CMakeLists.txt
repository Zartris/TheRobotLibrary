cmake_minimum_required(VERSION 3.20)
project(spline_fitting VERSION 0.1.0 LANGUAGES CXX)

if(NOT TARGET common)
    message(FATAL_ERROR "[spline_fitting] Target 'common' not found. Build as part of the workspace.")
endif()

file(GLOB_RECURSE SOURCES src/*.cpp)

if(SOURCES)
    add_library(spline_fitting STATIC ${SOURCES})
    set(_vis PUBLIC)
else()
    add_library(spline_fitting INTERFACE)
    set(_vis INTERFACE)
    message(STATUS "[spline_fitting] No source files yet — INTERFACE target")
endif()

target_include_directories(spline_fitting ${_vis}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
# Eigen used directly for tridiagonal linear systems in spline coefficient solving;
# propagated transitively through common but listed explicitly for clarity.
target_link_libraries(spline_fitting ${_vis} common)
target_compile_features(spline_fitting ${_vis} cxx_std_20)

add_library(robotlib::spline_fitting ALIAS spline_fitting)

if(BUILD_TESTING AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/CMakeLists.txt")
    add_subdirectory(tests)
endif()
