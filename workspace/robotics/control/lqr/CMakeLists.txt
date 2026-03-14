cmake_minimum_required(VERSION 3.20)
project(lqr VERSION 0.1.0 LANGUAGES CXX)

if(NOT TARGET common)
    message(FATAL_ERROR
        "[lqr] Target 'common' not found.\n"
        "Build lqr as part of the workspace, or add common as a subdirectory first.")
endif()

file(GLOB_RECURSE SOURCES src/*.cpp)

if(SOURCES)
    add_library(lqr STATIC ${SOURCES})
    set(_vis PUBLIC)
else()
    add_library(lqr INTERFACE)
    set(_vis INTERFACE)
    message(STATUS "[lqr] No source files yet — INTERFACE target")
endif()

target_include_directories(lqr ${_vis}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(lqr ${_vis} Eigen3::Eigen common)
target_compile_features(lqr ${_vis} cxx_std_20)

add_library(robotlib::lqr ALIAS lqr)

if(BUILD_TESTING AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/CMakeLists.txt")
    add_subdirectory(tests)
endif()
