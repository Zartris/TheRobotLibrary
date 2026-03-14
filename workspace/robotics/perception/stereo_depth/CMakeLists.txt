cmake_minimum_required(VERSION 3.20)
project(stereo_depth VERSION 0.1.0 LANGUAGES CXX)

if(NOT TARGET common)
    message(FATAL_ERROR
        "[stereo_depth] Target 'common' not found.\n"
        "Build stereo_depth as part of the workspace, or add common as a subdirectory first.")
endif()

file(GLOB_RECURSE SOURCES src/*.cpp)

if(SOURCES)
    add_library(stereo_depth STATIC ${SOURCES})
    set(_vis PUBLIC)
else()
    add_library(stereo_depth INTERFACE)
    set(_vis INTERFACE)
    message(STATUS "[stereo_depth] No source files yet — INTERFACE target")
endif()

target_include_directories(stereo_depth ${_vis}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(stereo_depth ${_vis} Eigen3::Eigen common)
target_compile_features(stereo_depth ${_vis} cxx_std_20)

add_library(robotlib::stereo_depth ALIAS stereo_depth)

if(BUILD_TESTING AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/CMakeLists.txt")
    add_subdirectory(tests)
endif()
