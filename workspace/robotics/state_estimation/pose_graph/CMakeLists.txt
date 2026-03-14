cmake_minimum_required(VERSION 3.20)
project(pose_graph VERSION 0.1.0 LANGUAGES CXX)

if(NOT TARGET common)
    message(FATAL_ERROR
        "[pose_graph] Target 'common' not found.\n"
        "Build pose_graph as part of the workspace, or add common as a subdirectory first.")
endif()

file(GLOB_RECURSE SOURCES src/*.cpp)

if(SOURCES)
    add_library(pose_graph STATIC ${SOURCES})
    set(_vis PUBLIC)
else()
    add_library(pose_graph INTERFACE)
    set(_vis INTERFACE)
    message(STATUS "[pose_graph] No source files yet — INTERFACE target")
endif()

target_include_directories(pose_graph ${_vis}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(pose_graph ${_vis} Eigen3::Eigen common)
target_compile_features(pose_graph ${_vis} cxx_std_20)

add_library(robotlib::pose_graph ALIAS pose_graph)

if(BUILD_TESTING AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/CMakeLists.txt")
    add_subdirectory(tests)
endif()
