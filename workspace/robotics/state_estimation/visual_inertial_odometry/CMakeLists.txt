cmake_minimum_required(VERSION 3.20)
project(visual_inertial_odometry VERSION 0.1.0 LANGUAGES CXX)

if(NOT TARGET common)
    message(FATAL_ERROR
        "[visual_inertial_odometry] Target 'common' not found.\n"
        "Build visual_inertial_odometry as part of the workspace, or add common as a subdirectory first.")
endif()

file(GLOB_RECURSE SOURCES src/*.cpp)

if(SOURCES)
    add_library(visual_inertial_odometry STATIC ${SOURCES})
    set(_vis PUBLIC)
else()
    add_library(visual_inertial_odometry INTERFACE)
    set(_vis INTERFACE)
    message(STATUS "[visual_inertial_odometry] No source files yet — INTERFACE target")
endif()

target_include_directories(visual_inertial_odometry ${_vis}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(visual_inertial_odometry ${_vis} Eigen3::Eigen common)
target_compile_features(visual_inertial_odometry ${_vis} cxx_std_20)

add_library(robotlib::visual_inertial_odometry ALIAS visual_inertial_odometry)

if(BUILD_TESTING AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/CMakeLists.txt")
    add_subdirectory(tests)
endif()
