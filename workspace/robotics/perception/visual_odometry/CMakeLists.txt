cmake_minimum_required(VERSION 3.20)
project(visual_odometry VERSION 0.1.0 LANGUAGES CXX)

if(NOT TARGET common)
    message(FATAL_ERROR "[visual_odometry] Target 'common' not found. Build as part of the workspace.")
endif()

file(GLOB_RECURSE SOURCES src/*.cpp)

if(SOURCES)
    add_library(visual_odometry STATIC ${SOURCES})
    set(_vis PUBLIC)
else()
    add_library(visual_odometry INTERFACE)
    set(_vis INTERFACE)
    message(STATUS "[visual_odometry] No source files yet — INTERFACE target")
endif()

target_include_directories(visual_odometry ${_vis}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(visual_odometry ${_vis} common)
target_compile_features(visual_odometry ${_vis} cxx_std_20)

add_library(robotlib::visual_odometry ALIAS visual_odometry)

if(BUILD_TESTING AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/CMakeLists.txt")
    enable_testing()
    add_subdirectory(tests)
endif()
