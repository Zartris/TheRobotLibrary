cmake_minimum_required(VERSION 3.20)
project(lidar_slam VERSION 0.1.0 LANGUAGES CXX)

if(NOT TARGET common)
    message(FATAL_ERROR "[lidar_slam] Target 'common' not found. Build as part of the workspace.")
endif()

file(GLOB_RECURSE SOURCES src/*.cpp)

if(SOURCES)
    add_library(lidar_slam STATIC ${SOURCES})
    set(_vis PUBLIC)
else()
    add_library(lidar_slam INTERFACE)
    set(_vis INTERFACE)
    message(STATUS "[lidar_slam] No source files yet — INTERFACE target")
endif()

target_include_directories(lidar_slam ${_vis}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(lidar_slam ${_vis} common)
target_compile_features(lidar_slam ${_vis} cxx_std_20)

add_library(robotlib::lidar_slam ALIAS lidar_slam)

if(BUILD_TESTING AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/CMakeLists.txt")
    add_subdirectory(tests)
endif()
