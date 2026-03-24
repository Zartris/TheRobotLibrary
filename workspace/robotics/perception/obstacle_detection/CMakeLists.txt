cmake_minimum_required(VERSION 3.20)
project(obstacle_detection VERSION 0.1.0 LANGUAGES CXX)

if(NOT TARGET common)
    message(FATAL_ERROR "[obstacle_detection] Target 'common' not found. Build as part of the workspace.")
endif()

file(GLOB_RECURSE SOURCES src/*.cpp)

if(SOURCES)
    add_library(obstacle_detection STATIC ${SOURCES})
    set(_vis PUBLIC)
else()
    add_library(obstacle_detection INTERFACE)
    set(_vis INTERFACE)
    message(STATUS "[obstacle_detection] No source files yet — INTERFACE target")
endif()

target_include_directories(obstacle_detection ${_vis}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(obstacle_detection ${_vis} common common_logging)
target_compile_features(obstacle_detection ${_vis} cxx_std_20)

add_library(robotlib::obstacle_detection ALIAS obstacle_detection)

if(BUILD_TESTING AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/CMakeLists.txt")
    add_subdirectory(tests)
endif()
