cmake_minimum_required(VERSION 3.20)
project(lane_detection VERSION 0.1.0 LANGUAGES CXX)

if(NOT TARGET common)
    message(FATAL_ERROR "[lane_detection] Target 'common' not found. Build as part of the workspace.")
endif()

file(GLOB_RECURSE SOURCES src/*.cpp)

if(SOURCES)
    add_library(lane_detection STATIC ${SOURCES})
    set(_vis PUBLIC)
else()
    add_library(lane_detection INTERFACE)
    set(_vis INTERFACE)
    message(STATUS "[lane_detection] No source files yet — INTERFACE target")
endif()

target_include_directories(lane_detection ${_vis}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(lane_detection ${_vis} common)
target_compile_features(lane_detection ${_vis} cxx_std_20)

add_library(robotlib::lane_detection ALIAS lane_detection)

if(BUILD_TESTING AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/CMakeLists.txt")
    add_subdirectory(tests)
endif()
