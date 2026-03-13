cmake_minimum_required(VERSION 3.20)
project(fleet_monitor VERSION 0.1.0 LANGUAGES CXX)

if(NOT TARGET common)
    message(FATAL_ERROR "[fleet_monitor] Target 'common' not found. Build as part of the workspace.")
endif()
if(NOT TARGET vda5050)
    message(FATAL_ERROR "[fleet_monitor] Target 'vda5050' not found. Build as part of the workspace.")
endif()

file(GLOB_RECURSE SOURCES src/*.cpp)

if(SOURCES)
    add_library(fleet_monitor STATIC ${SOURCES})
    set(_vis PUBLIC)
else()
    add_library(fleet_monitor INTERFACE)
    set(_vis INTERFACE)
    message(STATUS "[fleet_monitor] No source files yet — INTERFACE target")
endif()

target_include_directories(fleet_monitor ${_vis}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(fleet_monitor ${_vis} common vda5050)
target_compile_features(fleet_monitor ${_vis} cxx_std_20)

add_library(robotlib::fleet_monitor ALIAS fleet_monitor)

if(BUILD_TESTING AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/CMakeLists.txt")
    add_subdirectory(tests)
endif()
