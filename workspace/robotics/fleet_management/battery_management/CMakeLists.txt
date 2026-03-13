cmake_minimum_required(VERSION 3.20)
project(battery_management VERSION 0.1.0 LANGUAGES CXX)

if(NOT TARGET common)
    message(FATAL_ERROR "[battery_management] Target 'common' not found. Build as part of the workspace.")
endif()
if(NOT TARGET vda5050)
    message(FATAL_ERROR "[battery_management] Target 'vda5050' not found. Build as part of the workspace.")
endif()
if(NOT TARGET fleet_monitor)
    message(FATAL_ERROR "[battery_management] Target 'fleet_monitor' not found. Build as part of the workspace.")
endif()

file(GLOB_RECURSE SOURCES src/*.cpp)

if(SOURCES)
    add_library(battery_management STATIC ${SOURCES})
    set(_vis PUBLIC)
else()
    add_library(battery_management INTERFACE)
    set(_vis INTERFACE)
    message(STATUS "[battery_management] No source files yet — INTERFACE target")
endif()

target_include_directories(battery_management ${_vis}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(battery_management ${_vis} common vda5050 fleet_monitor)
target_compile_features(battery_management ${_vis} cxx_std_20)

add_library(robotlib::battery_management ALIAS battery_management)

if(BUILD_TESTING AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/CMakeLists.txt")
    add_subdirectory(tests)
endif()
