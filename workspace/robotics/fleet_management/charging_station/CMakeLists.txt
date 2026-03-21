cmake_minimum_required(VERSION 3.20)
project(charging_station VERSION 0.1.0 LANGUAGES CXX)

if(NOT TARGET common)
    message(FATAL_ERROR "[charging_station] Target 'common' not found. Build as part of the workspace.")
endif()
if(NOT TARGET vda5050)
    message(FATAL_ERROR "[charging_station] Target 'vda5050' not found. Build as part of the workspace.")
endif()
if(NOT TARGET battery_management)
    message(FATAL_ERROR "[charging_station] Target 'battery_management' not found. Build as part of the workspace.")
endif()

file(GLOB_RECURSE SOURCES src/*.cpp)

if(SOURCES)
    add_library(charging_station STATIC ${SOURCES})
    set(_vis PUBLIC)
else()
    add_library(charging_station INTERFACE)
    set(_vis INTERFACE)
    message(STATUS "[charging_station] No source files yet — INTERFACE target")
endif()

target_include_directories(charging_station ${_vis}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(charging_station ${_vis} common vda5050 battery_management)
target_compile_features(charging_station ${_vis} cxx_std_20)

add_library(robotlib::charging_station ALIAS charging_station)

if(BUILD_TESTING AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/CMakeLists.txt")
    add_subdirectory(tests)
endif()
