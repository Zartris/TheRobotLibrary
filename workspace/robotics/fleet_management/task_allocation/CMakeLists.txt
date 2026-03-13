cmake_minimum_required(VERSION 3.20)
project(task_allocation VERSION 0.1.0 LANGUAGES CXX)

if(NOT TARGET common)
    message(FATAL_ERROR "[task_allocation] Target 'common' not found. Build as part of the workspace.")
endif()
if(NOT TARGET vda5050)
    message(FATAL_ERROR "[task_allocation] Target 'vda5050' not found. Build as part of the workspace.")
endif()
if(NOT TARGET fleet_monitor)
    message(FATAL_ERROR "[task_allocation] Target 'fleet_monitor' not found. Build as part of the workspace.")
endif()

file(GLOB_RECURSE SOURCES src/*.cpp)

if(SOURCES)
    add_library(task_allocation STATIC ${SOURCES})
    set(_vis PUBLIC)
else()
    add_library(task_allocation INTERFACE)
    set(_vis INTERFACE)
    message(STATUS "[task_allocation] No source files yet — INTERFACE target")
endif()

target_include_directories(task_allocation ${_vis}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(task_allocation ${_vis} common vda5050 fleet_monitor)
target_compile_features(task_allocation ${_vis} cxx_std_20)

add_library(robotlib::task_allocation ALIAS task_allocation)

if(BUILD_TESTING AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/CMakeLists.txt")
    add_subdirectory(tests)
endif()
