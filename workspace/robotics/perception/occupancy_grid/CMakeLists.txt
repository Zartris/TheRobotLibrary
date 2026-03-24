cmake_minimum_required(VERSION 3.20)
project(occupancy_grid VERSION 0.1.0 LANGUAGES CXX)

if(NOT TARGET common)
    message(FATAL_ERROR "[occupancy_grid] Target 'common' not found. Build as part of the workspace.")
endif()

file(GLOB_RECURSE SOURCES src/*.cpp)

if(SOURCES)
    add_library(occupancy_grid STATIC ${SOURCES})
    set(_vis PUBLIC)
else()
    add_library(occupancy_grid INTERFACE)
    set(_vis INTERFACE)
    message(STATUS "[occupancy_grid] No source files yet — INTERFACE target")
endif()

target_include_directories(occupancy_grid ${_vis}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(occupancy_grid ${_vis} common common_logging)
target_compile_features(occupancy_grid ${_vis} cxx_std_20)

add_library(robotlib::occupancy_grid ALIAS occupancy_grid)

if(BUILD_TESTING AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/CMakeLists.txt")
    add_subdirectory(tests)
endif()
