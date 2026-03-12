cmake_minimum_required(VERSION 3.20)
project(dijkstra VERSION 0.1.0 LANGUAGES CXX)

if(NOT TARGET common)
    message(FATAL_ERROR "[dijkstra] Target 'common' not found. Build as part of the workspace.")
endif()

file(GLOB_RECURSE SOURCES src/*.cpp)

if(SOURCES)
    add_library(dijkstra STATIC ${SOURCES})
    set(_vis PUBLIC)
else()
    add_library(dijkstra INTERFACE)
    set(_vis INTERFACE)
    message(STATUS "[dijkstra] No source files yet — INTERFACE target")
endif()

target_include_directories(dijkstra ${_vis}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(dijkstra ${_vis} common)
target_compile_features(dijkstra ${_vis} cxx_std_20)

add_library(robotlib::dijkstra ALIAS dijkstra)

if(BUILD_TESTING AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/CMakeLists.txt")
    add_subdirectory(tests)
endif()
