cmake_minimum_required(VERSION 3.20)
project(mppi VERSION 0.1.0 LANGUAGES CXX)

if(NOT TARGET common)
    message(FATAL_ERROR "[mppi] Target 'common' not found. Build as part of the workspace.")
endif()

file(GLOB_RECURSE SOURCES src/*.cpp)

if(SOURCES)
    add_library(mppi STATIC ${SOURCES})
    set(_vis PUBLIC)
else()
    add_library(mppi INTERFACE)
    set(_vis INTERFACE)
    message(STATUS "[mppi] No source files yet — INTERFACE target")
endif()

target_include_directories(mppi ${_vis}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(mppi ${_vis} common)
target_compile_features(mppi ${_vis} cxx_std_20)

add_library(robotlib::mppi ALIAS mppi)

if(BUILD_TESTING AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/CMakeLists.txt")
    add_subdirectory(tests)
endif()
