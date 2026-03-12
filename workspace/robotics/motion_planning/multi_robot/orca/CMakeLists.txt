cmake_minimum_required(VERSION 3.20)
project(orca VERSION 0.1.0 LANGUAGES CXX)

if(NOT TARGET common)
    message(FATAL_ERROR "[orca] Target 'common' not found. Build as part of the workspace.")
endif()

file(GLOB_RECURSE SOURCES src/*.cpp)

if(SOURCES)
    add_library(orca STATIC ${SOURCES})
    set(_vis PUBLIC)
else()
    add_library(orca INTERFACE)
    set(_vis INTERFACE)
    message(STATUS "[orca] No source files yet — INTERFACE target")
endif()

target_include_directories(orca ${_vis}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(orca ${_vis} common)   # Eigen via common for velocity obstacles
target_compile_features(orca ${_vis} cxx_std_20)

add_library(robotlib::orca ALIAS orca)

if(BUILD_TESTING AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/CMakeLists.txt")
    add_subdirectory(tests)
endif()
