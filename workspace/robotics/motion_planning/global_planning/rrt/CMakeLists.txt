cmake_minimum_required(VERSION 3.20)
project(rrt VERSION 0.1.0 LANGUAGES CXX)

if(NOT TARGET common)
    message(FATAL_ERROR "[rrt] Target 'common' not found. Build as part of the workspace.")
endif()

file(GLOB_RECURSE SOURCES src/*.cpp)

if(SOURCES)
    add_library(rrt STATIC ${SOURCES})
    set(_vis PUBLIC)
else()
    add_library(rrt INTERFACE)
    set(_vis INTERFACE)
    message(STATUS "[rrt] No source files yet — INTERFACE target")
endif()

target_include_directories(rrt ${_vis}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(rrt ${_vis} common)   # Eigen via common for vector ops
target_compile_features(rrt ${_vis} cxx_std_20)

add_library(robotlib::rrt ALIAS rrt)

if(BUILD_TESTING AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/CMakeLists.txt")
    add_subdirectory(tests)
endif()
