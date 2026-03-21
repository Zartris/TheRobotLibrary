cmake_minimum_required(VERSION 3.20)
project(informed_rrt_star VERSION 0.1.0 LANGUAGES CXX)

if(NOT TARGET common)
    message(FATAL_ERROR "[informed_rrt_star] Target 'common' not found. Build as part of the workspace.")
endif()

file(GLOB_RECURSE SOURCES src/*.cpp)

if(SOURCES)
    add_library(informed_rrt_star STATIC ${SOURCES})
    set(_vis PUBLIC)
else()
    add_library(informed_rrt_star INTERFACE)
    set(_vis INTERFACE)
    message(STATUS "[informed_rrt_star] No source files yet — INTERFACE target")
endif()

target_include_directories(informed_rrt_star ${_vis}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(informed_rrt_star ${_vis} common)
target_compile_features(informed_rrt_star ${_vis} cxx_std_20)

add_library(robotlib::informed_rrt_star ALIAS informed_rrt_star)

if(BUILD_TESTING AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/CMakeLists.txt")
    add_subdirectory(tests)
endif()
