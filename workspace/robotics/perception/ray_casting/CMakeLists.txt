cmake_minimum_required(VERSION 3.20)
project(ray_casting VERSION 0.1.0 LANGUAGES CXX)

if(NOT TARGET common)
    message(FATAL_ERROR "[ray_casting] Target 'common' not found. Build as part of the workspace.")
endif()

file(GLOB_RECURSE SOURCES src/*.cpp)

if(SOURCES)
    add_library(ray_casting STATIC ${SOURCES})
    set(_vis PUBLIC)
else()
    add_library(ray_casting INTERFACE)
    set(_vis INTERFACE)
    message(STATUS "[ray_casting] No source files yet — INTERFACE target")
endif()

target_include_directories(ray_casting ${_vis}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(ray_casting ${_vis} common)
target_compile_features(ray_casting ${_vis} cxx_std_20)

add_library(robotlib::ray_casting ALIAS ray_casting)

if(BUILD_TESTING AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/CMakeLists.txt")
    add_subdirectory(tests)
endif()
