cmake_minimum_required(VERSION 3.20)
project(particle_filter VERSION 0.1.0 LANGUAGES CXX)

if(NOT TARGET common)
    message(FATAL_ERROR "[particle_filter] Target 'common' not found. Build as part of the workspace.")
endif()

file(GLOB_RECURSE SOURCES src/*.cpp)

if(SOURCES)
    add_library(particle_filter STATIC ${SOURCES})
    set(_vis PUBLIC)
else()
    add_library(particle_filter INTERFACE)
    set(_vis INTERFACE)
    message(STATUS "[particle_filter] No source files yet — INTERFACE target")
endif()

target_include_directories(particle_filter ${_vis}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(particle_filter ${_vis} common)
target_compile_features(particle_filter ${_vis} cxx_std_20)

add_library(robotlib::particle_filter ALIAS particle_filter)

if(BUILD_TESTING AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/CMakeLists.txt")
    add_subdirectory(tests)
endif()
