cmake_minimum_required(VERSION 3.20)
project(potential_field VERSION 0.1.0 LANGUAGES CXX)

if(NOT TARGET common)
    message(FATAL_ERROR "[potential_field] Target 'common' not found. Build as part of the workspace.")
endif()

file(GLOB_RECURSE SOURCES src/*.cpp)

if(SOURCES)
    add_library(potential_field STATIC ${SOURCES})
    set(_vis PUBLIC)
else()
    add_library(potential_field INTERFACE)
    set(_vis INTERFACE)
    message(STATUS "[potential_field] No source files yet — INTERFACE target")
endif()

target_include_directories(potential_field ${_vis}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(potential_field ${_vis} common)
target_compile_features(potential_field ${_vis} cxx_std_20)

add_library(robotlib::potential_field ALIAS potential_field)

if(BUILD_TESTING AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/CMakeLists.txt")
    add_subdirectory(tests)
endif()
