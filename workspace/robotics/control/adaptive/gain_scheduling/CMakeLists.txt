cmake_minimum_required(VERSION 3.20)
project(gain_scheduling VERSION 0.1.0 LANGUAGES CXX)

if(NOT TARGET common)
    message(FATAL_ERROR "[gain_scheduling] Target 'common' not found. Build as part of the workspace.")
endif()

file(GLOB_RECURSE SOURCES src/*.cpp)

if(SOURCES)
    add_library(gain_scheduling STATIC ${SOURCES})
    set(_vis PUBLIC)
else()
    add_library(gain_scheduling INTERFACE)
    set(_vis INTERFACE)
    message(STATUS "[gain_scheduling] No source files yet — INTERFACE target")
endif()

target_include_directories(gain_scheduling ${_vis}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(gain_scheduling ${_vis} common)
target_compile_features(gain_scheduling ${_vis} cxx_std_20)

add_library(robotlib::gain_scheduling ALIAS gain_scheduling)

if(BUILD_TESTING AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/CMakeLists.txt")
    add_subdirectory(tests)
endif()
