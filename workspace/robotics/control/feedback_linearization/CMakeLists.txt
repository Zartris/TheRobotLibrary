cmake_minimum_required(VERSION 3.20)
project(feedback_linearization VERSION 0.1.0 LANGUAGES CXX)

if(NOT TARGET common)
    message(FATAL_ERROR "[feedback_linearization] Target 'common' not found. Build as part of the workspace.")
endif()

file(GLOB_RECURSE SOURCES src/*.cpp)

if(SOURCES)
    add_library(feedback_linearization STATIC ${SOURCES})
    set(_vis PUBLIC)
else()
    add_library(feedback_linearization INTERFACE)
    set(_vis INTERFACE)
    message(STATUS "[feedback_linearization] No source files yet — INTERFACE target")
endif()

target_include_directories(feedback_linearization ${_vis}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(feedback_linearization ${_vis} common)
target_compile_features(feedback_linearization ${_vis} cxx_std_20)

add_library(robotlib::feedback_linearization ALIAS feedback_linearization)

if(BUILD_TESTING AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/CMakeLists.txt")
    add_subdirectory(tests)
endif()
