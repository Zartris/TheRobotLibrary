cmake_minimum_required(VERSION 3.20)
project(place_recognition VERSION 0.1.0 LANGUAGES CXX)

if(NOT TARGET common)
    message(FATAL_ERROR
        "[place_recognition] Target 'common' not found.\n"
        "Build place_recognition as part of the workspace, or add common as a subdirectory first.")
endif()

file(GLOB_RECURSE SOURCES src/*.cpp)

if(SOURCES)
    add_library(place_recognition STATIC ${SOURCES})
    set(_vis PUBLIC)
else()
    add_library(place_recognition INTERFACE)
    set(_vis INTERFACE)
    message(STATUS "[place_recognition] No source files yet — INTERFACE target")
endif()

target_include_directories(place_recognition ${_vis}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(place_recognition ${_vis} Eigen3::Eigen common)
target_compile_features(place_recognition ${_vis} cxx_std_20)

add_library(robotlib::place_recognition ALIAS place_recognition)

if(BUILD_TESTING AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/CMakeLists.txt")
    add_subdirectory(tests)
endif()
