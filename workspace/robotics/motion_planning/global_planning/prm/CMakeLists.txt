cmake_minimum_required(VERSION 3.20)
project(prm VERSION 0.1.0 LANGUAGES CXX)

if(NOT TARGET common)
    message(FATAL_ERROR
        "[prm] Target 'common' not found.\n"
        "Build prm as part of the workspace, or add common as a subdirectory first.")
endif()

file(GLOB_RECURSE SOURCES src/*.cpp)

if(SOURCES)
    add_library(prm STATIC ${SOURCES})
    set(_vis PUBLIC)
else()
    add_library(prm INTERFACE)
    set(_vis INTERFACE)
    message(STATUS "[prm] No source files yet — INTERFACE target")
endif()

target_include_directories(prm ${_vis}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(prm ${_vis} Eigen3::Eigen common)
target_compile_features(prm ${_vis} cxx_std_20)

add_library(robotlib::prm ALIAS prm)

if(BUILD_TESTING AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/CMakeLists.txt")
    add_subdirectory(tests)
endif()
