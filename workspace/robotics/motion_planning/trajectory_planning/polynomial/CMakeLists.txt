cmake_minimum_required(VERSION 3.20)
project(polynomial VERSION 0.1.0 LANGUAGES CXX)

if(NOT TARGET common)
    message(FATAL_ERROR
        "[polynomial] Target 'common' not found.\n"
        "Build polynomial as part of the workspace, or add common as a subdirectory first.")
endif()

file(GLOB_RECURSE SOURCES src/*.cpp)

if(SOURCES)
    add_library(polynomial STATIC ${SOURCES})
    set(_vis PUBLIC)
else()
    add_library(polynomial INTERFACE)
    set(_vis INTERFACE)
    message(STATUS "[polynomial] No source files yet — INTERFACE target")
endif()

target_include_directories(polynomial ${_vis}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(polynomial ${_vis} Eigen3::Eigen common)
target_compile_features(polynomial ${_vis} cxx_std_20)

add_library(robotlib::polynomial ALIAS polynomial)

if(BUILD_TESTING AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/CMakeLists.txt")
    add_subdirectory(tests)
endif()
