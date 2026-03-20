cmake_minimum_required(VERSION 3.20)
project(depth_camera VERSION 0.1.0 LANGUAGES CXX)

if(NOT TARGET common)
    message(FATAL_ERROR "[depth_camera] Target 'common' not found. Build as part of the workspace.")
endif()

file(GLOB_RECURSE SOURCES src/*.cpp)

if(SOURCES)
    add_library(depth_camera STATIC ${SOURCES})
    set(_vis PUBLIC)
else()
    add_library(depth_camera INTERFACE)
    set(_vis INTERFACE)
    message(STATUS "[depth_camera] No source files yet — INTERFACE target")
endif()

target_include_directories(depth_camera ${_vis}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(depth_camera ${_vis} common)
target_compile_features(depth_camera ${_vis} cxx_std_20)

add_library(robotlib::depth_camera ALIAS depth_camera)

if(BUILD_TESTING AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/CMakeLists.txt")
    add_subdirectory(tests)
endif()
