cmake_minimum_required(VERSION 3.20)
project(object_detection_3d VERSION 0.1.0 LANGUAGES CXX)

if(NOT TARGET common)
    message(FATAL_ERROR "[object_detection_3d] Target 'common' not found. Build as part of the workspace.")
endif()

file(GLOB_RECURSE SOURCES src/*.cpp)

if(SOURCES)
    add_library(object_detection_3d STATIC ${SOURCES})
    set(_vis PUBLIC)
else()
    add_library(object_detection_3d INTERFACE)
    set(_vis INTERFACE)
    message(STATUS "[object_detection_3d] No source files yet — INTERFACE target")
endif()

target_include_directories(object_detection_3d ${_vis}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(object_detection_3d ${_vis} common)
target_compile_features(object_detection_3d ${_vis} cxx_std_20)

add_library(robotlib::object_detection_3d ALIAS object_detection_3d)

if(BUILD_TESTING AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/CMakeLists.txt")
    add_subdirectory(tests)
endif()
