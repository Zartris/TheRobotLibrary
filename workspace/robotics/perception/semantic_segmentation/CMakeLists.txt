cmake_minimum_required(VERSION 3.20)
project(semantic_segmentation VERSION 0.1.0 LANGUAGES CXX)

if(NOT TARGET common)
    message(FATAL_ERROR "[semantic_segmentation] Target 'common' not found. Build as part of the workspace.")
endif()

file(GLOB_RECURSE SOURCES src/*.cpp)

if(SOURCES)
    add_library(semantic_segmentation STATIC ${SOURCES})
    set(_vis PUBLIC)
else()
    add_library(semantic_segmentation INTERFACE)
    set(_vis INTERFACE)
    message(STATUS "[semantic_segmentation] No source files yet — INTERFACE target")
endif()

target_include_directories(semantic_segmentation ${_vis}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(semantic_segmentation ${_vis} common)
target_compile_features(semantic_segmentation ${_vis} cxx_std_20)

add_library(robotlib::semantic_segmentation ALIAS semantic_segmentation)

if(BUILD_TESTING AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/CMakeLists.txt")
    add_subdirectory(tests)
endif()
