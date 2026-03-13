cmake_minimum_required(VERSION 3.20)
project(feature_extraction VERSION 0.1.0 LANGUAGES CXX)

if(NOT TARGET common)
    message(FATAL_ERROR "[feature_extraction] Target 'common' not found. Build as part of the workspace.")
endif()

file(GLOB_RECURSE SOURCES src/*.cpp)

if(SOURCES)
    add_library(feature_extraction STATIC ${SOURCES})
    set(_vis PUBLIC)
else()
    add_library(feature_extraction INTERFACE)
    set(_vis INTERFACE)
    message(STATUS "[feature_extraction] No source files yet — INTERFACE target")
endif()

target_include_directories(feature_extraction ${_vis}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(feature_extraction ${_vis} common)
target_compile_features(feature_extraction ${_vis} cxx_std_20)

add_library(robotlib::feature_extraction ALIAS feature_extraction)

if(BUILD_TESTING AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/CMakeLists.txt")
    enable_testing()
    add_subdirectory(tests)
endif()
