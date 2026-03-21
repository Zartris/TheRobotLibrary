cmake_minimum_required(VERSION 3.20)
project(factor_graph VERSION 0.1.0 LANGUAGES CXX)

if(NOT TARGET common)
    message(FATAL_ERROR "[factor_graph] Target 'common' not found. Build as part of the workspace.")
endif()

file(GLOB_RECURSE SOURCES src/*.cpp)

if(SOURCES)
    add_library(factor_graph STATIC ${SOURCES})
    set(_vis PUBLIC)
else()
    add_library(factor_graph INTERFACE)
    set(_vis INTERFACE)
    message(STATUS "[factor_graph] No source files yet — INTERFACE target")
endif()

target_include_directories(factor_graph ${_vis}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(factor_graph ${_vis} common)
target_compile_features(factor_graph ${_vis} cxx_std_20)

add_library(robotlib::factor_graph ALIAS factor_graph)

if(BUILD_TESTING AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/CMakeLists.txt")
    add_subdirectory(tests)
endif()
