cmake_minimum_required(VERSION 3.20)
project(lqg VERSION 0.1.0 LANGUAGES CXX)

if(NOT TARGET common)
    message(FATAL_ERROR "[lqg] Target 'common' not found. Build as part of the workspace.")
endif()

file(GLOB_RECURSE SOURCES src/*.cpp)

if(SOURCES)
    add_library(lqg STATIC ${SOURCES})
    set(_vis PUBLIC)
else()
    add_library(lqg INTERFACE)
    set(_vis INTERFACE)
    message(STATUS "[lqg] No source files yet — INTERFACE target")
endif()

target_include_directories(lqg ${_vis}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(lqg ${_vis} common lqr ekf)
target_compile_features(lqg ${_vis} cxx_std_20)

add_library(robotlib::lqg ALIAS lqg)

if(BUILD_TESTING AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/CMakeLists.txt")
    add_subdirectory(tests)
endif()
