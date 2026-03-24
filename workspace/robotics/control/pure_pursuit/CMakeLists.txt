cmake_minimum_required(VERSION 3.20)
project(pure_pursuit VERSION 0.1.0 LANGUAGES CXX)

if(NOT TARGET common)
    message(FATAL_ERROR "[pure_pursuit] Target 'common' not found. Build as part of the workspace.")
endif()

file(GLOB_RECURSE SOURCES src/*.cpp)

if(SOURCES)
    add_library(pure_pursuit STATIC ${SOURCES})
    set(_vis PUBLIC)
else()
    add_library(pure_pursuit INTERFACE)
    set(_vis INTERFACE)
    message(STATUS "[pure_pursuit] No source files yet — INTERFACE target")
endif()

target_include_directories(pure_pursuit ${_vis}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(pure_pursuit ${_vis} common common_logging)
target_compile_features(pure_pursuit ${_vis} cxx_std_20)

add_library(robotlib::pure_pursuit ALIAS pure_pursuit)

if(BUILD_TESTING AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/CMakeLists.txt")
    add_subdirectory(tests)
endif()
