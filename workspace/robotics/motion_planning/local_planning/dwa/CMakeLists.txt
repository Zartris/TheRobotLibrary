cmake_minimum_required(VERSION 3.20)
project(dwa VERSION 0.1.0 LANGUAGES CXX)

if(NOT TARGET common)
    message(FATAL_ERROR "[dwa] Target 'common' not found. Build as part of the workspace.")
endif()

file(GLOB_RECURSE SOURCES src/*.cpp)

if(SOURCES)
    add_library(dwa STATIC ${SOURCES})
    set(_vis PUBLIC)
else()
    add_library(dwa INTERFACE)
    set(_vis INTERFACE)
    message(STATUS "[dwa] No source files yet — INTERFACE target")
endif()

target_include_directories(dwa ${_vis}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(dwa ${_vis} common)   # Eigen via common for velocity sampling
target_compile_features(dwa ${_vis} cxx_std_20)

add_library(robotlib::dwa ALIAS dwa)

if(BUILD_TESTING AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/CMakeLists.txt")
    add_subdirectory(tests)
endif()
