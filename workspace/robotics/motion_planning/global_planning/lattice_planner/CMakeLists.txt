cmake_minimum_required(VERSION 3.20)
project(lattice_planner VERSION 0.1.0 LANGUAGES CXX)

if(NOT TARGET common)
    message(FATAL_ERROR "[lattice_planner] Target 'common' not found. Build as part of the workspace.")
endif()

file(GLOB_RECURSE SOURCES src/*.cpp)

if(SOURCES)
    add_library(lattice_planner STATIC ${SOURCES})
    set(_vis PUBLIC)
else()
    add_library(lattice_planner INTERFACE)
    set(_vis INTERFACE)
    message(STATUS "[lattice_planner] No source files yet — INTERFACE target")
endif()

target_include_directories(lattice_planner ${_vis}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(lattice_planner ${_vis} common)
target_compile_features(lattice_planner ${_vis} cxx_std_20)

add_library(robotlib::lattice_planner ALIAS lattice_planner)

if(BUILD_TESTING AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/CMakeLists.txt")
    add_subdirectory(tests)
endif()
