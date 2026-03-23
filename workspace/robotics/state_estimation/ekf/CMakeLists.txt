cmake_minimum_required(VERSION 3.20)
project(ekf VERSION 0.1.0 LANGUAGES CXX)

if(NOT TARGET common)
    message(FATAL_ERROR "[ekf] Target 'common' not found. Build as part of the workspace.")
endif()

file(GLOB_RECURSE SOURCES src/*.cpp)

if(SOURCES)
    add_library(ekf STATIC ${SOURCES})
    set(_vis PUBLIC)
else()
    add_library(ekf INTERFACE)
    set(_vis INTERFACE)
    message(STATUS "[ekf] No source files yet — INTERFACE target")
endif()

target_include_directories(ekf ${_vis}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(ekf ${_vis} common common_logging)   # Eigen via common
target_compile_features(ekf ${_vis} cxx_std_20)

add_library(robotlib::ekf ALIAS ekf)

if(BUILD_TESTING AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/CMakeLists.txt")
    add_subdirectory(tests)
endif()
