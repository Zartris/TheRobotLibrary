cmake_minimum_required(VERSION 3.20)
project(imu_processing VERSION 0.1.0 LANGUAGES CXX)

if(NOT TARGET common)
    message(FATAL_ERROR
        "[imu_processing] Target 'common' not found.\n"
        "Build imu_processing as part of the workspace, or add common as a subdirectory first.")
endif()

file(GLOB_RECURSE SOURCES src/*.cpp)

if(SOURCES)
    add_library(imu_processing STATIC ${SOURCES})
    set(_vis PUBLIC)
else()
    add_library(imu_processing INTERFACE)
    set(_vis INTERFACE)
    message(STATUS "[imu_processing] No source files yet — INTERFACE target")
endif()

target_include_directories(imu_processing ${_vis}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(imu_processing ${_vis} Eigen3::Eigen common)
target_compile_features(imu_processing ${_vis} cxx_std_20)

add_library(robotlib::imu_processing ALIAS imu_processing)

if(BUILD_TESTING AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/CMakeLists.txt")
    add_subdirectory(tests)
endif()
