cmake_minimum_required(VERSION 3.20)
project(transforms VERSION 0.1.0 LANGUAGES CXX)

if(NOT TARGET common)
    message(FATAL_ERROR "[transforms] Target 'common' not found. Build as part of the workspace.")
endif()

add_library(transforms INTERFACE)
add_library(robotlib::transforms ALIAS transforms)

target_include_directories(transforms INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(transforms INTERFACE common)   # Eigen via common
target_compile_features(transforms INTERFACE cxx_std_20)

if(BUILD_TESTING AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/CMakeLists.txt")
    enable_testing()
    add_subdirectory(tests)
endif()
