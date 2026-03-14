cmake_minimum_required(VERSION 3.20)
project(mppi)

add_library(mppi STATIC
    src/mppi_controller.cpp
)
add_library(robotlib::mppi ALIAS mppi)

target_include_directories(mppi
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>
)
target_link_libraries(mppi PUBLIC Eigen3::Eigen common)
target_compile_features(mppi PUBLIC cxx_std_20)

if(BUILD_TESTING AND EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/tests/CMakeLists.txt)
    add_subdirectory(tests)
endif()
