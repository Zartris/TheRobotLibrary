cmake_minimum_required(VERSION 3.20)
project(lane_detection)

add_library(lane_detection STATIC
    src/lane_detector.cpp
)
add_library(robotlib::lane_detection ALIAS lane_detection)

target_include_directories(lane_detection
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>
)
target_link_libraries(lane_detection PUBLIC Eigen3::Eigen common)
target_compile_features(lane_detection PUBLIC cxx_std_20)

if(BUILD_TESTING AND EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/tests/CMakeLists.txt)
    add_subdirectory(tests)
endif()
