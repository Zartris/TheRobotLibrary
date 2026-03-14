cmake_minimum_required(VERSION 3.20)
project(depth_camera)

add_library(depth_camera STATIC
    src/depth_image_processor.cpp
)
add_library(robotlib::depth_camera ALIAS depth_camera)

target_include_directories(depth_camera
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>
)
target_link_libraries(depth_camera PUBLIC Eigen3::Eigen common)
target_compile_features(depth_camera PUBLIC cxx_std_20)

if(BUILD_TESTING AND EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/tests/CMakeLists.txt)
    add_subdirectory(tests)
endif()
