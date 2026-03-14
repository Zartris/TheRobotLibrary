cmake_minimum_required(VERSION 3.20)
project(object_detection_3d)

add_library(object_detection_3d STATIC
    src/object_detector_3d.cpp
)
add_library(robotlib::object_detection_3d ALIAS object_detection_3d)

target_include_directories(object_detection_3d
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>
)
target_link_libraries(object_detection_3d PUBLIC Eigen3::Eigen common)
target_compile_features(object_detection_3d PUBLIC cxx_std_20)

if(BUILD_TESTING AND EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/tests/CMakeLists.txt)
    add_subdirectory(tests)
endif()
