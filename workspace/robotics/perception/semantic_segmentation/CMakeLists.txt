cmake_minimum_required(VERSION 3.20)
project(semantic_segmentation)

add_library(semantic_segmentation STATIC
    src/stub_semantic_segmenter.cpp
    src/plugin_semantic_segmenter.cpp
)
add_library(robotlib::semantic_segmentation ALIAS semantic_segmentation)

target_include_directories(semantic_segmentation
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>
)
target_link_libraries(semantic_segmentation PUBLIC Eigen3::Eigen common)
target_compile_features(semantic_segmentation PUBLIC cxx_std_20)

if(BUILD_TESTING AND EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/tests/CMakeLists.txt)
    add_subdirectory(tests)
endif()
