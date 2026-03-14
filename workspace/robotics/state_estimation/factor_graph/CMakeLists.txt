cmake_minimum_required(VERSION 3.20)
project(factor_graph)

add_library(factor_graph STATIC
    src/factor_graph.cpp
)
add_library(robotlib::factor_graph ALIAS factor_graph)

target_include_directories(factor_graph
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>
)
target_link_libraries(factor_graph PUBLIC Eigen3::Eigen common)
target_compile_features(factor_graph PUBLIC cxx_std_20)

if(BUILD_TESTING AND EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/tests/CMakeLists.txt)
    add_subdirectory(tests)
endif()
