cmake_minimum_required(VERSION 3.20)
project(potential_field)

add_library(potential_field STATIC
    src/potential_field_planner.cpp
)
add_library(robotlib::potential_field ALIAS potential_field)

target_include_directories(potential_field
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>
)
target_link_libraries(potential_field PUBLIC Eigen3::Eigen common)
target_compile_features(potential_field PUBLIC cxx_std_20)

if(BUILD_TESTING AND EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/tests/CMakeLists.txt)
    add_subdirectory(tests)
endif()
