cmake_minimum_required(VERSION 3.20)
project(feedback_linearization)

add_library(feedback_linearization STATIC
    src/feedback_linearization_controller.cpp
    src/lie_derivative_check.cpp
)
add_library(robotlib::feedback_linearization ALIAS feedback_linearization)

target_include_directories(feedback_linearization
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>
)
target_link_libraries(feedback_linearization PUBLIC Eigen3::Eigen common)
target_compile_features(feedback_linearization PUBLIC cxx_std_20)

if(BUILD_TESTING AND EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/tests/CMakeLists.txt)
    add_subdirectory(tests)
endif()
