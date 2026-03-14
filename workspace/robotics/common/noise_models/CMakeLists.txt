cmake_minimum_required(VERSION 3.20)
project(noise_models)

add_library(noise_models INTERFACE)
add_library(robotlib::noise_models ALIAS noise_models)
target_include_directories(noise_models INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(noise_models INTERFACE common)
target_compile_features(noise_models INTERFACE cxx_std_20)

if(BUILD_TESTING AND EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/tests/CMakeLists.txt)
    add_subdirectory(tests)
endif()
