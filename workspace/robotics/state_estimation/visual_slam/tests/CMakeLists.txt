include(CTest)

add_executable(visual_slam_tests
    visual_slam_tests.cpp
)
target_link_libraries(visual_slam_tests
    PRIVATE robotlib::visual_slam Catch2::Catch2WithMain
)

include(Catch2/extras/Catch.cmake OPTIONAL)
catch_discover_tests(visual_slam_tests)
