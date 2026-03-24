add_executable(obstacle_detection_tests test_obstacle_detector.cpp)
target_link_libraries(obstacle_detection_tests PRIVATE
    obstacle_detection
    common_logging
    common_testing
    Catch2::Catch2WithMain
)
include(Catch)
catch_discover_tests(obstacle_detection_tests)
