add_executable(pure_pursuit_tests test_pure_pursuit_controller.cpp)
target_link_libraries(pure_pursuit_tests PRIVATE
    pure_pursuit
    common_logging
    common_testing
    Catch2::Catch2WithMain
)

include(Catch)
catch_discover_tests(pure_pursuit_tests)
