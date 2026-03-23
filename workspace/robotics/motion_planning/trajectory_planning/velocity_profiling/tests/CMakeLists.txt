add_executable(velocity_profiling_tests
    test_trapezoidal_profiler.cpp
)

target_link_libraries(velocity_profiling_tests PRIVATE
    velocity_profiling
    common_logging
    Catch2::Catch2WithMain
)

include(Catch)
catch_discover_tests(velocity_profiling_tests)
