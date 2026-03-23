add_executable(pid_tests
    test_pid_controller.cpp
)

target_link_libraries(pid_tests PRIVATE
    pid
    common_logging
    common_testing
    Catch2::Catch2WithMain
)

include(Catch)
catch_discover_tests(pid_tests)
