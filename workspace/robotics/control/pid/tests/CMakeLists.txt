add_executable(pid_tests
    test_pid_controller.cpp
)

target_link_libraries(pid_tests PRIVATE
    pid
    common_logging
    Catch2::Catch2WithMain
)

include(Catch)
catch_discover_tests(pid_tests)
