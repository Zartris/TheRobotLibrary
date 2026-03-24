add_executable(mpc_tests test_mpc_controller.cpp)
target_link_libraries(mpc_tests PRIVATE
    mpc
    common_testing
    Catch2::Catch2WithMain
)

include(Catch)
catch_discover_tests(mpc_tests)
