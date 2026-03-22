add_executable(ray_casting_tests
    test_ray_caster.cpp
)

target_link_libraries(ray_casting_tests PRIVATE
    ray_casting
    common_logging
    Catch2::Catch2WithMain
)

include(Catch)
catch_discover_tests(ray_casting_tests)
