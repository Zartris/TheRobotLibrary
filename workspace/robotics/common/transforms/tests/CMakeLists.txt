include(CTest)

add_executable(transforms_tests
    transforms_tests.cpp
)
target_link_libraries(transforms_tests
    PRIVATE robotlib::transforms Catch2::Catch2WithMain
)

include(Catch2/extras/Catch.cmake OPTIONAL)
catch_discover_tests(transforms_tests)
