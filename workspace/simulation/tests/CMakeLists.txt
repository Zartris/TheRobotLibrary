add_executable(simulation_tests
    test_headless.cpp
)

target_link_libraries(simulation_tests PRIVATE
    simulation_core
    Catch2::Catch2WithMain
    glfw
    OpenGL::GL
)

target_compile_definitions(simulation_tests PRIVATE
    CMAKE_SOURCE_DIR="${CMAKE_SOURCE_DIR}"
)

include(Catch)
catch_discover_tests(simulation_tests)
