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

add_executable(integration_tests
    test_integration.cpp
)

target_link_libraries(integration_tests PRIVATE
    simulation_core
    Catch2::Catch2WithMain
    glfw
    OpenGL::GL
)

target_compile_definitions(integration_tests PRIVATE
    CMAKE_SOURCE_DIR="${CMAKE_SOURCE_DIR}"
)

catch_discover_tests(integration_tests)

add_executable(m3_integration_tests
    test_m3_integration.cpp
)
target_link_libraries(m3_integration_tests PRIVATE
    simulation_core
    Catch2::Catch2WithMain
    glfw
    OpenGL::GL
)
target_compile_definitions(m3_integration_tests PRIVATE
    CMAKE_SOURCE_DIR="${CMAKE_SOURCE_DIR}"
)
catch_discover_tests(m3_integration_tests)
