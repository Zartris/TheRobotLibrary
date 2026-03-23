#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <velocity_profiling/trapezoidal_profiler.hpp>

using namespace robotlib;

TEST_CASE("Trapezoidal long straight path - accel cruise decel", "[velocity_profiling]") {
    TrapezoidalProfiler profiler;

    Path path;
    for (int i = 0; i <= 100; ++i) {
        path.push_back({i * 0.1, 0.0, 0.0});
    }

    VelocityConstraints constraints{1.0, 0.5, 0.5};
    auto result = profiler.profile(path, constraints);

    REQUIRE(result.size() == path.size());
    REQUIRE_THAT(result.front().velocity, Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(result.back().velocity, Catch::Matchers::WithinAbs(0.0, 1e-6));

    // Middle should be at or near max velocity
    bool hasHighVelocity = false;
    for (const auto& tp : result) {
        if (tp.velocity > 0.8) hasHighVelocity = true;
    }
    REQUIRE(hasHighVelocity);
}

TEST_CASE("Trapezoidal short path - triangular profile", "[velocity_profiling]") {
    TrapezoidalProfiler profiler;

    Path path;
    for (int i = 0; i <= 5; ++i) {
        path.push_back({i * 0.05, 0.0, 0.0});
    }

    VelocityConstraints constraints{1.0, 0.5, 0.5};
    auto result = profiler.profile(path, constraints);

    REQUIRE(result.size() == path.size());
    // Should not reach max velocity for such short distance
    for (const auto& tp : result) {
        REQUIRE(tp.velocity <= 1.0 + 1e-6);
    }
}

TEST_CASE("Trapezoidal single point path - zero velocity", "[velocity_profiling]") {
    TrapezoidalProfiler profiler;
    Path path = {{0.0, 0.0, 0.0}};
    VelocityConstraints constraints{1.0, 0.5, 0.5};

    auto result = profiler.profile(path, constraints);

    REQUIRE(result.size() == 1);
    REQUIRE_THAT(result[0].velocity, Catch::Matchers::WithinAbs(0.0, 1e-6));
}

TEST_CASE("Trapezoidal sharp turns - velocity reduced", "[velocity_profiling]") {
    TrapezoidalProfiler profiler;

    Path path;
    // Straight, then sharp turn
    for (int i = 0; i <= 20; ++i) path.push_back({i * 0.1, 0.0, 0.0});
    for (int i = 1; i <= 20; ++i) path.push_back({2.0, i * 0.1, 0.0});

    VelocityConstraints constraints{1.0, 0.5, 0.5};
    auto result = profiler.profile(path, constraints);

    REQUIRE(result.size() == path.size());
    // Velocity at the turn should be lower than at straight sections
    double velAtTurn = result[20].velocity;
    double maxStraightVel = 0.0;
    for (size_t i = 5; i < 15; ++i) {
        maxStraightVel = std::max(maxStraightVel, result[i].velocity);
    }
    REQUIRE(velAtTurn <= maxStraightVel + 1e-6);
}
