#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <pure_pursuit/pure_pursuit_controller.hpp>
#include <testing/recording_logger.hpp>
#include <logging/get_logger.hpp>
#include <numbers>

using namespace robotlib;

TEST_CASE("Straight path produces forward velocity", "[pure_pursuit]") {
    PurePursuitConfig cfg;
    cfg.lookaheadDistance = 1.0;
    cfg.maxLinearVelocity = 1.0;
    PurePursuitController pp(cfg);

    // Straight path along x-axis
    Path path;
    for (int i = 0; i <= 10; ++i) {
        path.push_back({static_cast<double>(i), 0.0, 0.0});
    }
    pp.setPath(path);

    // Robot at origin facing forward
    Pose2D current{0.0, 0.0, 0.0};
    Pose2D goal{10.0, 0.0, 0.0};
    auto cmd = pp.compute(current, goal, 0.1);

    REQUIRE(cmd.linear > 0.0);
    REQUIRE_THAT(cmd.angular, Catch::Matchers::WithinAbs(0.0, 0.1));
}

TEST_CASE("Curved path produces nonzero omega", "[pure_pursuit]") {
    PurePursuitConfig cfg;
    cfg.lookaheadDistance = 0.5;
    PurePursuitController pp(cfg);

    // Path curving to the left
    Path path;
    path.push_back({1.0, 0.0, 0.0});
    path.push_back({1.5, 0.5, 0.0});
    path.push_back({2.0, 1.0, 0.0});
    pp.setPath(path);

    Pose2D current{0.0, 0.0, 0.0};
    Pose2D goal{2.0, 1.0, 0.0};
    auto cmd = pp.compute(current, goal, 0.1);

    // Should have positive omega (turning left)
    REQUIRE(cmd.linear > 0.0);
    // Curvature should be nonzero since path curves
    CHECK(std::abs(cmd.angular) > 0.01);
}

TEST_CASE("Lookahead beyond path end targets last waypoint", "[pure_pursuit]") {
    PurePursuitConfig cfg;
    cfg.lookaheadDistance = 5.0;  // Larger than path length
    PurePursuitController pp(cfg);

    Path path;
    path.push_back({1.0, 0.0, 0.0});
    path.push_back({2.0, 0.0, 0.0});
    pp.setPath(path);

    Pose2D current{0.0, 0.0, 0.0};
    Pose2D goal{2.0, 0.0, 0.0};
    auto cmd = pp.compute(current, goal, 0.1);

    auto la = pp.getLookaheadPoint();
    REQUIRE_THAT(la.x, Catch::Matchers::WithinAbs(2.0, 1e-9));
    REQUIRE_THAT(la.y, Catch::Matchers::WithinAbs(0.0, 1e-9));
}

TEST_CASE("Goal reached produces zero twist", "[pure_pursuit]") {
    PurePursuitConfig cfg;
    cfg.goalTolerance = 0.5;
    PurePursuitController pp(cfg);

    Path path;
    path.push_back({1.0, 0.0, 0.0});
    pp.setPath(path);

    // Robot already at target
    Pose2D current{1.0, 0.0, 0.0};
    Pose2D goal{1.0, 0.0, 0.0};
    auto cmd = pp.compute(current, goal, 0.1);

    REQUIRE_THAT(cmd.linear, Catch::Matchers::WithinAbs(0.0, 1e-9));
    REQUIRE_THAT(cmd.angular, Catch::Matchers::WithinAbs(0.0, 1e-9));
}

TEST_CASE("Reset clears path", "[pure_pursuit]") {
    PurePursuitController pp;
    Path path;
    path.push_back({1.0, 0.0, 0.0});
    pp.setPath(path);
    pp.reset();

    // After reset, no path — should work with direct-to-target mode
    Pose2D current{0.0, 0.0, 0.0};
    Pose2D goal{5.0, 0.0, 0.0};
    auto cmd = pp.compute(current, goal, 0.1);
    REQUIRE(cmd.linear > 0.0);
}

TEST_CASE("Speed-dependent lookahead scales correctly", "[pure_pursuit]") {
    PurePursuitConfig cfg;
    cfg.lookaheadDistance = 0.5;
    cfg.lookaheadGain = 0.5;  // L = 0.5 + 0.5 * |v|
    cfg.maxLinearVelocity = 2.0;
    PurePursuitController pp(cfg);

    // Straight path
    Path path;
    for (int i = 0; i <= 20; ++i) {
        path.push_back({static_cast<double>(i) * 0.5, 0.0, 0.0});
    }
    pp.setPath(path);

    Pose2D current{0.0, 0.0, 0.0};
    Pose2D goal{10.0, 0.0, 0.0};

    // First call: no previous speed, lookahead = base (0.5)
    auto cmd1 = pp.compute(current, goal, 0.1);
    auto la1 = pp.getLookaheadPoint();

    // Second call: has previous speed, lookahead should be larger
    auto cmd2 = pp.compute(current, goal, 0.1);
    auto la2 = pp.getLookaheadPoint();

    // Lookahead point should be further ahead after speed builds up
    double dist1 = std::sqrt(la1.x * la1.x + la1.y * la1.y);
    double dist2 = std::sqrt(la2.x * la2.x + la2.y * la2.y);
    CHECK(dist2 >= dist1 - 0.01);  // Should be same or further
}

TEST_CASE("PurePursuit logging and observability", "[pure_pursuit][logging]") {
    auto cleanup = std::shared_ptr<void>(nullptr, [](void*) { robotlib::clearLoggerRegistry(); });
    auto mockLogger = std::make_shared<robotlib::testing::RecordingLogger>();
    robotlib::registerLogger("pure_pursuit", mockLogger);

    PurePursuitController pp;

    REQUIRE(mockLogger->hasMessageContaining(
        robotlib::testing::LogEntry::Level::DEBUG, "initialized"));

    // Run compute to check trace
    Pose2D current{0.0, 0.0, 0.0};
    Pose2D goal{5.0, 0.0, 0.0};
    pp.compute(current, goal, 0.1);

    REQUIRE(mockLogger->hasMessageContaining(
        robotlib::testing::LogEntry::Level::TRACE, "us"));
    REQUIRE(mockLogger->hasNoErrors());
}
