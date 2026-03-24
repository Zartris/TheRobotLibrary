#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <mpc/mpc_controller.hpp>
#include <testing/recording_logger.hpp>
#include <logging/get_logger.hpp>

using namespace robotlib;

TEST_CASE("MPC reports correct backend", "[mpc]") {
    MPCController mpc;
    std::string be = mpc.backend();
    // Should be "eigen" in CI (no acados) or "acados" when installed
    REQUIRE((be == "eigen" || be == "acados"));
}

TEST_CASE("MPC tracks straight reference", "[mpc]") {
    MPCConfig cfg;
    cfg.horizon = 10;
    cfg.predictionTime = 1.0;
    cfg.maxLinearVelocity = 1.0;
    MPCController mpc(cfg);

    Pose2D current{0.0, 0.0, 0.0};
    Pose2D target{5.0, 0.0, 0.0};
    auto cmd = mpc.compute(current, target, 0.1);

    // Should command positive forward velocity
    REQUIRE(cmd.linear > 0.0);
    // Should command near-zero angular (straight line)
    CHECK(std::abs(cmd.angular) < 0.5);
}

TEST_CASE("MPC tracks curved reference", "[mpc]") {
    MPCConfig cfg;
    cfg.horizon = 10;
    MPCController mpc(cfg);

    Pose2D current{0.0, 0.0, 0.0};
    Pose2D target{3.0, 3.0, 1.57};
    auto cmd = mpc.compute(current, target, 0.1);

    // Should have nonzero velocity
    REQUIRE(cmd.linear > 0.0);
    // Predicted trajectory should exist
    REQUIRE(mpc.getPredictedTrajectory().size() > 1);
}

TEST_CASE("MPC respects velocity constraints", "[mpc]") {
    MPCConfig cfg;
    cfg.maxLinearVelocity = 0.5;
    cfg.maxAngularVelocity = 1.0;
    cfg.horizon = 10;
    MPCController mpc(cfg);

    Pose2D current{0.0, 0.0, 0.0};
    Pose2D target{10.0, 0.0, 0.0};  // Far away — wants high speed
    auto cmd = mpc.compute(current, target, 0.1);

    REQUIRE(std::abs(cmd.linear) <= cfg.maxLinearVelocity + 1e-9);
    REQUIRE(std::abs(cmd.angular) <= cfg.maxAngularVelocity + 1e-9);
}

TEST_CASE("MPC goal reached produces zero twist", "[mpc]") {
    MPCController mpc;

    Pose2D current{1.0, 1.0, 0.0};
    Pose2D target{1.0, 1.0, 0.0};
    auto cmd = mpc.compute(current, target, 0.1);

    REQUIRE_THAT(cmd.linear, Catch::Matchers::WithinAbs(0.0, 1e-9));
    REQUIRE_THAT(cmd.angular, Catch::Matchers::WithinAbs(0.0, 1e-9));
}

TEST_CASE("MPC with reference path", "[mpc]") {
    MPCConfig cfg;
    cfg.horizon = 5;
    MPCController mpc(cfg);

    Path path;
    for (int i = 0; i <= 10; ++i) {
        path.push_back({static_cast<double>(i) * 0.5, 0.0, 0.0});
    }
    mpc.setReferencePath(path);

    Pose2D current{0.0, 0.0, 0.0};
    Pose2D target{5.0, 0.0, 0.0};
    auto cmd = mpc.compute(current, target, 0.1);

    REQUIRE(cmd.linear > 0.0);
}

TEST_CASE("MPC logging and observability", "[mpc][logging]") {
    auto cleanup = std::shared_ptr<void>(nullptr, [](void*) { robotlib::clearLoggerRegistry(); });
    auto mockLogger = std::make_shared<robotlib::testing::RecordingLogger>();
    robotlib::registerLogger("mpc", mockLogger);

    MPCController mpc;

    REQUIRE(mockLogger->hasMessageContaining(
        robotlib::testing::LogEntry::Level::INFO, "initialized"));

    Pose2D current{0.0, 0.0, 0.0};
    Pose2D target{5.0, 0.0, 0.0};
    mpc.compute(current, target, 0.1);

    REQUIRE(mockLogger->hasMessageContaining(
        robotlib::testing::LogEntry::Level::TRACE, "us"));
    REQUIRE(mockLogger->hasNoErrors());
}
