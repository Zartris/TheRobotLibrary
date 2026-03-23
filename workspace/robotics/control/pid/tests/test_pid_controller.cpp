#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <logging/get_logger.hpp>
#include <pid/pid_controller.hpp>
#include <testing/recording_logger.hpp>
#include <numbers>

using namespace robotlib;

TEST_CASE("P-only: output proportional to error", "[pid]") {
    PIDConfig cfg{2.0, 0.0, 0.0, 100.0, 100.0};
    PIDController pid(cfg);

    double output = pid.compute(5.0, 0.1);
    REQUIRE_THAT(output, Catch::Matchers::WithinAbs(10.0, 1e-9));
}

TEST_CASE("PI: steady-state error goes to zero", "[pid]") {
    PIDConfig cfg{1.0, 2.0, 0.0, 100.0, 100.0};
    PIDController pid(cfg);

    double error = 1.0;
    double dt = 0.1;

    for (int i = 0; i < 100; ++i) {
        double output = pid.compute(error, dt);
        error -= output * dt;  // simple plant
    }

    REQUIRE_THAT(error, Catch::Matchers::WithinAbs(0.0, 0.1));
}

TEST_CASE("PID: derivative damps overshoot", "[pid]") {
    PIDConfig cfgPD{1.0, 0.0, 0.5, 100.0, 100.0};
    PIDConfig cfgP{1.0, 0.0, 0.0, 100.0, 100.0};
    PIDController pidPD(cfgPD);
    PIDController pidP(cfgP);

    // With increasing error, derivative should add damping
    pidPD.compute(1.0, 0.1);
    double outputPD = pidPD.compute(2.0, 0.1);

    pidP.compute(1.0, 0.1);
    double outputP = pidP.compute(2.0, 0.1);

    // PD should produce larger output when error is increasing (kd adds to kp)
    REQUIRE(outputPD > outputP);
}

TEST_CASE("Anti-windup: integral doesn't explode", "[pid]") {
    PIDConfig cfg{0.0, 1.0, 0.0, 100.0, 2.0};
    PIDController pid(cfg);

    // Apply large error many times
    for (int i = 0; i < 1000; ++i) {
        pid.compute(100.0, 0.1);
    }

    // Output should be clamped by maxIntegral
    double output = pid.compute(0.0, 0.1);
    REQUIRE_THAT(output, Catch::Matchers::WithinAbs(2.0, 1e-6));
}

TEST_CASE("Reset clears state", "[pid]") {
    PIDConfig cfg{1.0, 1.0, 1.0, 100.0, 100.0};
    PIDController pid(cfg);

    pid.compute(5.0, 0.1);
    pid.compute(3.0, 0.1);
    pid.reset();

    // After reset, first compute should behave as if fresh
    double output = pid.compute(1.0, 0.1);
    // P-only output (no I accumulated, no D since first update)
    REQUIRE_THAT(output, Catch::Matchers::WithinAbs(1.1, 0.2));
}

TEST_CASE("HeadingSpeedController compute", "[pid]") {
    PIDConfig hCfg{1.0, 0.0, 0.0, 10.0, 10.0};
    PIDConfig sCfg{1.0, 0.0, 0.0, 10.0, 10.0};
    HeadingSpeedController controller(hCfg, sCfg);

    Pose2D current{0.0, 0.0, 0.0};
    Pose2D target{1.0, 0.0, 0.0};

    auto cmd = controller.compute(current, target, 0.1);

    // Goal is straight ahead: heading error ~0, should drive forward
    REQUIRE(cmd.linear > 0.0);
    REQUIRE_THAT(cmd.angular, Catch::Matchers::WithinAbs(0.0, 0.1));
}

TEST_CASE("HeadingSpeedController logging and observability", "[pid][logging]") {
    auto cleanup = std::shared_ptr<void>(nullptr,
        [](void*) { robotlib::clearLoggerRegistry(); });

    auto mockLogger = std::make_shared<robotlib::testing::RecordingLogger>();
    robotlib::registerLogger("pid", mockLogger);

    PIDConfig hCfg{1.0, 0.0, 0.0, 10.0, 10.0};
    PIDConfig sCfg{1.0, 0.0, 0.0, 10.0, 10.0};
    HeadingSpeedController controller(hCfg, sCfg);

    // Verify DEBUG log on initialization
    REQUIRE(mockLogger->hasMessageContaining(
        robotlib::testing::LogEntry::Level::DEBUG, "HeadingSpeedController initialized"));

    // Verify no errors during nominal construction
    REQUIRE(mockLogger->hasNoErrors());

    // Run a compute call and check TRACE timing log
    Pose2D current{0.0, 0.0, 0.0};
    Pose2D target{1.0, 0.0, 0.0};
    controller.compute(current, target, 0.1);
    REQUIRE(mockLogger->hasMessageContaining(
        robotlib::testing::LogEntry::Level::TRACE, "us"));
}
