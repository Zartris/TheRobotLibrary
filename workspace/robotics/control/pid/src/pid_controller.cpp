#include <pid/pid_controller.hpp>
#include <common/geometry.hpp>
#include <cmath>
#include <algorithm>
#include <sstream>
#include <chrono>

namespace robotlib {

PIDController::PIDController(const PIDConfig& config) : m_config(config) {}

double PIDController::compute(double error, double dt) {
    if (dt <= 0.0) return 0.0;

    // Proportional
    double p = m_config.kp * error;

    // Integral with anti-windup
    m_integral += error * dt;
    m_integral = std::clamp(m_integral, -m_config.maxIntegral, m_config.maxIntegral);
    double i = m_config.ki * m_integral;

    // Derivative
    double d = 0.0;
    if (!m_firstUpdate) {
        d = m_config.kd * (error - m_prevError) / dt;
    }
    m_firstUpdate = false;
    m_prevError = error;

    double output = p + i + d;
    return std::clamp(output, -m_config.maxOutput, m_config.maxOutput);
}

void PIDController::reset() {
    m_integral = 0.0;
    m_prevError = 0.0;
    m_firstUpdate = true;
}

HeadingSpeedController::HeadingSpeedController(const PIDConfig& headingConfig,
                                                 const PIDConfig& speedConfig)
    : m_headingPID(headingConfig), m_speedPID(speedConfig),
      m_logger(getLogger("pid")) {
    m_logger->debug("HeadingSpeedController initialized");
}

Twist HeadingSpeedController::compute(const Pose2D& current, const Pose2D& target, double dt) {
    auto start = std::chrono::high_resolution_clock::now();

    // Heading error
    double targetAngle = std::atan2(target.y - current.y, target.x - current.x);
    double headingError = normalizeAngle(targetAngle - current.theta);

    // Distance error
    double dx = target.x - current.x;
    double dy = target.y - current.y;
    double distError = std::sqrt(dx * dx + dy * dy);

    // If facing away from target, reduce speed
    double speedError = distError * std::cos(headingError);

    Twist cmd;
    cmd.angular = m_headingPID.compute(headingError, dt);
    cmd.linear = m_speedPID.compute(speedError, dt);

    // Slow down when heading error is large
    if (std::abs(headingError) > 0.5) {
        cmd.linear *= 0.5;
    }

    auto end = std::chrono::high_resolution_clock::now();
    auto us = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
    std::ostringstream oss;
    oss << "PID compute: " << us << " us, heading_err=" << headingError << " dist_err=" << distError;
    m_logger->trace(oss.str());

    return cmd;
}

void HeadingSpeedController::reset() {
    m_headingPID.reset();
    m_speedPID.reset();
    m_logger->debug("HeadingSpeedController reset");
}

}  // namespace robotlib
