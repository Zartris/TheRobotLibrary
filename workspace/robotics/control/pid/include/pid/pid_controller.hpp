#pragma once
#include <common/interfaces/i_controller.hpp>
#include <common/types.hpp>
#include <logging/get_logger.hpp>
#include <memory>

namespace robotlib {

struct PIDConfig {
    double kp{1.0};
    double ki{0.0};
    double kd{0.0};
    double maxOutput{10.0};
    double maxIntegral{5.0};
};

class PIDController {
public:
    explicit PIDController(const PIDConfig& config = {});

    double compute(double error, double dt);
    void reset();

    const PIDConfig& config() const { return m_config; }

private:
    PIDConfig m_config;
    double m_integral{0.0};
    double m_prevError{0.0};
    bool m_firstUpdate{true};
};

class HeadingSpeedController : public IController {
public:
    HeadingSpeedController(const PIDConfig& headingConfig, const PIDConfig& speedConfig);

    Twist compute(const Pose2D& current, const Pose2D& target, double dt) override;
    void reset() override;

private:
    PIDController m_headingPID;
    PIDController m_speedPID;
    std::shared_ptr<ILogger> m_logger;
};

}  // namespace robotlib
