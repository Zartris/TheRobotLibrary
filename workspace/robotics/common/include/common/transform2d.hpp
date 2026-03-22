#pragma once
#include <common/types.hpp>
#include <cmath>

namespace robotlib {

class Transform2D {
public:
    Transform2D() = default;
    explicit Transform2D(const Pose2D& pose) : m_pose(pose) {}
    Transform2D(double x, double y, double theta) : m_pose{x, y, theta} {}

    [[nodiscard]] static Transform2D identity() noexcept { return Transform2D{}; }

    [[nodiscard]] Transform2D compose(const Transform2D& other) const noexcept {
        const double c = std::cos(m_pose.theta);
        const double s = std::sin(m_pose.theta);
        return Transform2D{
            m_pose.x + c * other.m_pose.x - s * other.m_pose.y,
            m_pose.y + s * other.m_pose.x + c * other.m_pose.y,
            m_pose.theta + other.m_pose.theta
        };
    }

    [[nodiscard]] Transform2D inverse() const noexcept {
        const double c = std::cos(m_pose.theta);
        const double s = std::sin(m_pose.theta);
        return Transform2D{
            -(c * m_pose.x + s * m_pose.y),
            -(-s * m_pose.x + c * m_pose.y),
            -m_pose.theta
        };
    }

    [[nodiscard]] Point2D transformPoint(const Point2D& p) const noexcept {
        const double c = std::cos(m_pose.theta);
        const double s = std::sin(m_pose.theta);
        return Point2D{
            c * p.x - s * p.y + m_pose.x,
            s * p.x + c * p.y + m_pose.y
        };
    }

    [[nodiscard]] const Pose2D& pose() const noexcept { return m_pose; }

private:
    Pose2D m_pose;
};

}  // namespace robotlib
