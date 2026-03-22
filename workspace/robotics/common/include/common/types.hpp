#pragma once
#include <Eigen/Core>
#include <vector>
#include <cmath>

namespace robotlib {

struct Point2D {
    double x{0.0};
    double y{0.0};

    [[nodiscard]] double distanceTo(const Point2D& other) const noexcept {
        const double dx = x - other.x;
        const double dy = y - other.y;
        return std::sqrt(dx * dx + dy * dy);
    }

    [[nodiscard]] bool operator==(const Point2D& other) const noexcept {
        constexpr double kEps = 1e-9;
        return std::abs(x - other.x) < kEps && std::abs(y - other.y) < kEps;
    }
};

struct Pose2D {
    double x{0.0};
    double y{0.0};
    double theta{0.0};

    [[nodiscard]] Point2D position() const noexcept { return {x, y}; }

    [[nodiscard]] bool operator==(const Pose2D& other) const noexcept {
        constexpr double kEps = 1e-9;
        return std::abs(x - other.x) < kEps &&
               std::abs(y - other.y) < kEps &&
               std::abs(theta - other.theta) < kEps;
    }
};

struct Twist {
    double linear{0.0};
    double angular{0.0};

    [[nodiscard]] bool operator==(const Twist& other) const noexcept {
        constexpr double kEps = 1e-9;
        return std::abs(linear - other.linear) < kEps && std::abs(angular - other.angular) < kEps;
    }
};

using Path = std::vector<Pose2D>;

struct TimedPose {
    Pose2D pose;
    double time{0.0};
    double velocity{0.0};
};

using TimedPath = std::vector<TimedPose>;

}  // namespace robotlib
