#pragma once
#include <common/types.hpp>
#include <cmath>
#include <numbers>

namespace robotlib {

[[nodiscard]] inline double normalizeAngle(double angle) noexcept {
    while (angle > std::numbers::pi) angle -= 2.0 * std::numbers::pi;
    while (angle <= -std::numbers::pi) angle += 2.0 * std::numbers::pi;
    return angle;
}

[[nodiscard]] inline double angleWrap(double angle) noexcept {
    return normalizeAngle(angle);
}

[[nodiscard]] inline double distance(const Point2D& a, const Point2D& b) noexcept {
    return a.distanceTo(b);
}

[[nodiscard]] inline double distance(double x1, double y1, double x2, double y2) noexcept {
    const double dx = x1 - x2;
    const double dy = y1 - y2;
    return std::sqrt(dx * dx + dy * dy);
}

[[nodiscard]] inline double lerp(double a, double b, double t) noexcept {
    return a + t * (b - a);
}

[[nodiscard]] inline double angleDifference(double from, double to) noexcept {
    return normalizeAngle(to - from);
}

}  // namespace robotlib
