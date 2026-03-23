#pragma once
#include <vector>
#include <numbers>

namespace robotlib {

struct LaserScan {
    double angleMin{-std::numbers::pi};
    double angleMax{std::numbers::pi};
    double angleIncrement{0.01};
    double rangeMin{0.1};
    double rangeMax{30.0};
    std::vector<float> ranges;
    std::vector<float> intensities;

    [[nodiscard]] int numRays() const noexcept {
        return static_cast<int>(ranges.size());
    }

    [[nodiscard]] double angleAt(int index) const noexcept {
        return angleMin + index * angleIncrement;
    }
};

}  // namespace robotlib
