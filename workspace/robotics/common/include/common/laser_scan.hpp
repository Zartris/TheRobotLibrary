#pragma once
#include <vector>
#include <cmath>

namespace robotlib {

struct LaserScan {
    double angleMin{-M_PI};
    double angleMax{M_PI};
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
