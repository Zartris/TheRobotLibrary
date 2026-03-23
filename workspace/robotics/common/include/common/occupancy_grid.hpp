#pragma once
#include <common/types.hpp>
#include <vector>
#include <cstdint>
#include <cmath>

namespace robotlib {

enum class CellState : int8_t {
    Free = -1,
    Unknown = 0,
    Occupied = 1
};

struct OccupancyGrid {
    int width{0};
    int height{0};
    double resolution{0.1};
    Point2D origin{0.0, 0.0};
    std::vector<int8_t> cells;  // log-odds values, 0 = unknown

    OccupancyGrid() = default;
    OccupancyGrid(int w, int h, double res, Point2D org = {0.0, 0.0})
        : width(w), height(h), resolution(res), origin(org), cells(w * h, 0) {}

    [[nodiscard]] bool isValid(int gx, int gy) const noexcept {
        return gx >= 0 && gx < width && gy >= 0 && gy < height;
    }

    [[nodiscard]] int8_t& at(int gx, int gy) { return cells[gy * width + gx]; }
    [[nodiscard]] int8_t at(int gx, int gy) const { return cells[gy * width + gx]; }

    [[nodiscard]] Point2D toWorld(int gx, int gy) const noexcept {
        return {origin.x + (gx + 0.5) * resolution,
                origin.y + (gy + 0.5) * resolution};
    }

    [[nodiscard]] std::pair<int, int> toGrid(double wx, double wy) const noexcept {
        return {static_cast<int>(std::floor((wx - origin.x) / resolution)),
                static_cast<int>(std::floor((wy - origin.y) / resolution))};
    }

    [[nodiscard]] CellState cellState(int gx, int gy) const noexcept {
        if (!isValid(gx, gy)) return CellState::Unknown;
        const auto v = cells[gy * width + gx];
        if (v > 0) return CellState::Occupied;
        if (v < 0) return CellState::Free;
        return CellState::Unknown;
    }
};

}  // namespace robotlib
