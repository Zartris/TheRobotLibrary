#pragma once
#include <common/laser_scan.hpp>
#include <logging/get_logger.hpp>
#include <memory>

namespace robotlib {

struct FilterConfig {
    double rangeMin{0.1};
    double rangeMax{30.0};
    int medianWindow{0};  // 0 = disabled, must be odd
};

class ScanFilter {
public:
    explicit ScanFilter(const FilterConfig& config = {});

    LaserScan filterScan(const LaserScan& input) const;

private:
    void clipRanges(LaserScan& scan) const;
    void applyMedianFilter(LaserScan& scan) const;

    FilterConfig m_config;
    std::shared_ptr<ILogger> m_logger;
};

}  // namespace robotlib
