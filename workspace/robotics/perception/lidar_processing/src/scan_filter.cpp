#include <lidar_processing/scan_filter.hpp>
#include <algorithm>
#include <cmath>
#include <chrono>
#include <sstream>

namespace robotlib {

ScanFilter::ScanFilter(const FilterConfig& config)
    : m_config(config), m_logger(getLogger("lidar_processing")) {
    std::ostringstream oss;
    oss << "ScanFilter initialized: rangeMin=" << config.rangeMin
        << " rangeMax=" << config.rangeMax
        << " medianWindow=" << config.medianWindow;
    m_logger->debug(oss.str());
}

LaserScan ScanFilter::filterScan(const LaserScan& input) const {
    auto start = std::chrono::high_resolution_clock::now();

    LaserScan output = input;
    clipRanges(output);
    if (m_config.medianWindow > 0) {
        applyMedianFilter(output);
    }

    auto end = std::chrono::high_resolution_clock::now();
    auto us = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
    std::ostringstream oss;
    oss << "filterScan: " << input.numRays() << " rays in " << us << " us";
    m_logger->debug(oss.str());

    return output;
}

void ScanFilter::clipRanges(LaserScan& scan) const {
    for (auto& r : scan.ranges) {
        if (std::isnan(r) || r < m_config.rangeMin || r > m_config.rangeMax) {
            r = std::numeric_limits<float>::quiet_NaN();
        }
    }
}

void ScanFilter::applyMedianFilter(LaserScan& scan) const {
    if (m_config.medianWindow < 3) return;

    const int half = m_config.medianWindow / 2;
    const int n = static_cast<int>(scan.ranges.size());
    std::vector<float> filtered(n);

    for (int i = 0; i < n; ++i) {
        std::vector<float> window;
        window.reserve(m_config.medianWindow);
        for (int j = i - half; j <= i + half; ++j) {
            int idx = std::clamp(j, 0, n - 1);
            if (!std::isnan(scan.ranges[idx])) {
                window.push_back(scan.ranges[idx]);
            }
        }
        if (window.empty()) {
            filtered[i] = std::numeric_limits<float>::quiet_NaN();
        } else {
            std::sort(window.begin(), window.end());
            filtered[i] = window[window.size() / 2];
        }
    }
    scan.ranges = std::move(filtered);
}

}  // namespace robotlib
