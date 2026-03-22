#pragma once
#include <common/interfaces/i_velocity_profiler.hpp>
#include <logging/get_logger.hpp>
#include <memory>

namespace robotlib {

class TrapezoidalProfiler : public IVelocityProfiler {
public:
    TrapezoidalProfiler();

    TimedPath profile(const Path& path, const VelocityConstraints& constraints) override;

private:
    double curvatureAtIndex(const Path& path, size_t idx) const;
    std::shared_ptr<ILogger> m_logger;
};

}  // namespace robotlib
