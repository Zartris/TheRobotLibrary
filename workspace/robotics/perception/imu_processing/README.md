# imu_processing

IMU filtering and pre-integration module. Provides a complementary filter for real-time
orientation estimation, exponential moving average bias estimation, and IMU pre-integration
between keyframes (required for loosely-coupled VIO).

## Components

- **`ImuFilter`** — complementary filter fusing accelerometer + gyroscope; bias drift estimation
- **`ImuPreintegrator`** — accumulates ΔR, Δv, Δp between keyframes; computes Jacobians w.r.t. bias
- **`ImuTypes`** — `ImuMeasurement`, `ImuBias`, `ImuState`

## Dependencies

- `common` (logging)
- `Eigen3`

## Usage

```cpp
#include <imu_processing/imu_filter.hpp>
#include <imu_processing/imu_preintegrator.hpp>
#include <imu_processing/imu_types.hpp>

ImuFilter filter(common::getLogger("imu_processing"));
filter.reset();
for (auto& meas : imu_stream) {
    filter.update(meas);
}
ImuState state = filter.getState();

ImuPreintegrator preintegrator(common::getLogger("imu_processing"));
preintegrator.reset(initial_bias);
for (auto& meas : imu_segment) {
    preintegrator.integrate(meas);
}
auto result = preintegrator.getResult(); // ΔR, Δv, Δp + Jacobians
```

## Milestone

Part of **M15 — Visual-Inertial Odometry** (entry module).  
See `repo-plans/modules/imu_processing.md` for full task checklist.
