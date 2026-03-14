# visual_inertial_odometry

Loosely-coupled Visual-Inertial Odometry (VIO). Fuses IMU pre-integrated motion priors
with visual odometry (VO) pose updates through an EKF/UKF estimator, achieving lower
drift than either source alone. Accepts inputs as `common/` types — does not link against
`imu_processing` or `visual_odometry` directly; inputs are assembled by the caller.

## Architecture

**Loosely-coupled design:**
- IMU pre-integration result propagates the state between keyframes (high-rate)
- VO relative pose update corrects the state at keyframe rate (low-rate)
- State: `VIOState` = SE3 pose + velocity + `ImuBias`

## Dependencies

- `common` (IStateEstimator interface, logging, common types)
- `Eigen3`
- Caller provides: `ImuPreintegrationResult` and `OdometryResult` (from `common/` types)

## Usage

```cpp
#include <visual_inertial_odometry/visual_inertial_odometry.hpp>
#include <visual_inertial_odometry/vio_config.hpp>

VIOConfig cfg;
cfg.imu_noise = ...;
cfg.vo_noise  = ...;

VisualInertialOdometry vio(cfg, common::getLogger("visual_inertial_odometry"));
vio.predict(imu_preint_result);         // called at 100–1000 Hz
vio.update(vo_pose_result);             // called at keyframe rate
VIOState state = vio.getState();
```

## Milestone

Part of **M15 — Visual-Inertial Odometry** (culmination module).  
Prerequisite: `perception/imu_processing` must be complete.  
See `repo-plans/modules/visual_inertial_odometry.md` for full task checklist.
