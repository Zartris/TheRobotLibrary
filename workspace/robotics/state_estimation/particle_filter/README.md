# particle_filter

Monte Carlo Localization (MCL) — particle filter for global and local robot localization.

**Standalone module.** Depends only on `common`. Copy this folder to any C++ project.

---

## What's in this module

- `ParticleFilter` — configurable MCL with resampling and adaptive particle count
- `DifferentialDriveMotionModel` — same odometry model as in `ekf/`, shared via `common`
- `BeamSensorModel` — lidar beam likelihood for particle weighting
- `LikelihoodFieldModel` — faster likelihood field alternative to the full beam model

---

## Integration

```cmake
add_subdirectory(common)
add_subdirectory(particle_filter)
target_link_libraries(your_target PRIVATE particle_filter)
```

```cpp
#include <particle_filter/particle_filter.hpp>

particle_filter::Config cfg{.num_particles = 500};
particle_filter::ParticleFilter mcl{cfg, map};

// Initialization: spread particles over the whole map (global localization)
mcl.initGlobal();

// Or initialize near a known pose (tracking mode)
mcl.initPose(initial_pose, position_std=0.3, angle_std=0.1);

// In the control loop:
mcl.predict(cmd_vel, dt);
mcl.update(lidar_scan);
common::Pose2D best_pose = mcl.bestEstimate();
```

---

## Theory

See [`docs/theory.md`](docs/theory.md) for the particle filter algorithm, resampling
strategies, and the adaptive particle count extension (AMCL).
