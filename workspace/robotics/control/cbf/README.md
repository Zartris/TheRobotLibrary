# Control Barrier Functions (CBF)

`CbfSafetyFilter` is a safety decorator that wraps any `IController`. It filters the
nominal control output through a QP-based safety filter, ensuring the robot stays within
user-defined safe sets — typically collision-free regions around detected obstacles.

## Module Layout

```
cbf/
├── CMakeLists.txt
├── README.md
├── include/cbf/
│   └── cbf_safety_filter.hpp   ← CbfSafetyFilter : IController
├── src/
│   └── cbf_safety_filter.cpp
├── tests/
└── docs/
    └── theory.md
```

## Usage

```cpp
#include <cbf/cbf_safety_filter.hpp>
#include <pid/pid_controller.hpp>

auto nominal = std::make_unique<HeadingSpeedController>(pid_config);
CbfConfig cbf_cfg{.r_safe = 0.5, .alpha_gain = 1.0};
CbfSafetyFilter safe_ctrl(std::move(nominal), cbf_cfg);

// Call each tick before using the controller
safe_ctrl.updateObstacles(detected_obstacles);
auto twist = safe_ctrl.compute(current_state, goal_pose, dt);
```

## Hot-swap

Select the active controller via the simulation app's ImGui panel. The `"wraps"` field
in the controller config specifies the nominal controller underneath the safety filter.

## Theory

See `docs/theory.md`.
