# common

Shared types, math primitives, and core interfaces used by all other robotics modules.

This is the only module that other modules are allowed to depend on. If a type or utility
is needed by more than one domain module, it belongs here.

---

## What's in this module

- **Geometry types:** `Pose2D`, `Transform2D`, `Point2D`, `Vector2D`
- **Motion types:** `Twist` (linear + angular velocity), `Wrench`
- **Math utilities:** angle wrapping, interpolation, numeric differentiation
- **Core interfaces:** abstract base classes that domain modules implement
  (e.g. `IController`, `IPlanner`, `ISensor`)
- **Units:** strong typedef wrappers for meters, radians, seconds (avoids unit confusion)

---

## Integration

```cmake
# In your CMakeLists.txt
add_subdirectory(common)
target_link_libraries(your_target PRIVATE common)
```

```cpp
// In your code
#include <common/pose2d.hpp>
#include <common/twist.hpp>

common::Pose2D robot_pose{1.0, 2.0, 0.5};  // x, y, theta
common::Twist  cmd_vel{0.3, 0.0, 0.1};     // vx, vy, omega
```

---

## Theory

See [`docs/theory.md`](docs/theory.md) for background on the mathematical representations
used in 2D robotics.
