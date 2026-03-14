# feedback_linearization

Exact input-output linearization controller for differential-drive robots in chained form. Transforms the nonlinear system `(x, y, θ, v)` to a pair of decoupled linear integrators by introducing a virtual output point `ξ` ahead of the robot's axle, then applies a linear inner controller to the linearized system.

**Milestone:** M18 — Advanced Nonlinear Control  
**Status:** Scaffold only — awaiting implementation

## Features

- `FeedbackLinearizationController : IController` — drop-in replacement for any `IController`
- Virtual output point: `ξ = (x + l·cos θ, y + l·sin θ)` where `l` is the lookahead distance
- Linearizing control law maps inner controller output back to `(v, ω)` for the robot
- Numerical Lie derivative checker for test verification
- Singularity at `l = 0` documented via `std::expected` error

## Dependencies

- `common` (logging, types)
- Eigen3 (matrix math)
