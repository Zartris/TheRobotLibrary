# M13 — Classical & Optimal Control

**Status:** Not Started  
**Dependencies:** M3 (stable `IController` interface + kinematic models). M11 recommended but not required.  
**Scope:** Fills the gap between geometric (pure_pursuit) and model-based (MPC) control with two classical algorithms: LQR state-feedback and Stanley cross-track path following.

---

## Goal

Add two foundational control algorithms that cover complementary design philosophies:
LQR teaches cost-function design and optimal state feedback (the conceptual sibling to MPC);
Stanley teaches geometric path tracking with speed-normalized CTE normalization
(the Ackermann-friendly sibling to pure_pursuit). Both are independently implementable
and testable within M13.

---

## Modules

### control/lqr

Discrete-time infinite-horizon LQR. Solves the DARE (Discrete Algebraic Riccati Equation)
using Eigen's `GeneralizedSelfAdjointEigenSolver` or iterative solver. Teaches cost-function
design and optimal state feedback — the conceptual sibling to MPC.

- [ ] `include/lqr/lqr_controller.hpp` — `LQRController : IController`
- [ ] `include/lqr/lqr_config.hpp` — `LQRConfig` (Q, R, A, B matrices; discretization dt)
- [ ] `src/lqr_controller.cpp` — DARE solver + `u = -Kx` feedback
- [ ] Support linearised kinematic model at operating point (diff-drive linearization helper)
- [ ] `tests/test_lqr_controller.cpp`:
  - Regulator drives state to zero from perturbation within N steps
  - Higher Q (R fixed) → faster convergence; higher R (Q fixed) → smaller control inputs
  - Provided A/B matrices for double integrator → analytic K matches theoretical solution
  - Stability: eigenvalues of `(A - BK)` lie strictly inside unit circle
- [ ] Phase 4.5: `ILogger`, state transitions at `DEBUG`, DARE solve time at `TRACE`
- [ ] Sim: selectable via ImGui controller dropdown
- [ ] ImGui panel: render Q/R gain panel + current control effort

### control/stanley

Stanley controller from Stanford's DARPA challenge. Combines heading error and
speed-normalized cross-track error into a single steering command. Ackermann-friendly
but also applicable to differential-drive via Ackermann approximation.

- [ ] `include/stanley/stanley_controller.hpp` — `StanleyController : IController`
- [ ] `include/stanley/stanley_config.hpp` — `StanleyConfig` (gain k, velocity softening ε, max steering angle)
- [ ] `src/stanley_controller.cpp` — `δ = ψ_e + arctan(k·e_cte / (v + ε))`
- [ ] `tests/test_stanley_controller.cpp`:
  - Straight path: CTE decays exponentially from initial offset (verify gain k effect)
  - Curved path: tracks within tolerance on circular arc
  - Heading-only error (CTE = 0): pure heading correction, no CTE term
  - Zero velocity: softening ε prevents division by zero; output bounded
  - Reversing: heading error flips sign correctly
- [ ] Phase 4.5: `ILogger`, CTE + heading error logged at `DEBUG`, per-step σ at `TRACE`
- [ ] Sim: selectable via ImGui controller dropdown
- [ ] ImGui panel: cross-track error overlay (same style as `frenet` from M11)

---

## Deliverables

- [ ] `control/lqr` module: interface, implementation, tests
- [ ] `control/stanley` module: interface, implementation, tests
- [ ] Both controllers hot-swappable via ImGui mid-run without crash
- [ ] LQR: Q/R weighting behaviour verified quantitatively in tests
- [ ] Stanley: CTE overlay rendered in simulation app
- [ ] All modules pass Phase 4.5 — Observability gate

---

## Exit Criteria

1. LQR drives linearized double-integrator to zero; eigenvalues of `(A-BK)` inside unit circle
2. LQR Q/R weighting behaviour verified quantitatively in tests
3. Stanley tracks straight and curved paths within CTE tolerance
4. Both controllers hot-swappable via ImGui mid-run without crash
5. All unit tests pass, CI green
6. All modules pass Phase 4.5 — Observability gate (state transitions at `DEBUG`, metrics at `TRACE`)

---

## NOT IN

Full nonlinear DARE iteration (use existing Eigen facility), LQG (P8) observer,
feedback linearization (P6), hardware Ackermann kinematics, Riccati recursion
for time-varying LQR.
