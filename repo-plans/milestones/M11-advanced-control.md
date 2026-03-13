# M11 — Advanced Control

**Status:** Not Started  
**Dependencies:** M3 (stable control interfaces and kinematic models)  
**Scope:** Two new control algorithms (adaptive gain scheduling, Frenet-Serret path following) plus RLS parameter estimation. All hot-swappable. CBF was delivered in M3.

> **Note:** CBF is implemented in M3 as a decorator on existing controllers. M11 adds
> adaptive control (gain scheduling + RLS parameter estimation) and the Frenet frame
> controller.

---

## Goal

Extend the control repertoire beyond geometric (pure_pursuit) and optimization-based (MPC)
controllers with algorithms that address real-world deployment concerns: safety guarantees,
online adaptation to unknown robot dynamics, and smooth path tracking in the Frenet frame.

---

## Modules

### adaptive/gain_scheduling

Adaptive PID controller that adjusts gains online from tracking error history. Demonstrates
the performance gain of adaptation vs. fixed-gain PID on changing robot payloads or terrain.

- [ ] `include/gain_scheduling/adaptive_pid_controller.hpp` — `AdaptivePidController : IController`
- [ ] `include/gain_scheduling/adaptive_pid_config.hpp` — `AdaptivePidConfig` (base gains, adaptation rates, gain bounds)
- [ ] `src/adaptive_pid_controller.cpp`
- [ ] `tests/test_adaptive_pid_controller.cpp`:
  - Fixed setpoint: gains adapt, tracking error decreases monotonically
  - Step disturbance (robot load doubles): gains readjust within N steps
  - Gain bounds enforced: gains never exceed `[min, max]`
  - Reset clears adaptation state
- [ ] Sim integration: selectable via `PUT /api/robot/controller {"type":"adaptive_pid"}`
- [ ] Frontend: render current gain values as a live readout panel

### adaptive/rls

Recursive Least Squares estimator for online robot parameter identification. Runs in
parallel with any controller and exposes identified model parameters (mass, friction,
drag) for consumption by MPC or adaptive PID.

- [ ] `include/rls/rls_estimator.hpp` — `RlsEstimator`, `RlsConfig`, `RobotModelParams`
- [ ] `src/rls_estimator.cpp`
- [ ] `tests/test_rls_estimator.cpp`:
  - Known-parameter system: RLS converges to true params within 50 steps
  - Forgetting factor < 1: tracks parameter step-change within 20 steps
  - Covariance doesn't blow up or collapse under repeated identical inputs
  - `getEstimatedParams()` returns stable values after convergence
- [ ] Sim integration: RLS runs as background estimator; estimated params logged each tick
- [ ] Frontend: render estimated params (mass estimate, friction estimate) as live readout

### frenet

Frenet-Serret frame path following controller. Decomposes tracking into lateral (cross-track
error) and longitudinal (speed) loops. Produces smoother following on curved paths than
pure_pursuit, especially at higher speeds.

- [ ] `include/frenet/frenet_controller.hpp` — `FrenetController : IController`, `FrenetConfig`
- [ ] `src/frenet_controller.cpp`
- [ ] `tests/test_frenet_controller.cpp`:
  - Straight path: lateral error converges to zero from offset start
  - Curved path: robot follows circle with bounded cross-track error
  - Feedforward curvature term: robot follows curve with less lateral error than P-only
  - Speed tracking: longitudinal PI tracks `v_ref` from velocity profiler
  - Path end: robot decelerates and stops at last waypoint
- [ ] Sim integration: selectable via `PUT /api/robot/controller {"type":"frenet"}`
- [ ] Frontend: render cross-track error overlay (distance from robot to closest path point)

---

## Deliverables

- [ ] `adaptive/gain_scheduling` module: interface, implementation, tests, sim integration
- [ ] `adaptive/rls` module: interface, implementation, tests, sim integration
- [ ] `frenet` module: interface, implementation, tests, sim integration
- [ ] All controllers selectable via REST mid-run without crash
- [ ] Frontend shows module-specific debug panels (gain values, RLS params, cross-track error)
- [ ] Mini-demo: compare Frenet vs pure_pursuit on tight hairpin curve; compare adaptive PID vs fixed-gain PID after sudden load change; show RLS param convergence plot

## Exit Criteria

1. All three new modules navigate standard scenarios without regression
2. Adaptive PID visibly reduces tracking error after a simulated load change
3. Frenet cross-track error lower than pure_pursuit on curved paths (quantified in logs)
4. RLS correctly identifies mass/friction on a known-params scenario (within 10%)
5. REST controller swap works mid-run for all new modules
6. All unit tests pass, CI green
7. All modules pass Phase 4.5 — Observability gate (state transitions logged, metrics at TRACE)

## NOT IN

True MRAC stability proofs, full SLAM-integrated re-planning, 3D dynamics, hardware-specific
parameter identification routines.
