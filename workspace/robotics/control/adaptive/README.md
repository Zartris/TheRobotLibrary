# Adaptive Control

Modules that modify control parameters or model estimates online, in response to observed
tracking errors or identified system dynamics.

## Sub-modules

| Sub-module          | Description                                                                |
|---------------------|----------------------------------------------------------------------------|
| `gain_scheduling/`  | `AdaptivePidController` — adjusts K_p/K_i/K_d online from tracking error  |
| `rls/`              | `RlsEstimator` — Recursive Least Squares for online parameter identification|

## Composition

`RlsEstimator` runs in parallel with the controller and feeds updated model parameters
(mass, friction, drag) into `AdaptivePidController` or `MPCController`. Neither module
depends on the other — they communicate through `RobotModelParams` in `common/`.

## Theory

Each sub-module contains its own `docs/theory.md`.
