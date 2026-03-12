# Theory: State Estimation — Overview

This document covers the Bayes filter framework that all estimators in this domain
are built on. Each sub-module has its own `docs/theory.md` with detailed derivations.

---

## The Fundamental Problem

The robot state $\mathbf{x}_t$ (pose, velocity, ...) is never directly observable.
It is inferred from:
- **Motion commands** $\mathbf{u}_t$ (what we told the robot to do)
- **Sensor measurements** $\mathbf{z}_t$ (what the sensors report)

The goal is the **belief** — a probability distribution over possible states:
$$\text{bel}(\mathbf{x}_t) = p(\mathbf{x}_t \mid \mathbf{z}_{1:t},\, \mathbf{u}_{1:t})$$

---

## The Bayes Filter

All estimators in this domain are instances of the Bayes filter — a two-step recursive
algorithm:

**Predict** (propagate through motion model $p(\mathbf{x}_t \mid \mathbf{u}_t, \mathbf{x}_{t-1})$):
$$\overline{\text{bel}}(\mathbf{x}_t) = \int p(\mathbf{x}_t \mid \mathbf{u}_t, \mathbf{x}_{t-1})\, \text{bel}(\mathbf{x}_{t-1})\, d\mathbf{x}_{t-1}$$

**Update** (incorporate sensor measurement via likelihood $p(\mathbf{z}_t \mid \mathbf{x}_t)$):
$$\text{bel}(\mathbf{x}_t) = \eta\, p(\mathbf{z}_t \mid \mathbf{x}_t)\, \overline{\text{bel}}(\mathbf{x}_t)$$

The sub-modules differ in how they **represent** the belief distribution:

| Sub-module | Belief representation | Strength |
|---|---|---|
| `ekf/` | Gaussian $\mathcal{N}(\boldsymbol{\mu}, \boldsymbol{\Sigma})$ | Efficient, unimodal |
| `particle_filter/` | Weighted samples $\{\mathbf{x}^{[m]}, w^{[m]}\}$ | Multi-modal, global localization |
| `ekf_slam/` | Gaussian over robot + landmarks | Simultaneous map building |
| `lidar_slam/` | Pose graph + scan matching | Large-scale, loop closure |

---

## Where to Start

- New to probabilistic robotics? Start with [`../ekf/docs/theory.md`](../ekf/docs/theory.md).
- Need global localization (robot wakes up without knowing where it is)? Read [`../particle_filter/docs/theory.md`](../particle_filter/docs/theory.md).
- Building a map and localizing at the same time? Read [`../ekf_slam/docs/theory.md`](../ekf_slam/docs/theory.md).
- Working with lidar-only large environments? Read [`../lidar_slam/docs/theory.md`](../lidar_slam/docs/theory.md).

---

## Further Reading

- Thrun, Burgard, Fox — *Probabilistic Robotics* (the definitive reference for this entire domain)
- Dellaert & Kaess — *Factor Graphs for Robot Perception* (modern graph-based SLAM)
