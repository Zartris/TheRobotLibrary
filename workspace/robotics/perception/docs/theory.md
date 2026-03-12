# Theory: Perception — Overview

This document introduces the perception problem and the probabilistic framework used
across all perception sub-modules. Each sub-module has its own `docs/theory.md`.

---

## What is perception?

Perception transforms raw sensor measurements $\mathbf{z}$ into structured knowledge about
the world state $\mathbf{m}$ (obstacle positions, free space, object identities). The key
insight from probabilistic robotics:

> Sensors are noisy. The same world produces slightly different readings every time.
> We model this uncertainty explicitly rather than pretending readings are exact.

---

## The Probabilistic Sensor Model

A sensor model defines:

$$p(\mathbf{z} \mid \mathbf{x}, \mathbf{m})$$

the probability of observing measurement $\mathbf{z}$ given robot pose $\mathbf{x}$ and
map $\mathbf{m}$. This likelihood is used by:
- **State estimators** (EKF, particle filter) to update their pose belief
- **Mapping algorithms** (occupancy grid) to update the map

---

## Sub-module Map

| Sub-module | Answers |
|---|---|
| `lidar_processing/` | What does the raw scan tell us about surfaces nearby? |
| `occupancy_grid/` | What is the probability each cell in the map is occupied? |
| `ray_casting/` | What *would* the sensor see if the map were ground truth? |
| `obstacle_detection/` | Are there obstacles, and where are they? |

---

## Where to go next

- Start with [`../lidar_processing/docs/theory.md`](../lidar_processing/docs/theory.md) for the beam sensor model.
- Then read [`../occupancy_grid/docs/theory.md`](../occupancy_grid/docs/theory.md) for the mapping update rule.
- [`../ray_casting/docs/theory.md`](../ray_casting/docs/theory.md) explains synthetic sensing, used in simulation and for testing state estimators.

---

## Further Reading

- Thrun, Burgard, Fox — *Probabilistic Robotics*, Chapters 5 and 9
