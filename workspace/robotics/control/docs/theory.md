# Theory: Robot Control — Overview

This document introduces feedback control for mobile robots at a high level.
Each sub-module has its own `docs/theory.md` with the detailed mathematics.

---

## What is a controller?

A controller drives the robot from its current state $\mathbf{x}$ to a desired state
$\mathbf{x}^*$ by computing control actions $\mathbf{u}$ that reduce the error:

$$e(t) = \mathbf{x}^*(t) - \mathbf{x}(t)$$

The three controllers in this domain represent a progression in capability:

| Controller    | Input            | Output          | Accounts for dynamics? |
|---------------|------------------|-----------------|------------------------|
| PID           | Scalar error     | Scalar command  | No — pure error feedback |
| Pure pursuit  | Path + pose      | $(v, \omega)$   | No — geometric only    |
| MPC           | State + full model | $(v, \omega)$ | Yes — predicts future  |

---

## Feedback control loop

```
     goal/reference
          │
          ▼
      [−] error ───►  Controller  ──►  Robot  ──►  state
          ▲                                          │
          └───────────────────────────────────────────────┘
                              feedback
```

---

## Where to go next

- New to control? Read [`../pid/docs/theory.md`](../pid/docs/theory.md) — PID is the
  foundation everything else builds on.
- Tracking a path? Read [`../pure_pursuit/docs/theory.md`](../pure_pursuit/docs/theory.md).
- Need to respect physical constraints or optimize a cost? Read
  [`../mpc/docs/theory.md`](../mpc/docs/theory.md).

---

## Further Reading

- Åström & Wittenmark — *Computer-Controlled Systems* (PID and discrete control theory)
- Rawlings, Mayne, Diehl — *Model Predictive Control: Theory, Computation, and Design*
