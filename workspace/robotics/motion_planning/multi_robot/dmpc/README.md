# DMPC — Distributed Model Predictive Control

Each robot **independently** solves an MPC problem at every timestep, using
the last communicated predicted trajectories of its neighbors as **avoidance constraints**.

DMPC bridges the gap between:
- **ORCA:** reactive, fast, no trajectory prediction
- **MADER:** full asynchronous decentralized trajectory optimization

---

## When to Use

| Good fit | Poor fit |
|---|---|
| Continuous environments | No communication budget |
| Robots with known dynamics | Very large fleets ($N > 15$) |
| Predictable trajectories needed | Highly dynamic obstacle-dense scenes |
| Soft constraint on time optimality | Hard real-time (sub-millisecond) |

---

## Interface

```cpp
#include "dmpc/dmpc_agent.hpp"

DmpcConfig cfg;
cfg.horizon    = 20;         // steps
cfg.dt         = 0.05;       // seconds
cfg.v_max      = 1.5;
cfg.a_max      = 0.8;
cfg.sep_min    = 0.6;        // minimum separation between robot centres
cfg.solver     = SolverType::OSQP;

DmpcAgent agent(cfg, robot_id);

// Call once per control cycle:
Trajectory my_traj = agent.solve(
    my_state,                 // current state
    goal_pose,
    neighbor_committed_trajs  // last received trajectories per neighbor
);
agent.broadcast(my_traj);     // share committed trajectory
```

---

## File Layout

```
dmpc/
├── CMakeLists.txt
├── README.md
├── docs/
│   └── theory.md          ← MPC problem, inter-robot constraints, convergence
├── include/
│   └── dmpc/
│       ├── dmpc_agent.hpp
│       ├── dmpc_config.hpp
│       └── trajectory_buffer.hpp    ← ring buffer for received neighbor trajectories
├── src/
│   ├── dmpc_agent.cpp
│   └── trajectory_buffer.cpp
└── tests/
    └── test_dmpc.cpp
```

---

## Dependencies

- `common/`
- `control/mpc/` — the QP formulation here extends the single-robot MPC
- OSQP or qpOASES — quadratic program solver

---

## References

- Luis & Schoellig — *Trajectory Generation for Multiagent Point-To-Point Transitions via Distributed Model Predictive Control*, RAL 2019
- Borrelli, Bemporad & Morari — *Predictive Control for Linear and Hybrid Systems*, Cambridge 2017
