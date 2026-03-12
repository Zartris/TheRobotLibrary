# Theory: Distributed MPC (DMPC) for Multi-Robot Planning

---

## 1. Single-Robot MPC Recap

A single robot (diff-drive, linearized) descends the cost:
$$J_i = \sum_{k=0}^{H-1}\left[(x_i^k - x_i^g)^\top Q (x_i^k - x_i^g) + u_i^{k\top} R\, u_i^k\right] + (x_i^H - x_i^g)^\top P_f (x_i^H - x_i^g)$$

subject to dynamics $x_i^{k+1} = A x_i^k + B u_i^k$, input limits $|u| \le u_{\max}$.
(See `control/mpc/docs/theory.md` for the full single-robot derivation.)

---

## 2. Inter-Robot Avoidance Constraint

For each neighbor $j$, add the constraint at each prediction horizon step $k$:

$$\|x_i^k - x_j^k\|_2 \ge d_{\min}$$

This is a **nonlinear** (non-convex) constraint. Two common approaches:

### 2a. Linearized (Sequential Convex Programming — SCP)

At iteration $\ell$, linearize around the previous iterate $\bar{x}_{ij}^k = x_i^{k,(\ell-1)} - x_j^k$:

$$\hat{n}^k = \frac{\bar{x}_{ij}^k}{\|\bar{x}_{ij}^k\|}, \quad d_{\min} \le \hat{n}^k \cdot (x_i^k - x_j^k)$$

This is a **linear constraint** — the full problem remains a QP. Iterate to convergence.

### 2b. Penalty / Barrier

Add a soft penalty to the cost:
$$J_{\text{sep}} = \sum_{k,j} \max\!\left(0,\; d_{\min}^2 - \|x_i^k - x_j^k\|^2\right)^2$$

No hard guarantee, but smooth — suitable for gradient-based solvers (CasADi + IPOPT).

---

## 3. Distributed Structure

In **centralized** MPC, all robots' QPs are bundled into one large optimization:
feasibility and optimality guaranteed, complexity $O((N H n)^3)$ per step.

In **distributed** MPC:
1. At time $t$: robot $i$ receives neighbor $j$'s **committed trajectory** from time $t$ onwards.
2. Robot $i$ forms avoidance constraints using committed trajectories of all neighbors.
3. Robot $i$ solves its own QP (size $O(H \cdot n)$), obtains a new trajectory.
4. Robot $i$ broadcasts the first several steps of its new trajectory as its committed segment.

**Key assumption:** neighbor trajectories change slowly enough that the constraint formed
from the last received trajectory is still approximately valid when robot $i$'s new plan
is executed. This holds when the control frequency is high and robots move slowly relative
to the sampling period.

---

## 4. Convergence and Feasibility

**Theorem (Luis & Schoellig 2019):** under bounded communication delay and sufficiently
short horizon, the DMPC iteration converges to a fixed point where no avoidance constraint
is violated, provided the **initial committed trajectories are collision-free**.

**Practical startup:** initialize with a set of collision-free hover/stop trajectories,
then let the MPC guide each robot toward its goal.

**Recursive feasibility:** if the MPC adds a terminal constraint (a positively invariant
set around the goal), the problem remains feasible at each step — i.e., a safe trajectory
always exists.

---

## 5. Communication Requirements

| | ORCA | DMPC |
|---|---|---|
| Data shared | Current state (pos + vel) | Full predicted trajectory ($H \times n$ floats) |
| Frequency | Every control step | Every control step |
| Latency tolerance | Low | Moderate (1–2 steps) |
| Bandwidth per agent | $O(N)$ floats/step | $O(N \cdot H)$ floats/step |

For $H=20$, $n=4$ (pose + vel), $N=10$ neighbors: $10 \times 20 \times 4 = 800$ floats
per step. At 20 Hz: 128 KB/s per robot — manageable over WiFi.

---

## Further Reading

- Luis & Schoellig — *Trajectory Generation for Multiagent Point-To-Point Transitions via Distributed Model Predictive Control*, RAL 2019
- Borrelli, Bemporad & Morari — *Predictive Control for Linear and Hybrid Systems*, Cambridge 2017
- Kamel et al. — *Voliro: An Omnidirectional Hexarotor...*, RAL 2018 (practical DMPC example)
- OSQP solver: <https://osqp.org/>
