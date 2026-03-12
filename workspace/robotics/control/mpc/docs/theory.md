# Theory: Model Predictive Control (MPC)

MPC is a control strategy that **optimizes future behavior** by solving a finite-horizon
optimal control problem at every timestep, applying only the first action, then repeating
with the new state. This "receding horizon" principle lets the controller continuously
replan as new information arrives.

---

## 1. The Receding Horizon Idea

At time $t$, MPC predicts the system trajectory over the next $N$ steps and finds the
control sequence $\mathbf{u}_t, \ldots, \mathbf{u}_{t+N-1}$ that minimizes a cost.
Only $\mathbf{u}_t^*$ is applied. At $t+1$ the horizon shifts forward and the optimization
repeats with the updated state.

```
t        t+1      t+2      ...    t+N
|---------|---------|---------|-----|
  apply    predict (optimize over this window)
  u*_t
```

---

## 2. Problem Formulation

$$\min_{\mathbf{u}_t,\ldots,\mathbf{u}_{t+N-1}} \sum_{k=0}^{N-1} \ell(\mathbf{x}_{t+k}, \mathbf{u}_{t+k}) + V_f(\mathbf{x}_{t+N})$$

Subject to:
- **Dynamics:** $\mathbf{x}_{k+1} = f(\mathbf{x}_k, \mathbf{u}_k)$
- **Input constraints:** $\mathbf{u}_{\min} \le \mathbf{u}_k \le \mathbf{u}_{\max}$
- **State constraints:** $\mathbf{x}_k \in \mathcal{X}$

### Stage cost $\ell$

Typically a quadratic penalizing tracking error and control effort:

$$\ell(\mathbf{x}, \mathbf{u}) = (\mathbf{x} - \mathbf{x}^*)^\top Q (\mathbf{x} - \mathbf{x}^*) + \mathbf{u}^\top R\, \mathbf{u}$$

$Q \succeq 0$ weights state deviation; $R \succ 0$ weights control effort (penalizes large commands).

### Terminal cost $V_f$

The terminal cost stabilizes the closed-loop system. A common choice: the infinite-horizon
LQR cost for the linearized system around the goal.

---

## 3. Differential-Drive Robot Model

For a differential-drive robot with state $\mathbf{x} = (x, y, \theta)^\top$ and
control $\mathbf{u} = (v, \omega)^\top$:

$$\mathbf{x}_{k+1} = \mathbf{x}_k + \Delta t \begin{pmatrix} v_k \cos\theta_k \\ v_k \sin\theta_k \\ \omega_k \end{pmatrix}$$

This is nonlinear in $\theta$. Two approaches:
- **Linearize** around the current state for each horizon step → solve a QP (fast)
- **Solve the nonlinear program** directly with e.g. CasADi or IPOPT (more accurate, slower)

---

## 4. Why MPC over PID / Pure Pursuit?

| Capability | PID | Pure Pursuit | MPC |
|---|---|---|---|
| Constraint handling | No | No | Yes |
| Multi-variable | Hard to tune | No | Yes |
| Predictive | No | No | Yes |
| Cost | Trivial | Trivial | Needs solver |

MPC is the right choice when you need to respect speed/acceleration limits, optimize
energy use, or plan around time-varying obstacles. The cost is computational — the
optimization must solve within the control timestep.

---

## Further Reading

- Rawlings, Mayne, Diehl — *Model Predictive Control: Theory, Computation, and Design* (2nd ed.)
- Borrelli, Bemporad, Morari — *Predictive Control for Linear and Hybrid Systems*
- Kong et al. — *Kinematic and Dynamic Vehicle Models for Autonomous Driving Control Design*, IV 2015
