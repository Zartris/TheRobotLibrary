# MPC -- Model Predictive Control: Zero to Hero

## 1. What is MPC?

Model Predictive Control (MPC) is a control strategy that **optimizes future behavior**
by solving a finite-horizon optimal control problem at every timestep. The key idea:

1. **Predict** the system trajectory over the next N steps
2. **Optimize** a control sequence that minimizes a cost function subject to constraints
3. **Apply** only the first control action
4. **Repeat** at the next timestep with updated state measurements

This "receding horizon" approach lets the controller continuously replan as new
information arrives, naturally handling constraints, multi-variable coupling, and
look-ahead planning.

```
t        t+1      t+2      ...    t+N
|---------|---------|---------|-----|
  apply    predict (optimize over this window)
  u*_t
```

The key insight: even though we compute N future controls, we only apply the first one.
At the next timestep, we shift the horizon forward and re-optimize with fresh state data.
This feedback mechanism provides robustness to model errors and disturbances.

---

## 2. Mathematical Formulation

### Continuous-Time State Space

A dynamical system in continuous time:

$$\dot{\mathbf{x}}(t) = f(\mathbf{x}(t), \mathbf{u}(t))$$

where $\mathbf{x} \in \mathbb{R}^{n_x}$ is the state and $\mathbf{u} \in \mathbb{R}^{n_u}$
is the control input.

### Discrete-Time State Space

For digital control, we discretize with timestep $\Delta t$:

$$\mathbf{x}_{k+1} = f_d(\mathbf{x}_k, \mathbf{u}_k)$$

Common discretization methods:
- **Forward Euler:** $\mathbf{x}_{k+1} = \mathbf{x}_k + \Delta t \cdot f(\mathbf{x}_k, \mathbf{u}_k)$ (simple, first-order accurate)
- **Runge-Kutta 4 (RK4):** Fourth-order accurate, more computation per step
- **Exact discretization:** For linear systems, $\mathbf{x}_{k+1} = e^{A \Delta t} \mathbf{x}_k + \int_0^{\Delta t} e^{A\tau} B \, d\tau \, \mathbf{u}_k$

### The Optimal Control Problem (OCP)

$$\min_{\mathbf{u}_0,\ldots,\mathbf{u}_{N-1}} \sum_{k=0}^{N-1} \ell(\mathbf{x}_k, \mathbf{u}_k) + V_f(\mathbf{x}_N)$$

Subject to:
- **Dynamics:** $\mathbf{x}_{k+1} = f_d(\mathbf{x}_k, \mathbf{u}_k)$
- **Input constraints:** $\mathbf{u}_{\min} \le \mathbf{u}_k \le \mathbf{u}_{\max}$
- **State constraints:** $\mathbf{x}_k \in \mathcal{X}$
- **Initial condition:** $\mathbf{x}_0 = \mathbf{x}_{\text{measured}}$

### Stage Cost

Typically quadratic, penalizing tracking error and control effort:

$$\ell(\mathbf{x}, \mathbf{u}) = (\mathbf{x} - \mathbf{x}^{\text{ref}})^\top Q (\mathbf{x} - \mathbf{x}^{\text{ref}}) + \mathbf{u}^\top R \, \mathbf{u}$$

- $Q \succeq 0$ weights state deviation (how much we care about tracking accuracy)
- $R \succ 0$ weights control effort (penalizes large commands, promotes smoothness)

### Terminal Cost

$$V_f(\mathbf{x}_N) = (\mathbf{x}_N - \mathbf{x}^{\text{ref}}_N)^\top Q_f (\mathbf{x}_N - \mathbf{x}^{\text{ref}}_N)$$

The terminal cost stabilizes the closed-loop system. A principled choice: $Q_f$ equals the
infinite-horizon LQR cost matrix for the linearized system around the goal. In practice,
$Q_f$ is often set to a scaled version of $Q$ (e.g., $Q_f = 10 \cdot Q$).

### QP Formulation (Linear MPC)

When dynamics are linear ($f_d(\mathbf{x}, \mathbf{u}) = A\mathbf{x} + B\mathbf{u}$) and
the cost is quadratic, the OCP reduces to a Quadratic Program (QP):

$$\min_{\mathbf{U}} \frac{1}{2} \mathbf{U}^\top H \mathbf{U} + \mathbf{f}^\top \mathbf{U}$$

subject to linear inequality constraints. This can be solved very efficiently.

### NLP Formulation (Nonlinear MPC)

When dynamics are nonlinear, we get a Nonlinear Program (NLP). Common solution methods:
- **Direct Multiple Shooting:** Discretize both states and controls; enforce dynamics as equality constraints
- **Direct Collocation:** Use polynomial approximation within each interval
- **Single Shooting:** Propagate dynamics forward, optimize over controls only

---

## 3. Linear vs Nonlinear MPC

### LTV-MPC (Linear Time-Varying MPC)

Approach: linearize the nonlinear dynamics around the current state/trajectory to obtain
time-varying linear matrices $A_k$, $B_k$ at each prediction step. The resulting OCP
is a QP, solvable in milliseconds.

**Condensed formulation:**

Substitute the dynamics constraints to express all future states in terms of the initial
state and control sequence:

$$\mathbf{X} = S \cdot \mathbf{x}_0 + T \cdot \mathbf{U}$$

where $S$ encodes the free response and $T$ the control-to-state mapping. This reduces
the QP to:

$$H = T^\top \bar{Q} T + \bar{R}, \qquad \mathbf{f} = T^\top \bar{Q} (S \mathbf{x}_0 - \mathbf{X}^{\text{ref}})$$

**Pros:** Fast (Cholesky solve), always available, no external dependencies.
**Cons:** Linear approximation may be poor for large heading changes; no hard constraint
handling in the unconstrained version.

### NMPC (Nonlinear MPC)

Approach: solve the full nonlinear OCP directly. The dynamics are not linearized; the
solver handles the nonlinearity internally (e.g., via sequential quadratic programming).

**Pros:** Exact dynamics, handles strong nonlinearities, proper constraint handling.
**Cons:** More complex, requires a nonlinear solver, harder to guarantee real-time feasibility.

### When to Use Which

- **LTV-MPC:** Good for small perturbations around a reference trajectory, fast prototyping,
  environments where installing a solver is impractical
- **NMPC:** Required when the robot operates far from any linearization point (large turns,
  aggressive maneuvers), when hard constraints must be satisfied exactly

---

## 4. The acados Framework

acados is an open-source software package for fast optimal control. It combines several
high-performance components into a unified solver for real-time NMPC.

### Architecture

```
+-------------------+     +-------------------+     +-------------------+
|    CASAdi          |     |    BLASFEO         |     |    HPIPM          |
|    (symbolic math, |     |    (optimized      |     |    (interior-point|
|     auto-diff)     |     |     linear algebra |     |     QP solver for |
|                    |     |     for small dense |     |     optimal       |
|                    |     |     matrices)       |     |     control)      |
+--------+-----------+     +---------+----------+     +---------+---------+
         |                           |                          |
         +---------------------------+--------------------------+
                                     |
                          +----------+----------+
                          |       acados        |
                          |  (OCP solver,       |
                          |   code generation,  |
                          |   C API)            |
                          +---------------------+
```

- **CASAdi:** Symbolic math framework providing automatic differentiation. Used to define
  the dynamics, cost, and constraints symbolically. acados uses CASAdi to generate efficient
  C code for function evaluations and their derivatives.

- **BLASFEO:** High-performance linear algebra library optimized for the small dense matrices
  (typically 3x3 to 20x20) that arise in optimal control. Uses SIMD instructions and
  cache-friendly memory layouts. Significantly faster than BLAS/LAPACK for these sizes.

- **HPIPM:** Interior-point method QP solver designed specifically for the banded structure
  of optimal control problems. Exploits the fact that QP subproblems in MPC have a
  stage-wise structure, achieving $O(N)$ complexity instead of $O(N^3)$.

- **acados:** Ties everything together. Provides:
  - Python interface for OCP definition
  - Code generation for standalone C solvers
  - Multiple NLP algorithms (SQP, SQP-RTI)
  - Multiple QP solvers (HPIPM, qpOASES, OSQP)
  - Multiple integrators (ERK, IRK)

### Installation

```bash
# 1. Install CASAdi (Python)
pip install casadi

# 2. Clone and build acados
git clone https://github.com/acados/acados.git
cd acados && git submodule update --init --recursive
mkdir build && cd build
cmake -DACADOS_WITH_QPOASES=ON -DCMAKE_BUILD_TYPE=Release ..
make install -j4

# 3. Install acados Python interface
pip install -e ../interfaces/acados_template

# 4. Set environment variable
export ACADOS_DIR=/path/to/acados

# 5. Verify
python -c "from acados_template import AcadosOcp; print('OK')"
```

### Code Generation Workflow

The workflow separates model definition (Python) from solver execution (C/C++):

```
+-----------+      +-------------+      +------------------+      +-----------+
|  Python   |      |   acados    |      |  Generated C     |      |   C++     |
|  script   | ---> |   template  | ---> |  solver code     | ---> |  wrapper  |
|  (CASAdi) |      |   (codegen) |      |  (committed to   |      |  (calls   |
|           |      |             |      |   src/generated/) |      |   C API)  |
+-----------+      +-------------+      +------------------+      +-----------+
```

1. **Define model in CASAdi:** Write symbolic dynamics, cost expressions, and constraints
   using CASAdi's `SX` symbolic type
2. **Define OCP in acados Python API:** Set dimensions, cost weights, solver options,
   constraint bounds
3. **Generate C solver code:** `AcadosOcpSolver(ocp)` produces standalone C code that
   implements the solver for this specific OCP
4. **Write C++ wrapper:** Call the generated C API (`create_capsule`, `solve`, `get`) from
   your C++ module
5. **Commit generated code:** Users don't need Python or acados to build; they only need
   acados libraries at link time

### Key Concepts

**SQP-RTI (Sequential QP -- Real-Time Iteration):**
Instead of running SQP to convergence at each control step (which may take many iterations),
RTI performs exactly one SQP iteration per control cycle. The preparation phase (linearization,
QP setup) runs between control cycles; the feedback phase (QP solve) runs when the new state
measurement arrives. This splits the computation into a predictable, bounded-time operation.

**HPIPM (High-Performance Interior Point Method):**
A QP solver that exploits the banded structure of optimal control problems. Standard QP
solvers treat the problem as a generic dense or sparse QP. HPIPM recognizes the stage-wise
coupling (each stage only depends on the previous one) and uses Riccati-like recursions,
achieving linear complexity in the horizon length $N$.

**ERK (Explicit Runge-Kutta) integrator:**
Used to discretize the continuous-time dynamics within each prediction interval. acados
supports multiple RK methods (RK4 is common). For stiff systems, implicit integrators (IRK)
are available.

---

## 5. The Eigen Fallback (LTV-MPC)

When acados is not installed, this module uses a pure-Eigen LTV-MPC implementation.

### Condensed Formulation

Given linearized dynamics $\mathbf{x}_{k+1} = A \mathbf{x}_k + B \mathbf{u}_k$:

1. **Build prediction matrices:**
   - $S$: maps initial state to future states (free response)
   - $T$: maps control sequence to future states (forced response)

2. **Form QP:**
   - $H = T^\top \bar{Q} T + \bar{R}$ (Hessian, positive definite)
   - $\mathbf{f} = T^\top \bar{Q} (S \mathbf{x}_0 - \mathbf{X}^{\text{ref}})$ (gradient)

3. **Solve via Cholesky:** $\mathbf{U}^* = -H^{-1} \mathbf{f}$ using `Eigen::LLT`

4. **Post-hoc clamping:** Clamp controls to box constraints (not optimal, but simple)

### Limitations

- **Linear approximation:** Accuracy degrades for large heading changes
- **No hard constraints:** Post-hoc clamping is not the same as constrained optimization;
  the trajectory prediction does not account for the clamped inputs
- **Single linearization point:** Uses one linearization for the entire horizon (could
  iterate to improve, at the cost of computation time)

---

## 6. Unicycle Model

The unicycle (differential-drive) model is used throughout this module.

### State and Control

- State: $\mathbf{x} = (p_x, p_y, \theta)^\top$ -- position and heading
- Control: $\mathbf{u} = (v, \omega)^\top$ -- linear velocity and angular velocity

### Continuous Dynamics

$$\dot{p}_x = v \cos\theta, \qquad \dot{p}_y = v \sin\theta, \qquad \dot{\theta} = \omega$$

### Discrete Dynamics (Forward Euler)

$$\mathbf{x}_{k+1} = \mathbf{x}_k + \Delta t \begin{pmatrix} v_k \cos\theta_k \\ v_k \sin\theta_k \\ \omega_k \end{pmatrix}$$

### Jacobians (for LTV-MPC)

$$A = \frac{\partial f_d}{\partial \mathbf{x}} = \begin{pmatrix} 1 & 0 & -v \sin\theta \cdot \Delta t \\ 0 & 1 & v \cos\theta \cdot \Delta t \\ 0 & 0 & 1 \end{pmatrix}$$

$$B = \frac{\partial f_d}{\partial \mathbf{u}} = \begin{pmatrix} \cos\theta \cdot \Delta t & 0 \\ \sin\theta \cdot \Delta t & 0 \\ 0 & \Delta t \end{pmatrix}$$

### Why Not a More Complex Model?

The unicycle model captures the essential nonlinearity (heading-dependent motion) while
being simple enough for real-time control. For higher-fidelity control, one could add:
- **Acceleration limits** (extend state to include velocity)
- **Tire slip** (bicycle or dynamic model)
- **Full 3D dynamics** (for UAVs or legged robots)

The acados codegen script can be modified to use any of these models.

---

## 7. Tuning Guide

### Horizon Length N

- **Too short (N < 5):** Controller is myopic; cannot plan around curves
- **Too long (N > 50):** Computationally expensive; diminishing returns
- **Sweet spot:** 10--30 steps, depending on the task and available compute budget
- **Rule of thumb:** Choose $N$ so that $N \cdot \Delta t$ covers the relevant planning
  distance (e.g., 2--5 seconds of lookahead)

### Cost Weights Q, R

- **Large Q:** Aggressive tracking, may cause oscillation or large control inputs
- **Large R:** Smooth, conservative control; may track poorly
- **Q_theta vs Q_position:** If heading tracking is less important than position tracking,
  reduce the theta weight (e.g., $Q = \text{diag}(10, 10, 1)$)
- **Start with:** $Q = I$, $R = 0.01 \cdot I$, then adjust based on behavior

### Terminal Weight Qf

- Typically $Q_f \gg Q$ (e.g., $10\times$ to $100\times$) to encourage reaching the goal
- Too large: can cause aggressive end-of-horizon behavior
- Can be computed from the discrete-time algebraic Riccati equation (DARE) for optimal
  stability guarantees

### Prediction Time T

- $T = N \cdot \Delta t$ is the total lookahead time
- Should cover the relevant planning horizon for the task
- For trajectory tracking: $T \approx$ time to traverse the next curve
- For point-to-point: $T \approx$ time to reach the goal at nominal speed

### SQP Iterations (acados)

- **SQP-RTI (1 iteration):** Fastest, but may not fully converge. Works well when the
  solution changes slowly between timesteps (warm-starting helps)
- **Full SQP (multiple iterations):** Better optimality, but slower. Use when the problem
  changes significantly between timesteps

---

## 8. Comparison with Other Approaches

| Approach | Pros | Cons |
|----------|------|------|
| PID | Simple, fast, well-understood | No prediction, reactive only, hard to tune for MIMO |
| Pure Pursuit | Geometric, smooth paths | No obstacle handling, no constraints, speed-dependent |
| Stanley | Good lateral tracking | Longitudinal control separate, no constraints |
| MPC (Eigen) | Predictive, always available, no deps | Linear approximation, no hard constraints |
| MPC (acados) | Full nonlinear, real-time, hard constraints | Requires acados installation |
| MPC (IPOPT) | Full nonlinear, general NLP | Slower than acados, not real-time guaranteed |

---

## 9. Further Reading

### Textbooks
- Rawlings, Mayne, Diehl -- *Model Predictive Control: Theory, Computation, and Design* (2nd ed.)
- Borrelli, Bemporad, Morari -- *Predictive Control for Linear and Hybrid Systems*
- Grune, Pannek -- *Nonlinear Model Predictive Control*

### Papers
- Verschueren et al. -- *acados -- a modular open-source framework for fast embedded optimal control* (2021)
- Frison, Jorgensen -- *HPIPM: a high-performance quadratic programming framework for
  model predictive control* (2020)
- Kong et al. -- *Kinematic and Dynamic Vehicle Models for Autonomous Driving Control Design*, IV 2015

### Online Resources
- acados documentation: https://docs.acados.org/
- CASAdi documentation: https://web.casadi.org/docs/
- HPIPM: https://github.com/giaf/hpipm
- BLASFEO: https://github.com/giaf/blasfeo
