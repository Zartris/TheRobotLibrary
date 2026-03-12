# Theory: PID Controller

---

## 1. The Continuous PID Law

Given scalar error $e(t) = x^*(t) - x(t)$ (setpoint minus measurement):

$$u(t) = K_p\, e(t) + K_i \int_0^t e(\tau)\,d\tau + K_d\, \frac{de}{dt}$$

| Term | Role |
|---|---|
| $K_p \cdot e$ | **Proportional** — immediate reaction to current error |
| $K_i \int e\, dt$ | **Integral** — eliminates steady-state error over time |
| $K_d \dot{e}$ | **Derivative** — anticipates and damps oscillations |

---

## 2. Discrete Implementation

Running at fixed timestep $\Delta t$, using backward Euler integration:

$$u_k = K_p\, e_k + K_i \underbrace{\sum_{j=0}^{k} e_j\,\Delta t}_{I_k} + K_d\,\frac{e_k - e_{k-1}}{\Delta t}$$

Incremental update (avoids storing all past errors):

$$I_k = I_{k-1} + e_k \cdot \Delta t$$

---

## 3. Practical Issues

### Integral windup
When the actuator is saturated (output is clamped), the integral term keeps growing even
though the robot cannot respond. This causes overshoot when the constraint is removed.

**Fix:** Clamp the integral accumulator:
$$I_k \leftarrow \text{clamp}(I_k,\; -I_{\max},\; I_{\max})$$

Or use **conditional integration** — only accumulate when the output is not saturated.

### Derivative kick
When the setpoint suddenly jumps, $\dot{e}$ spikes, causing a large impulse output.

**Fix:** Compute the derivative on the **measurement** instead of the error:
$$D_k = -K_d\,\frac{x_k - x_{k-1}}{\Delta t}$$

The sign flip accounts for $e = x^* - x$, so $\dot{e} = -\dot{x}$ when $\dot{x}^* = 0$.

### Derivative filtering
The derivative term amplifies measurement noise. Low-pass filter it with time constant $\tau_f$:

$$D_k = \frac{\tau_f}{\tau_f + \Delta t} D_{k-1} - \frac{K_d}{\tau_f + \Delta t}(x_k - x_{k-1})$$

---

## 4. Tuning Guidelines

**Ziegler-Nichols (empirical starting point):**
1. Set $K_i = K_d = 0$. Increase $K_p$ until the output oscillates steadily → record $K_u$ and period $T_u$.
2. Apply the Ziegler-Nichols table:

| Type | $K_p$ | $K_i$ | $K_d$ |
|---|---|---|---|
| P | $0.5 K_u$ | 0 | 0 |
| PI | $0.45 K_u$ | $1.2 K_p / T_u$ | 0 |
| PID | $0.6 K_u$ | $2 K_p / T_u$ | $K_p T_u / 8$ |

These are starting points — always refine on the real system.

---

## Further Reading

- Åström & Wittenmark — *Computer-Controlled Systems* (Chapter 3–5)
- Seborg, Edgar et al. — *Process Dynamics and Control*
