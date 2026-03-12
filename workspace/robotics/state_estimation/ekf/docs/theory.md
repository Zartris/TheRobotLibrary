# Theory: Extended Kalman Filter (EKF)

---

## 1. Kalman Filter (Linear Case)

The Kalman filter is the optimal Bayes filter for **linear Gaussian** systems.
State $\mathbf{x}_t \in \mathbb{R}^n$, control $\mathbf{u}_t$, measurement $\mathbf{z}_t$:

$$\mathbf{x}_t = A\mathbf{x}_{t-1} + B\mathbf{u}_t + \mathbf{w}_t, \quad \mathbf{w}_t \sim \mathcal{N}(0, R)$$
$$\mathbf{z}_t = H\mathbf{x}_t + \mathbf{v}_t, \quad \mathbf{v}_t \sim \mathcal{N}(0, Q)$$

The belief is always a Gaussian $\mathcal{N}(\boldsymbol{\mu}_t, \boldsymbol{\Sigma}_t)$.

**Predict:**
$$\bar{\boldsymbol{\mu}}_t = A\boldsymbol{\mu}_{t-1} + B\mathbf{u}_t$$
$$\bar{\boldsymbol{\Sigma}}_t = A\boldsymbol{\Sigma}_{t-1}A^\top + R$$

**Update:**
$$K_t = \bar{\boldsymbol{\Sigma}}_t H^\top (H\bar{\boldsymbol{\Sigma}}_t H^\top + Q)^{-1}$$
$$\boldsymbol{\mu}_t = \bar{\boldsymbol{\mu}}_t + K_t(\mathbf{z}_t - H\bar{\boldsymbol{\mu}}_t)$$
$$\boldsymbol{\Sigma}_t = (I - K_t H)\bar{\boldsymbol{\Sigma}}_t$$

$K_t$ is the **Kalman gain** — it balances trust in the prediction vs. the measurement.
When $Q \to 0$ (perfect sensor), $K_t \to H^{-1}$ and the measurement is trusted fully.

---

## 2. Extension to Nonlinear Systems (EKF)

Robot kinematics and sensor models are **nonlinear**. The EKF linearizes them with
first-order Taylor expansion (Jacobians).

Nonlinear system:
$$\mathbf{x}_t = g(\mathbf{x}_{t-1}, \mathbf{u}_t) + \mathbf{w}_t$$
$$\mathbf{z}_t = h(\mathbf{x}_t) + \mathbf{v}_t$$

Jacobians at the current estimate:
$$G_t = \frac{\partial g}{\partial \mathbf{x}}\bigg|_{\boldsymbol{\mu}_{t-1}, \mathbf{u}_t}, \qquad H_t = \frac{\partial h}{\partial \mathbf{x}}\bigg|_{\bar{\boldsymbol{\mu}}_t}$$

**EKF Predict:**
$$\bar{\boldsymbol{\mu}}_t = g(\boldsymbol{\mu}_{t-1}, \mathbf{u}_t)$$
$$\bar{\boldsymbol{\Sigma}}_t = G_t\boldsymbol{\Sigma}_{t-1}G_t^\top + R_t$$

**EKF Update:**
$$K_t = \bar{\boldsymbol{\Sigma}}_t H_t^\top (H_t\bar{\boldsymbol{\Sigma}}_t H_t^\top + Q_t)^{-1}$$
$$\boldsymbol{\mu}_t = \bar{\boldsymbol{\mu}}_t + K_t\bigl(\mathbf{z}_t - h(\bar{\boldsymbol{\mu}}_t)\bigr)$$
$$\boldsymbol{\Sigma}_t = (I - K_t H_t)\bar{\boldsymbol{\Sigma}}_t$$

---

## 3. Differential-Drive Motion Model Jacobian

State $\mathbf{x} = (x, y, \theta)^\top$, control $\mathbf{u} = (v, \omega)^\top$:

$$g(\mathbf{x}, \mathbf{u}) = \mathbf{x} + \Delta t\begin{pmatrix} v\cos\theta \\ v\sin\theta \\ \omega \end{pmatrix}$$

Jacobian:
$$G = \frac{\partial g}{\partial \mathbf{x}} = \begin{pmatrix} 1 & 0 & -v\Delta t\sin\theta \\ 0 & 1 & v\Delta t\cos\theta \\ 0 & 0 & 1 \end{pmatrix}$$

---

## 4. Range-Bearing Observation Model Jacobian

Observation of landmark $k$ at position $(m_{k,x}, m_{k,y})$:

$$h_k(\mathbf{x}) = \begin{pmatrix} \sqrt{(m_{k,x}-x)^2+(m_{k,y}-y)^2} \\ \text{atan2}(m_{k,y}-y,\, m_{k,x}-x) - \theta \end{pmatrix}$$

The Jacobian $H_k$ is derived by differentiating each component w.r.t. $(x, y, \theta)$.
See Thrun et al. Table 7.3 for the explicit expression.

---

## 5. When the EKF Fails

The EKF assumes the posterior is well-approximated by a Gaussian. This breaks when:
- The initial pose is unknown (multi-modal prior) — use particle filter instead
- Highly nonlinear dynamics cause the Jacobian approximation to be poor
- There are data association ambiguities (multiple possible landmark matches)

---

## Further Reading

- Thrun, Burgard, Fox — *Probabilistic Robotics*, Chapters 3 and 7
- Welch & Bishop — *An Introduction to the Kalman Filter*, UNC TR 95-041 (gentle introduction)
