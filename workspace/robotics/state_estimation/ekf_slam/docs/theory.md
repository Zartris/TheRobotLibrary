# Theory: EKF-SLAM

---

## 1. The SLAM Problem

SLAM answers: *where am I, and what does the map look like?* It is harder than localization
(known map) because the robot must build the map and use it simultaneously — errors in pose
corrupt the map estimate, and errors in the map corrupt the pose estimate.

---

## 2. Augmented State Vector

EKF-SLAM operates on a joint state over the robot pose and all $N$ landmark positions:

$$\mathbf{X}_t = \begin{pmatrix} \mathbf{x}_t \\ \mathbf{m}_1 \\ \vdots \\ \mathbf{m}_N \end{pmatrix} \in \mathbb{R}^{3+2N}$$

The joint covariance $\boldsymbol{\Sigma}_t \in \mathbb{R}^{(3+2N)\times(3+2N)}$ encodes
correlations between the robot pose and every landmark.

**Key insight:** when the robot sees a known landmark, the update step improves *both* the robot
pose estimate and the landmark estimate simultaneously — and all other landmark estimates
improve too, because they are all correlated through the map.

---

## 3. Predict Step

Only the robot pose evolves; landmarks are static:

$$G_t = \begin{pmatrix} G_{\mathbf{x}} & 0 \\ 0 & I_{2N} \end{pmatrix}$$

where $G_{\mathbf{x}}$ is the $3\times 3$ robot motion Jacobian (same as `ekf/`).

$$\bar{\boldsymbol{\Sigma}}_t = G_t\boldsymbol{\Sigma}_{t-1}G_t^\top + R_t$$

where $R_t$ has non-zero entries only in the $3\times 3$ robot block.

---

## 4. Update Step (Known Landmark)

For a range-bearing observation of landmark $k$:

$$\mathbf{z}_k = h_k(\mathbf{X}) + \mathbf{v}_k, \quad h_k(\mathbf{X}) = \begin{pmatrix} r_k \\ \phi_k \end{pmatrix}$$

$H_k$ is a $2 \times (3+2N)$ matrix with non-zeros only in the robot pose columns and
landmark $k$'s columns.

The EKF update is the standard formula with this $H_k$.

---

## 5. Adding a New Landmark

When an observation cannot be matched to any existing landmark (beyond gating threshold),
it is initialized as a new landmark:

$$\mathbf{m}_{\text{new}} = \begin{pmatrix} x + r\cos(\theta + \phi) \\ y + r\sin(\theta + \phi) \end{pmatrix}$$

The state vector is augmented from size $3+2N$ to $3+2(N+1)$, and the covariance
is extended accordingly using the Jacobian of the initialization function.

---

## 6. Data Association

Each new observation must be matched to an existing landmark or declared new.
**Mahalanobis distance** with a $\chi^2$ gate is the standard test:

$$d^2 = (\mathbf{z} - h(\hat{\mathbf{x}}))^\top S^{-1} (\mathbf{z} - h(\hat{\mathbf{x}}))$$

where $S = H\bar{\boldsymbol{\Sigma}}H^\top + Q$ is the innovation covariance.
Match to the landmark with smallest $d^2$ within the gate; declare new if no match.

---

## 7. Computational Complexity

Each update step requires inverting the $2\times 2$ innovation covariance $S$ and
multiplying matrices involving $\boldsymbol{\Sigma} \in \mathbb{R}^{(3+2N)\times(3+2N)}$:
cost is $O(N^2)$ per update. For $N > 200$, consider sparse methods or graph SLAM.

---

## Further Reading

- Thrun, Burgard, Fox — *Probabilistic Robotics*, Chapter 10
- Dissanayake et al. — *A Solution to the Simultaneous Localization and Map Building Problem*, TRA 2001
