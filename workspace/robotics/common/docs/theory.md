# Theory: Mathematical Representations in 2D Robotics

This document covers the mathematical foundations used throughout the `common` module and
the rest of TheRobotLibrary. Understanding these representations is prerequisite reading
before working with any of the other modules.

---

## 1. Coordinate Frames and Poses

A **pose** describes the position and orientation of a rigid body in the world.
In 2D, a pose has three degrees of freedom:

$$\mathbf{x} = \begin{pmatrix} x \\ y \\ \theta \end{pmatrix}$$

where $(x, y)$ is the position in the world frame and $\theta$ is the heading angle
(counterclockwise from the positive x-axis), measured in radians.

### Angle convention

All angles are in radians. The canonical range is $(-\pi, \pi]$.
Angle wrapping is required whenever angles are added or differenced:

$$\theta_{\text{wrapped}} = \text{atan2}(\sin\theta,\, \cos\theta)$$

---

## 2. Rigid Body Transformations

A **transform** $T_{A}^{B}$ maps coordinates from frame $A$ to frame $B$.
In 2D, this is represented as a $3 \times 3$ homogeneous matrix:

$$T = \begin{pmatrix} \cos\theta & -\sin\theta & x \\ \sin\theta & \cos\theta & y \\ 0 & 0 & 1 \end{pmatrix}$$

Composing two transforms (chaining coordinate frame changes):

$$T_{A}^{C} = T_{B}^{C} \cdot T_{A}^{B}$$

Inverting a transform (going from $B$ back to $A$):

$$T_{B}^{A} = \begin{pmatrix} \cos\theta & \sin\theta & -(x\cos\theta + y\sin\theta) \\ -\sin\theta & \cos\theta & x\sin\theta - y\cos\theta \\ 0 & 0 & 1 \end{pmatrix}$$

---

## 3. Velocity Representation (Twist)

Robot velocity is expressed as a **twist** in the robot's body frame:

$$\boldsymbol{\nu} = \begin{pmatrix} v_x \\ v_y \\ \omega \end{pmatrix}$$

For a differential-drive robot, $v_y = 0$ (no lateral motion), so the twist reduces to:

$$\boldsymbol{\nu} = \begin{pmatrix} v \\ \omega \end{pmatrix}$$

where $v$ is forward speed (m/s) and $\omega$ is angular velocity (rad/s).

---

## 4. Motion Model (Odometry)

Given an initial pose $\mathbf{x}_t$ and a velocity command $\boldsymbol{\nu}$ applied for
time $\Delta t$, the next pose $\mathbf{x}_{t+1}$ under the simple kinematic model is:

$$\mathbf{x}_{t+1} = \mathbf{x}_t + \begin{pmatrix} v \cos\theta \\ v \sin\theta \\ \omega \end{pmatrix} \Delta t$$

This is the **Euler integration** of the motion equations. It introduces errors when
$\Delta t$ is large; higher-order integration (Runge-Kutta) can be used for accuracy.

---

## 5. Probabilistic Notation

The other modules (state estimation, perception) use probabilistic representations.
Key notation used throughout:

- $p(\mathbf{x})$ — probability density of robot state $\mathbf{x}$
- $p(\mathbf{x} \mid \mathbf{z})$ — posterior: probability of state given observation $\mathbf{z}$
- $\mathcal{N}(\boldsymbol{\mu}, \boldsymbol{\Sigma})$ — Gaussian distribution with mean $\boldsymbol{\mu}$ and covariance $\boldsymbol{\Sigma}$

---

## Further Reading

- Thrun, Burgard, Fox — *Probabilistic Robotics* (the definitive reference for this codebase)
- Lynch, Park — *Modern Robotics* (rigorous treatment of kinematics and transforms)
- Selig — *Geometrical Methods in Robotics* (Lie group formulations for advanced use)
