# Feedback Linearization Theory

## Input-Output Linearization

Feedback linearization transforms a nonlinear system into a linear one via a diffeomorphism (smooth invertible state transformation) and a nonlinear feedback control law.

## Differential-Drive Chained Form

For a differential-drive robot with state $(x, y, \theta)$, introduce a virtual output point at lookahead distance $l$:

$$\xi = \begin{pmatrix} x + l\cos\theta \\ y + l\sin\theta \end{pmatrix}$$

The time derivative is:

$$\dot{\xi} = \begin{pmatrix} \cos\theta & -l\sin\theta \\ \sin\theta & \phantom{-}l\cos\theta \end{pmatrix} \begin{pmatrix} v \\ \omega \end{pmatrix}$$

Setting $\dot{\xi} = \nu$ (desired virtual output velocity), the inverse kinematics gives:

$$\begin{pmatrix} v \\ \omega \end{pmatrix} = \frac{1}{l} \begin{pmatrix} l\cos\theta & l\sin\theta \\ -\sin\theta & \cos\theta \end{pmatrix} \nu$$

## Lie Derivatives

The Lie derivative $L_f h(\mathbf{x})$ measures the rate of change of $h$ along the flow of $f$:

$$L_f h(\mathbf{x}) = \nabla h(\mathbf{x}) \cdot f(\mathbf{x})$$

For verification: numerical $L_f h$ via finite difference should match the analytic expression to within $10^{-5}$ relative error.

## Singularity

When $l = 0$, the transformation matrix is singular (the virtual point coincides with the robot axle). This case returns a `std::expected` error; it must never cause division by zero in production code.

## References

- Isidori, "Nonlinear Control Systems," 3rd ed., Springer, 1995
- De Luca & Oriolo, "Feedback Linearization of Differential Drive Vehicles," ICRA 1998
