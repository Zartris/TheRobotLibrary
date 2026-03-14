# LQR Theory

## 1. Problem Formulation

The discrete-time infinite-horizon LQR minimises the quadratic cost functional
$J = \sum_{k=0}^{\infty} \left( x_k^T Q x_k + u_k^T R u_k \right)$
subject to the linear dynamics $x_{k+1} = A x_k + B u_k$, where $Q \succeq 0$ and
$R \succ 0$ are designer-chosen weighting matrices.

## 2. Discrete Algebraic Riccati Equation (DARE)

The optimal cost-to-go matrix $P$ satisfies the DARE:
$P = Q + A^T P A - A^T P B (R + B^T P B)^{-1} B^T P A$

$P$ can be computed iteratively (value iteration) or via eigenvalue decomposition of the
symplectic pencil. Eigen's `GeneralizedSelfAdjointEigenSolver` provides the latter.

## 3. Optimal Feedback Gain

Once $P$ is known, the optimal state-feedback gain is:
$K = (R + B^T P B)^{-1} B^T P A$

The optimal control law is $u_k = -K x_k$.

## 4. Stability

The closed-loop matrix $A - BK$ must have all eigenvalues strictly inside the unit circle
for the regulated system to be asymptotically stable. This is guaranteed when $(A, B)$ is
stabilisable and $(A, Q^{1/2})$ is detectable.

## 5. Q/R Weighting Intuition

- Increasing $Q$ (relative to $R$) penalises state deviation more → faster convergence,
  larger control effort.
- Increasing $R$ penalises control effort → slower, more energy-efficient response.

## 6. Linearization at Operating Point

For nonlinear systems, $A$ and $B$ are obtained by Jacobian linearization around a nominal
trajectory or equilibrium. The diff-drive linearization helper in this module computes
$A$, $B$ for a unicycle model at a given operating speed and heading.
