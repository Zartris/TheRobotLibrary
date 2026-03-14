# LQG Theory

## Linear Quadratic Gaussian Control

LQG combines two independently optimal designs:

1. **LQR state feedback** — minimizes $J = \sum_{k=0}^{\infty} \mathbf{x}_k^T Q \mathbf{x}_k + \mathbf{u}_k^T R \mathbf{u}_k$
2. **Kalman filter** (optimal state observer) — minimizes estimation covariance given process noise $Q_w$ and measurement noise $R_v$

## Separation Principle

The key result: the LQR gain $K$ and Kalman gain $L$ can be designed independently; their combination is globally optimal for the combined control + estimation problem.

- **LQR gain**: solve DARE for $(A, B, Q, R)$ → $K$
- **Kalman gain**: solve dual DARE for $(A^T, C^T, Q_w, R_v)$ → $L$
- **LQG law**: $\mathbf{u}_k = -K\hat{\mathbf{x}}_k$, where $\hat{\mathbf{x}}$ is the Kalman estimate

## Partial Observation Model

$$\mathbf{x}_{k+1} = A\mathbf{x}_k + B\mathbf{u}_k + \mathbf{w}_k, \quad \mathbf{w}_k \sim \mathcal{N}(0, Q_w)$$
$$\mathbf{y}_k = C\mathbf{x}_k + \mathbf{v}_k, \quad \mathbf{v}_k \sim \mathcal{N}(0, R_v)$$

The output matrix $C$ can have fewer rows than the state dimension — LQG works with partial observations.

## References

- Anderson & Moore, "Optimal Control: Linear Quadratic Methods," Dover, 2007
- Åström, "Introduction to Stochastic Control Theory," Academic Press, 1970
