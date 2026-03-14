# MPPI Theory

## Algorithm Overview

Model Predictive Path Integral (MPPI) is a sampling-based stochastic optimal control framework. It optimizes a control sequence by estimating the expected cost of perturbed trajectories using importance sampling.

## Cost Function

Given a state trajectory $\mathbf{x}_{0:H}$ and control sequence $\mathbf{u}_{0:H-1}$, the total cost is:

$$J = \phi(\mathbf{x}_H) + \sum_{t=0}^{H-1} c(\mathbf{x}_t, \mathbf{u}_t)$$

where $\phi$ is a terminal cost and $c$ is a running cost.

## Importance-Weighted Update

MPPI samples $N$ perturbations $\{\epsilon_k\}$ from $\mathcal{N}(0, \Sigma)$, propagates each through the motion model, and computes the importance weights:

$$w_k = \exp\!\left(-\frac{J_k}{\lambda}\right)$$

The optimal control update is:

$$\mathbf{u}^* = \sum_{k=1}^{N} \tilde{w}_k \epsilon_k, \quad \tilde{w}_k = \frac{w_k}{\sum_j w_j}$$

## Temperature Parameter λ

- **High λ**: weights approach uniform → broad exploration
- **Low λ**: weights concentrate on best rollout → exploitation

## References

- Williams et al., "Model Predictive Path Integral Control using Covariance Variable Importance Sampling," 2018
- Williams et al., "Information Theoretic MPC for Model-Based Reinforcement Learning," ICRA 2017
