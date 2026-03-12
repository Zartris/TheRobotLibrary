# Theory: Particle Filter (Monte Carlo Localization)

---

## 1. Why Particles?

The EKF constrains the belief to a Gaussian — a single peak. When the robot does not know
its initial position (global localization), the true belief is multi-modal: the robot could
be at any of several plausible locations. Particles can represent this directly.

A particle cloud $\{\mathbf{x}^{[m]}, w^{[m]}\}_{m=1}^M$ approximates any distribution:
high particle density ≈ high probability.

---

## 2. The MCL Algorithm

Repeat at each timestep:

```
Input: particles {x^[m], w^[m]}, command u_t, measurement z_t

// Predict: sample new particle positions via motion model
For m = 1..M:
    x̃^[m] ~ p(x_t | u_t, x^[m]_{t-1})

// Update: weight each particle by measurement likelihood
For m = 1..M:
    w^[m] = p(z_t | x̃^[m], map)

// Normalize weights
w^[m] ← w^[m] / Σ_j w^[j]

// Resample: draw M new particles proportional to weights
{x^[m]} ← resample({x̃^[m], w^[m]})

Output: particles {x^[m], 1/M}
```

---

## 3. Resampling Strategies

Resampling replaces low-weight particles with copies of high-weight particles.

**Multinomial resampling** — draw $M$ samples independently from the weight distribution.
Simple but high variance: good particles may be duplicated many times while others are lost.

**Systematic resampling** (preferred) — draw one uniform $u \sim U[0, 1/M]$, then use
$u, u+1/M, u+2/M, \ldots, u+(M-1)/M$ as the $M$ sample positions on the cumulative weight
curve. Lower variance, $O(M)$ time.

**Low-variance resampling** = systematic resampling. Used in all production MCL implementations.

---

## 4. Particle Deprivation Problem

If $M$ is too small, resampling can eliminate all particles near the true pose — the
filter **loses** the robot (particle deprivation). Two fixes:

**Injection of random particles** — after resampling, replace a small fraction of particles
with uniformly sampled poses. This adds recovery capability.

**AMCL (Adaptive MCL)** — KLD-sampling: estimate the error between the particle distribution
and the true posterior using KL divergence, and set $M$ just large enough to keep the error
below a threshold $\epsilon$. More particles when uncertain, fewer when confident.

$$M_\epsilon = \frac{1}{2\epsilon}\chi^2_{d-1, 1-\delta}$$

where $d$ is the number of non-empty histogram bins covering particle positions, and
$\chi^2$ is the chi-squared quantile.

---

## 5. Global vs. Tracking Mode

| Mode | Initial distribution | Particles needed |
|---|---|---|
| **Global localization** | Uniform over free space | 5000–50000 |
| **Pose tracking** | Gaussian around known pose | 200–1000 |
| **Kidnap recovery** | Mix of both | 2000–10000 |

Start global, reduce to tracking once converged. AMCL handles this automatically.

---

## Further Reading

- Thrun, Burgard, Fox — *Probabilistic Robotics*, Chapter 4 (particle filters) and Chapter 8 (MCL)
- Fox — *Adapting the Sample Size in Particle Filters Through KLD-Sampling*, IJRR 2003
