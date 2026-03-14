# Noise Models Theory

## Purpose

`noise_models` provides seeded, reproducible noise primitives for use in tests across all modules. Reproducibility is achieved by accepting an explicit integer seed for the `std::mt19937` Mersenne Twister RNG.

## Gaussian Noise

Additive white Gaussian noise (AWGN) with zero mean and standard deviation $\sigma$:

$$n \sim \mathcal{N}(0, \sigma^2)$$

Sample via `std::normal_distribution<T>`. The seeded RNG ensures identical sequences for the same seed, enabling deterministic test assertions.

## Uniform Noise

Uniform distribution over $[lo, hi]$:

$$n \sim \mathcal{U}(lo, hi)$$

Bucket histogram tests verify coverage and uniformity (all buckets within 20% of expected count).

## Outlier Injection

`OutlierInjector<T>` wraps any base noise model and probabilistically replaces samples with large-magnitude outliers:

$$n = \begin{cases} n_{base} & \text{with probability } 1 - p \\ n_{outlier} \sim \mathcal{U}(lo_{out}, hi_{out}) & \text{with probability } p \end{cases}$$

Bernoulli trial uses the same seeded Mersenne Twister, preserving reproducibility.

## Batch & In-Place API

- `sampleN(n)` → `std::vector<T>`: draw $n$ independent samples
- `addTo(Eigen::VectorXd&)`: add independent noise to each element in-place (dimension-wise, vector size unchanged)
