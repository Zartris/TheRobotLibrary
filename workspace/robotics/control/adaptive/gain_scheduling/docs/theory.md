# Gain Scheduling — Theory

## Overview

Gain scheduling is an adaptive control strategy where controller gains are adjusted as a
function of measured system state or tracking performance. Unlike fixed-gain PID, a gain-
scheduled controller can maintain consistent closed-loop behavior across different operating
conditions (low vs. high speed, loaded vs. unloaded robot, etc.).

## Error-Based Gain Adaptation

`AdaptivePidController` uses a **performance-based** adaptation law. A sliding window of
recent tracking errors `{e(k), e(k−1), …, e(k−N)}` is maintained. The gain update rule is:

```
K_p(k+1) = K_p(k) + η_p · e(k) · ė(k)
K_i(k+1) = K_i(k) + η_i · e(k) · Σe
K_d(k+1) = K_d(k) + η_d · ė(k)²
```

where `η_p`, `η_i`, `η_d` are small positive learning rates, and gains are clamped to
user-defined `[min, max]` bounds to prevent instability.

This is a simplified **Model Reference Adaptive Control (MRAC)**-inspired approach: the
reference model specifies desired tracking bandwidth, and gains are nudged to minimize
the observed error.

## Stability Considerations

- Gains are **always clamped** to safe bounds — the system cannot become unstable due to
  unbounded gain growth.
- Adaptation rate `η` must be chosen small relative to the closed-loop dynamics. A rule
  of thumb: `η ≪ T_s / τ_cl` where `T_s` is the sample period and `τ_cl` is the desired
  closed-loop time constant.
- The **integral term** is reset when the sign of the error changes, preventing wind-up
  during adaptation transients.

## Difference from Pure Gain Scheduling

Classical gain scheduling precomputes gain tables indexed by a scheduling variable
(e.g., speed). Error-based adaptation does not require a precomputed table — it responds
directly to tracking performance, making it more portable across robot configurations.

## References

- Åström & Wittenmark, *Adaptive Control*, 2nd ed., Dover, 2008.
- Slotine & Li, *Applied Nonlinear Control*, Prentice Hall, 1991, ch. 8.
