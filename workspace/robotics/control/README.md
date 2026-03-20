# control

Robot controller implementations for mobile robots.

Each sub-folder is a **standalone C++ library** — copy only the controller you need into
your project. All sub-modules depend only on `common`.

---

## Sub-modules

| Sub-module        | Description                                              |
|-------------------|----------------------------------------------------------|
| [`pid/`](pid/)    | PID: proportional-integral-derivative controller         |
| [`pure_pursuit/`](pure_pursuit/) | Geometric path-tracking for differential-drive robots |
| [`mpc/`](mpc/)    | Model Predictive Control: receding-horizon optimizer     |

---

## Domain overview

See [`docs/theory.md`](docs/theory.md) for the big-picture introduction to feedback control
before diving into a specific sub-module's theory document.

---

## Dependency rule

All sub-modules depend only on `common`. They have no dependency on each other, on
`simulation`, or on the simulation app.
