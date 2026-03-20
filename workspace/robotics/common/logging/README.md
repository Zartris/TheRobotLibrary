# common/logging

Structured logging interface and default spdlog implementation for all robotlib modules.

## Purpose

Provides an `ILogger` interface and `SpdlogLogger` concrete implementation so every module can emit structured, leveled log messages without coupling to a specific logging backend. AI agents and automated tools can consume the log output without needing the simulation app running.

## Interfaces

| Symbol | Description |
|--------|-------------|
| `ILogger` | Abstract base — `debug`, `info`, `warn`, `error` with `fmt`-style format strings |
| `SpdlogLogger` | Default impl backed by [spdlog](https://github.com/gabime/spdlog) |
| `getLogger(name)` | Factory function returning a shared `ILogger` for the given module name |

## Log format

```
[LEVEL][MODULE] message key=value key2=value2
```

Example:
```
[INFO][pid] step dt=0.010 error=0.34 output=1.20
[WARN][dwa] velocity_samples=0 — no feasible window
[ERROR][ekf] covariance not positive-definite; resetting
```

## Usage (Phase 4.5 of every module)

```cpp
#include <common/logging/logger.hpp>

auto log = common::getLogger("my_module");
log->info("initialised gain={}", gain);
log->debug("step dt={:.4f} error={:.4f}", dt, error);
```

## Dependencies

- `common` (owns `ILogger` interface header)
- [spdlog](https://github.com/gabime/spdlog) v1.12+ (compile-time dep; header-only or compiled)

## Directory layout

```
logging/
├── CMakeLists.txt
├── README.md
├── docs/
│   └── theory.md          # Design rationale and structured-logging conventions
├── include/
│   └── common/
│       └── logging/
│           ├── ilogger.hpp       # ILogger abstract interface
│           └── logger.hpp        # getLogger() factory + type aliases
└── src/
    └── spdlog_logger.cpp         # SpdlogLogger implementation
```
