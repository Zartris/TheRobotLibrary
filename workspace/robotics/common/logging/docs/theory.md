# Observability Design for TheRobotLibrary

## Motivation

AI coding agents and automated CI pipelines have no access to the simulation app. Every correctness indicator that a human developer would read from a visualiser must also be emitted as a machine-readable log line or metric. This document defines the logging conventions that make every robotlib module observable without a display.

## ILogger Contract

```
ILogger::debug(fmt, args...)
ILogger::info(fmt, args...)
ILogger::warn(fmt, args...)
ILogger::error(fmt, args...)
```

All methods are pure-virtual. Implementations must be thread-safe. Format strings follow libfmt / C++20 `std::format` conventions.

## Structured Log Format

```
[LEVEL][MODULE] message key=value key2=value2
```

Rules:
- `LEVEL` — one of `DEBUG`, `INFO`, `WARN`, `ERROR` (uppercase, 5 chars max)
- `MODULE` — the logger name passed to `getLogger()`, e.g. `pid`, `dwa`, `ekf`
- Free-text message, then zero or more `key=value` pairs separated by spaces
- Numeric values: use fixed precision (`.4f` for floats, `.6f` for angles in radians)
- No newlines inside a single log call

## Per-module Logging Requirements (Phase 4.5)

Every module **must** emit at minimum:

| Event | Level | Required keys |
|-------|-------|---------------|
| Module construction / parameter load | `INFO` | all tunable parameters |
| Each control/estimation step | `DEBUG` | `dt`, primary input, primary output |
| Constraint violation or fallback | `WARN` | constraint name, actual vs limit |
| Unrecoverable error | `ERROR` | error description, recovery action |

## SpdlogLogger Implementation Notes

- Uses spdlog's `stdout_color_mt` sink by default.
- Log level is controlled by the `ROBOTLIB_LOG_LEVEL` environment variable (default: `INFO`).
- `getLogger(name)` returns a shared logger; calling it twice with the same name returns the same instance.
- In tests, swap `SpdlogLogger` for a `NullLogger` or `RecordingLogger` to assert log content without polluting stdout.

## Why Not ROS2 / std::cout

- `std::cout` is not thread-safe and has no level filtering.
- ROS2 would couple the library to a middleware it explicitly avoids.
- spdlog is header-only optional, ships fast, and is widely used in C++ robotics tooling.

## References

- [spdlog GitHub](https://github.com/gabime/spdlog)
- [libfmt](https://fmt.dev/)
- `repo-plans/module-task-template.md` — Phase 4.5 checklist
