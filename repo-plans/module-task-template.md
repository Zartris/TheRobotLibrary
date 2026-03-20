# Module Task Template

> Copy this template into `repo-plans/modules/<module-name>.md` when starting work on a module.
> Check off tasks as you go. When all phases are complete, move the file to `repo-plans/modules/done/`.

---

## Module: `<name>`

**Milestone:** M`<N>` — `<milestone name>`  
**Status:** Not Started | In Progress | Done  
**Depends on:** common, `<other modules if any>`

---

### Phase 1 — Interface Design

- [ ] Define public headers in `include/<module>/`
- [ ] Config struct with sensible defaults
- [ ] Implement the relevant `I<Interface>` from `common/interfaces/` (if applicable)
- [ ] Document inputs, outputs, and error conditions in header comments

### Phase 2 — Failing Tests (Red)

- [ ] Create `tests/CMakeLists.txt` with Catch2 target
- [ ] Write tests for core functionality with known-answer inputs
- [ ] Write edge-case tests (empty input, degenerate configs, numerical limits)
- [ ] Verify tests fail (module not yet implemented)

```bash
cmake --build build --target <module>_tests
cd build && ctest -R <module> --output-on-failure
```

### Phase 3 — Implementation (Green)

- [ ] Implement core algorithm in `src/`
- [ ] Only depend on `common` (+ Eigen transitively if needed)
- [ ] Follow conventions: `PascalCase` types, `camelCase` methods, `m_` member prefix
- [ ] Use `std::expected<T,E>` for recoverable errors at public API boundaries

### Phase 4 — Passing Tests

- [ ] All unit tests pass
- [ ] Add any additional tests discovered during implementation
- [ ] CI green

```bash
cmake --build build --target <module>_tests
cd build && ctest -R <module> --output-on-failure
```

### Phase 4.5 — Observability

> **This phase gates module completion.** Both human developers and AI agents must be able
> to verify correct behavior through logs and metrics — not just visual inspection.

- [ ] `ILogger` injected into module constructor via `common::getLogger("<module>")` (mockable in tests)
- [ ] All state transitions logged at `DEBUG` level (init, reset, mode changes, error paths)
- [ ] Hot-loop performance metrics logged at `TRACE` level (cycle time per iteration in µs, iteration count)
- [ ] At least one test asserts expected log lines appear via stdout capture
- [ ] Zero `ERROR`-level log entries during all nominal test runs

```bash
# Confirm logging output during tests:
cmake --build build --target <module>_tests
cd build && ctest -R <module> --output-on-failure 2>&1 | grep "\[DEBUG\]\|\[TRACE\]"
```

### Phase 5 — Simulation Integration

> _For M1 sub-phases B–I, sim integration happens during M1-K when the pipeline is created. For post-M1 modules, register with the existing `RobotPipeline`._

- [ ] Wire module into simulation loop (register with `RobotPipeline`)
- [ ] Add module output to bridge state adapter
- [ ] Add ImGui panel entries for module-specific config (if needed)
- [ ] Integration test: sim runs with this module active, state is valid

### Phase 6 — Visualization

- [ ] Add visualization to ImGui telemetry panel (if applicable)
- [ ] Toggle visibility from UI panel

### Phase 7 — Docs Polish

- [ ] Update module `README.md` with usage examples
- [ ] Add implementation notes / lessons learned to `docs/theory.md`
- [ ] Move this task file to `repo-plans/modules/done/<module-name>.md`
