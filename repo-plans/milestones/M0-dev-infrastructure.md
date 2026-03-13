# M0 — Dev Infrastructure

**Status:** Not Started  
**Dependencies:** None  
**Scope:** Tooling, configuration, CI. **Zero library code.**

---

## Goal

Every line of code written after this milestone benefits from CI, linting, AI agent context, and a reproducible dev container. This is the cheapest time to establish conventions — no code exists yet.

---

## Deliverables

### AI Agent Instructions

- [ ] `.claude/CLAUDE.md` — Points agents at `architecture.md`, naming conventions, module patterns, dependency rules, build commands
- [ ] `.github/copilot-instructions.md` — Same guidance for GitHub Copilot
- [ ] `.opencode/instructions.md` — Same guidance for OpenCode

### Dev Container

- [ ] `.devcontainer/devcontainer.json` — References existing `Dockerfile.dev` + `docker-compose.dev.yml`, installs VS Code extensions: clangd, cmake-tools, Catch2 test explorer

### CI/CD

- [ ] `.github/workflows/ci.yml` — GitHub Actions: CMake configure → build → ctest on Ubuntu 24.04 (GCC 13), triggered on push and PR

### Code Quality

- [ ] `.clang-format` — Google-derived style, 100-column, C++20 features (this is located in workspace currently, but might need to be moved to the correct place.)
- [ ] `.clang-tidy (clangd)` — Core checks, modernize-*, readability-*, performance-* (this is located in workspace currently, but might need to be moved to the correct place.)
- [ ] `.pre-commit-config.yaml` — clang-format check, clang-tidy, trailing whitespace, end-of-file fixer

### Planning Artifacts

- [ ] `repo-plans/module-task-template.md` — Standard per-module workflow
- [ ] `repo-plans/milestones/M0-M12` — Skeleton milestone docs (this set of files)

### Fleet Management Scaffold

- [ ] `workspace/robotics/fleet_management/` directory tree created (CMakeLists.txt stubs + READMEs + theory.md for all 4 sub-modules)
- [ ] Added to `workspace/robotics/CMakeLists.txt` via `add_subdirectory(fleet_management)`
- [ ] All fleet_management sub-modules build as INTERFACE targets (no source files yet)

---

## Exit Criteria

1. `.devcontainer/devcontainer.json` opens in VS Code Remote Containers and `cmake -B build -S workspace` succeeds (on the empty scaffold)
2. CI workflow runs on push, succeeds with zero-source scaffold (configure only, no compile errors)
3. `pre-commit run --all-files` passes
4. AI agent instructions exist and reference `architecture.md` + naming conventions
5. All milestone doc skeletons exist in `repo-plans/milestones/`
6. Fleet management scaffold builds cleanly as INTERFACE targets
7. `common/logging/` scaffold exists (`ILogger`, `SpdlogLogger`, `getLogger()` stubs, `docs/theory.md`); all modules build with logging sub-target present

---

## Notes

- Keep AI instructions DRY: all three files (CLAUDE.md, copilot-instructions.md, opencode) can share content or point at a single source of truth (`architecture.md`)
- The `.clang-format` should match the conventions already documented: `PascalCase` types, `camelCase` methods, `m_` prefix — clang-format doesn't enforce naming, but it enforces layout
- CI should use `cmake --build build` even with no sources — the scaffold creates INTERFACE targets that should configure cleanly
