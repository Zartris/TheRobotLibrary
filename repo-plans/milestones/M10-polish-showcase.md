# M10 — Polish & Showcase

**Status:** Not Started  
**Dependencies:** M8, M9 (can begin partially after M8; full completion requires all milestones done)  
**Scope:** Turn working codebase into a presentable, distributable project.

---

## Goal

The project is portfolio-ready: live documentation site, demo GIFs, standalone examples, CMake install support, clean README with badges. A visitor can understand, build, and use the library within minutes.

---

## Deliverables

### Documentation Site

- [ ] mkdocs or Doxygen from READMEs + theory.md + API headers
- [ ] Auto-build in CI → deploy to GitHub Pages
- [ ] Landing page: project overview, architecture diagram, quick start
- [ ] Per-module pages: theory + API reference + usage examples
- [ ] Search functionality

### Example Programs

- [ ] ≥ 1 standalone example per domain (5+ total):
  - Control: PID + pure_pursuit tracking comparison
  - Perception: lidar processing + grid building demo
  - State Estimation: EKF vs particle filter visualization
  - Planning: A* vs RRT* path comparison
  - Multi-robot: 5-robot ORCA scenario
- [ ] Each example independently buildable with own CMakeLists.txt
- [ ] Examples documented in docs site

### Capstone Demo Scenarios

- [ ] "Full autonomy": robot explores unknown room with SLAM → plans to goal → avoids dynamic obstacles
- [ ] "Controller comparison": same path, three controllers, side-by-side
- [ ] "Multi-robot coordination": 5 robots with algorithm switching
- [ ] Screen recordings → demo GIFs for README

### CMake Packaging

- [ ] CMake install targets for all modules
- [ ] `find_package(TheRobotLibrary COMPONENTS pid ekf astar)` works in a clean consumer project
- [ ] CMake config files (TheRobotLibraryConfig.cmake)
- [ ] Install instructions in docs

### README Polish

- [ ] CI badge, coverage badge, license badge
- [ ] Demo GIFs at top of README
- [ ] Clear "Getting Started" section (3-step: clone → build → run demo)
- [ ] Architecture overview with diagram
- [ ] "Available Modules" table with status per module

### Web Deployment

- [ ] Web frontend deployed to GitHub Pages or Netlify
- [ ] README links to live demo
- [ ] Build pipeline automated in CI

---

## Exit Criteria

1. Documentation site live and auto-deploying
2. `cmake --install` + `find_package` work in a clean consumer project
3. Demo GIFs in README show end-to-end navigation
4. All CI checks green, coverage ≥ 80%
5. Web demo accessible via URL
6. A new developer can go from zero to running demo in < 10 minutes
7. All modules pass Phase 4.5 — Observability gate (confirmed during M1–M10 delivery; verified in CI)

## NOT IN

New algorithms, new features. Bug fixes only.
