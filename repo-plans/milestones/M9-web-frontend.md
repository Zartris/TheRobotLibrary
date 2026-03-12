# M9 — Web Frontend

**Status:** Not Started  
**Dependencies:** M2 (stable API contract). Can start after M2; full feature parity after M8.  
**Scope:** TypeScript/React web app with Canvas 2D rendering. Same functionality as native frontend.

---

## Goal

A browser-based frontend with the same visual elements and controls as the native ImGui app. Deployable as a static site for portfolio/demo purposes. No server-side rendering — connects directly to `simulation_server` via WebSocket + REST.

---

## Deliverables

### App Scaffold

- [ ] React + TypeScript bootstrapped with Vite
- [ ] Project structure: `workspace/frontends/web/src/`
- [ ] `package.json` with scripts: `dev`, `build`, `preview`, `lint`, `test`

### State Connection

- [ ] WebSocket client: connect to sim server, receive JSON state stream
- [ ] REST client: fetch-based command sender (start/stop/reset, module swap, scenario)
- [ ] State store: parsed sim state → React context or lightweight state manager

### Canvas 2D Renderer

- [ ] Grid map (free/occupied/unknown cell colors)
- [ ] Robot (triangle with heading indicator)
- [ ] Lidar rays
- [ ] Planned path (global + local trajectories)
- [ ] EKF covariance ellipse / particle cloud (toggle)
- [ ] SLAM map overlay (when SLAM active)
- [ ] Multi-robot visualization (colored robots, per-robot paths — after M8)
- [ ] Detected obstacles + extracted lines (after M4)

### UI Panels

- [ ] Sim controls: start, stop, reset, step
- [ ] Scenario selector dropdown
- [ ] Module selector dropdowns (controller, planner, estimator)
- [ ] Robot state readout (pose, velocity, estimator type)
- [ ] Visualization layer toggles (lidar, path, particles, etc.)

### Build & Deploy

- [ ] Vite production build → `dist/` static files
- [ ] CI step to build web frontend
- [ ] Deployable to GitHub Pages / Netlify (just static files)
- [ ] **Deployment note:** When hosted on a remote static site, the WebSocket/REST URLs must be configurable (env variable or runtime config). For portfolio demos, the sim server must be running and accessible (e.g. via a tunnel or a cloud VM). Document this in the web frontend README.

---

## Exit Criteria

1. Web frontend connects to `simulation_server` and displays all visual elements
2. All sim controls work (start/stop/reset, module swap, scenario load)
3. Lighthouse performance ≥ 90
4. Static build deploys successfully
5. Feature parity with native frontend (for features implemented so far)

## NOT IN

3D rendering, WebGL, mobile-specific layouts, SSR.
