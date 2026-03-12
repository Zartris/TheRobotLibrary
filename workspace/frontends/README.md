# Frontends

This directory contains all user interface code for TheRobotLibrary simulation.

---

## Key Rule

**Frontends must not link against any C++ library from this repository.**
They communicate exclusively with the simulation backend via:
- **WebSocket** `ws://host:8080/state` — receives world state updates (read-only)
- **REST HTTP** `http://host:8080/api/...` — sends commands (start, stop, cmd_vel, etc.)

This constraint keeps each frontend independently buildable and deployable, and ensures
that changing a frontend never requires touching simulation or robotics module code.

---

## Frontends

### [`native/`](native/)

A C++ desktop application using **ImGui** for rendering the 2D simulation world.
Designed for local development, debugging, and demonstration from the command line.

- Connects to the simulation WebSocket for live state updates
- Sends commands via REST using cpp-httplib
- Renders using `ImDrawList` (2D canvas API built into ImGui)

### [`web/`](web/)

A browser-based frontend using **TypeScript + React**.
Designed to be embedded on a personal homepage or portfolio as a live interactive demo.

- Connects to the simulation WebSocket using the browser's native `WebSocket` API
- Sends commands via `fetch()` REST calls
- Renders the 2D world on an HTML5 Canvas element

---

## Running a Frontend

1. Start the simulation backend first — see [`../simulation/README.md`](../simulation/README.md)
2. Then launch either frontend — see the README inside each subfolder
