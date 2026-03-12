# Web Frontend (TypeScript / React)

A browser-based frontend for the 2D simulation, built with TypeScript and React.

This frontend is designed to be embedded on a personal homepage or portfolio site as a
live interactive demo. It connects to a running simulation backend over WebSocket and REST.

---

## Technology

- **Language:** TypeScript
- **Framework:** React
- **Rendering:** HTML5 Canvas (2D) — upgradeable to WebGL or Three.js for 3D later
- **State connection:** Browser-native `WebSocket` API for the live state stream
- **Commands:** `fetch()` REST calls for start/stop/cmd_vel

---

## Prerequisites

Start the simulation backend before launching this frontend:

```bash
# Terminal 1 — simulation backend
cd workspace/simulation && ./build/simulation_server --port 8080
```

If the frontend is served from a different origin than the simulation backend, ensure
the backend sends appropriate CORS headers.

---

## Development

```bash
cd workspace/frontends/web
npm install
npm run dev        # Development server with hot reload (default: http://localhost:5173)
```

## Production Build

```bash
npm run build      # Outputs static files to dist/
```

The `dist/` folder can be served by any static hosting provider or embedded into a
personal website as a sub-path.

---

## Directory Layout

```
web/
├── package.json
├── tsconfig.json
├── vite.config.ts      # (or equivalent bundler config)
├── README.md           ← This file
└── src/
    ├── main.tsx        # React entry point
    ├── App.tsx         # Root component; WebSocket connection lifecycle
    ├── SimCanvas.tsx   # HTML5 Canvas component — renders robot, map, sensors
    ├── ControlPanel.tsx# React UI for start/stop/speed/cmd_vel controls
    ├── api.ts          # REST command helpers (thin wrappers over fetch)
    └── types.ts        # TypeScript types matching the simulation state JSON schema
```
