# Simulation Backend

The simulation backend is a standalone C++ server application that runs the 2D simulated
world and exposes it to frontends over a WebSocket + REST API.

**Important:** The simulation is the single source of truth for world state. Both the native
and web frontends are read-only subscribers (via WebSocket) that also send commands (via REST).
No frontend links against this library — they communicate only through the network API.

---

## Responsibilities

- Maintaining the simulated world: robot pose, map, obstacles, sensor readings
- Running the simulation loop at a fixed internal tick rate (default 50 Hz)
- Streaming world state to connected clients via WebSocket at 30 Hz
- Accepting control commands and configuration over REST HTTP
- Loading and managing simulation scenarios

---

## API Summary

See [`../../architecture.md`](../../architecture.md) for the full API specification.

| Protocol   | Endpoint              | Purpose                              |
|------------|-----------------------|--------------------------------------|
| WebSocket  | `ws://host:8080/state`| Continuous state stream to clients   |
| REST POST  | `/api/sim/start`      | Start / resume simulation            |
| REST POST  | `/api/sim/stop`       | Pause simulation                     |
| REST POST  | `/api/sim/reset`      | Reset to initial state               |
| REST PUT   | `/api/sim/speed`      | Set speed multiplier                 |
| REST POST  | `/api/robot/cmd_vel`  | Send velocity command `{v, omega}`   |
| REST GET   | `/api/scenario/list`  | List available scenarios             |
| REST POST  | `/api/scenario/load`  | Load a named scenario                |

---

## Technology

- **Server library:** [Crow](https://crowcpp.org/) — header-only C++ HTTP + WebSocket server
- **Build:** CMake, produces an executable `simulation_server`
- **Dependencies:** Crow, nlohmann/json (JSON serialization), and the robotics modules from
  `workspace/robotics/` (linked directly, not via network)

---

## Directory Layout

```
simulation/
├── CMakeLists.txt
├── README.md               ← This file
├── include/simulation/     # Internal headers (not for external consumers)
├── src/
│   ├── main.cpp            # Entry point; starts Crow server
│   ├── world.cpp           # World state model
│   ├── sim_loop.cpp        # Fixed-timestep simulation loop
│   ├── api_server.cpp      # WebSocket broadcaster + REST route handlers
│   └── scenario_loader.cpp # Loads scenario files from disk
├── tests/
└── docs/
    └── design.md           # Internal design decisions and extension guide
```

---

## Running the Simulation Server

```bash
cd workspace/simulation
cmake -B build && cmake --build build
./build/simulation_server --port 8080 --scenario default
```

Once running, open the native frontend or web frontend and they will connect automatically.
