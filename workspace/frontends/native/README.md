# Native Frontend (ImGui)

A C++ desktop application that visualizes the 2D simulation in real time using **ImGui**.

This frontend is intended for local development, debugging visualization, and running
the simulation locally. It connects to a running simulation backend over WebSocket and REST.

---

## Technology

- **GUI framework:** [Dear ImGui](https://github.com/ocornut/imgui) with SDL2 + OpenGL3 backend
- **Rendering:** `ImDrawList` — ImGui's immediate-mode 2D canvas for drawing the robot,
  map, paths, and sensor beams
- **HTTP client:** [cpp-httplib](https://github.com/yhirose/cpp-httplib) for REST commands
- **WebSocket client:** [IXWebSocket](https://github.com/machinezone/IXWebSocket) or
  [websocketpp](https://github.com/zaphoyd/websocketpp) for receiving state updates

---

## Prerequisites

Start the simulation backend before launching this frontend:

```bash
# Terminal 1 — simulation backend
cd workspace/simulation && ./build/simulation_server --port 8080
```

---

## Building and Running

```bash
cd workspace/frontends/native
cmake -B build && cmake --build build
./build/robot_viz --host localhost --port 8080
```

---

## Directory Layout

```
native/
├── CMakeLists.txt
├── README.md           ← This file
└── src/
    ├── main.cpp        # Entry point; SDL2 init, ImGui setup, main loop
    ├── renderer.cpp    # ImDrawList-based 2D world renderer
    ├── ws_client.cpp   # WebSocket client — receives state, parses JSON
    ├── rest_client.cpp # REST client — sends commands
    └── ui_panels.cpp   # ImGui control panels (start/stop, cmd_vel sliders, etc.)
```
