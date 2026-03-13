# TheRobotLibrary

A modular C++ robotics library and simulation platform, built from a mobile robot perspective.

The library covers the core domains of robotics engineering — perception, state estimation,
motion planning, and control — with each module designed to be independently reusable across
projects. Alongside the library, the project includes a 2D simulation engine with a clearly
separated backend and frontends: a native C++ desktop application (ImGui) and a web-based
frontend, both communicating with the simulation server over WebSockets and REST.

Each module ships with learning documentation explaining the underlying theory, making this
both a practical engineering library and an educational resource.

---

## Goals

- **Modular** — Every module under `workspace/robotics/` is self-contained with its own build
  config, headers, source, and tests. Copy any subfolder into another project and use it directly
  with no changes to the rest of this repo.

- **Educational** — Each module has a `docs/theory.md` that introduces the concepts, algorithms,
  and mathematics behind it. Readable without needing to understand the rest of the codebase.

- **Simulation-first** — All algorithms are developed and validated against a clean 2D simulation
  before being deployed to real hardware.

- **Multi-frontend** — The simulation backend is a standalone C++ server. Both the native ImGui
  frontend and the web frontend connect to the same server instance — one backend, multiple
  independent clients.

---

## Repository Layout

```
TheRobotLibrary/
├── README.md                  ← You are here
├── repo-plans/                ← High-level roadmap and milestones for this repository
├── docker/                    ← Dev environment (Ubuntu 24.04 + AI coding tools)
└── workspace/                 ← All code and documentation
    ├── architecture.md        ← Full system architecture — read before writing code
    ├── robotics/              ← Reusable, self-contained C++ robotics modules
    │   ├── common/            ← Shared types, math primitives, and core interfaces
    │   ├── control/           ← Controllers: PID, pure pursuit, MPC
    │   ├── perception/        ← Sensor models, feature extraction, occupancy grids
    │   ├── state_estimation/  ← EKF, particle filter, SLAM, localization
    │   └── motion_planning/   ← Path planners: A*, RRT, DWA, trajectory optimization
    ├── simulation/            ← 2D simulation backend — C++ server with WS + REST API
    └── frontends/
        ├── native/            ← Desktop GUI built with ImGui (C++)
        └── web/               ← Browser frontend (TypeScript + React)
```

---

## Quick Start

### Development environment

```bash
cd docker
docker compose -f docker-compose.dev.yml up -d --build
docker exec -it therobotlibrary-sandbox bash
```

### Using a robotics module in your own project

Each module under `workspace/robotics/` is standalone. Copy the module folder into your project
and add it to your CMake build:

```cmake
add_subdirectory(control)
target_link_libraries(your_target PRIVATE control)
```

---

## Navigation

| Goal                                       | Location                                                     |
|--------------------------------------------|--------------------------------------------------------------|
| Understand the full system architecture    | [`workspace/architecture.md`](workspace/architecture.md)     |
| Learn the theory behind a robotics topic   | `workspace/robotics/<module>/docs/theory.md`                 |
| Reuse a robotics module in another project | `workspace/robotics/<module>/`                               |
| Understand or extend the simulation server | [`workspace/simulation/`](workspace/simulation/)             |
| Work on the native desktop GUI (ImGui)     | [`workspace/frontends/native/`](workspace/frontends/native/) |
| Work on the web frontend                   | [`workspace/frontends/web/`](workspace/frontends/web/)       |
| View the project roadmap                   | [`repo-plans/`](repo-plans/)                                 |

---

## Design Philosophy

The robotics modules are written in modern C++ (C++20 minimum) with no external framework
dependencies beyond what each module explicitly declares. They are intentionally small in scope
so that the code and the accompanying theory docs can be read together.

The simulation is not a research-grade physics engine — it is a clean, deterministic 2D
environment whose purpose is to make algorithms observable, debuggable, and comparable.

The frontend separation is intentional and permanent. If you add a new visualization capability,
it goes into the API contract, not into a direct library link. This constraint keeps both
frontends independently deployable.
