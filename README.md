# Impulse Engine

A real-time 2D physics simulation engine written in C++ with OpenGL rendering.

Impulse Engine models rigid-body dynamics and gravity for many simultaneous
bodies, detects and resolves collisions using an impulse-based (sequential
impulse) solver, and ships with an interactive sandbox for building and
stress-testing scenarios.

Impulse Engine provides:

- rigid-body dynamics for circles, convex polygons, and planes under gravity
  and applied forces
- broad-phase + narrow-phase collision detection (spatial grid + SAT)
- impulse-based collision response with restitution and Coulomb friction
- a fixed-timestep simulation loop decoupled from render rate, for
  deterministic, frame-rate-independent physics
- an interactive ImGui sandbox for spawning bodies, tuning parameters, and
  loading pre-built stress-test scenarios (12 built-in)
- gravity wells (attractors) for orbital and radial-gravity effects
- built-in diagnostics: frame-time profiler with per-phase breakdown,
  instability detector (NaN/Inf/velocity/position thresholds), debug overlay
  (AABBs, velocity vectors)

This is a from-scratch educational/portfolio implementation — no external
physics library (Box2D, Chipmunk, etc.) is used for the simulation core.

- Build instructions: see [Build, Test, and Development](#build-test-and-development)
- Design docs: see root-level docs (`01_SCOPE_AND_REQUIREMENTS.md` through
  `06_ROADMAP.md`)
- Agent/contributor guidance: see [`AGENTS.md`](./AGENTS.md)

## Repository Structure

```
ImpulseEngine/
├── physics/             # Physics core: Vec2, RigidBody, World
├── collision/           # CollisionDetect: SAT narrow-phase + impulse solver
├── diagnostics/         # Frame profiler + instability detector
├── glad/                # GLAD OpenGL loader
├── imgui/               # Dear ImGui (v1.91.8, cloned)
├── tests/               # Unit tests (39 tests)
├── scenario_library.h   # 12 pre-built scenarios
├── main.cpp             # Interactive ImGui sandbox application
├── 01_SCOPE_AND_REQUIREMENTS.md
├── 02_ARCHITECTURE.md
├── 03_USER_FLOW.md
├── 04_TESTING_AND_PROFILING.md
├── 05_AGENT_WORKFLOW.md
├── 06_ROADMAP.md
├── AGENTS.md
├── ImpulseEngine.sln
├── ImpulseEngine.vcxproj
├── ImpulseEngineTests.vcxproj
└── README.md
```

## Current Milestone: M8 (Spatial-Grid Broad-Phase)

Collision detection now uses a uniform spatial grid (`cellSize = 120` world
units). Candidate pairs are gathered per cell, deduplicated, sorted into the
canonical ascending body-index order, and fed to the narrow phase. The pair
emission order is identical to brute-force detection, so results are
bit-for-bit reproducible — a regression test (`Grid broad-phase matches brute
force bit-for-bit`) verifies this over 240 steps on a mixed scene.

The validated impact (Release build, same scenario/hardware, before/after via
`runPerformanceBenchmarks` in the test binary):

| Scenario (bodies) | Brute force µs/step | Spatial grid µs/step | Speedup |
|---|---|---|---|
| Ball Pit (154) | 409.1 | 222.4 | 1.84× |
| High-Restitution Chaos (84) | 153.4 | 138.1 | 1.11× |
| Mixed Shapes Mosaic (104) | 1530.6 | 291.9 | 5.24× |
| Stress Ramp (154) | 413.0 | 187.4 | 2.20× |

The sandbox runs 12 built-in scenarios selectable via the ImGui dropdown:

| # | Scenario | Description |
|---|---|---|
| 1 | Empty World | Floor only — spawn your own bodies |
| 2 | Single Bounce | One circle dropped onto floor |
| 3 | Box Stack | 10-box static stack |
| 4 | Ball Pit (150+) | 150 circles in a walled container |
| 5 | Pyramid | 6-row box pyramid |
| 6 | Orbiting Bodies | Central attractor with 3 orbiting bodies |
| 7. | Friction Ramp | Inclined plane with 4 different-friction blocks |
| 8 | High-Restitution Chaos | 80 high-restitution circles with initial velocities |
| 9 | Mixed Shapes Mosaic | 100 circles + boxes in a walled container |
| 10 | Explosion | Radial impulse burst on 60 circles |
| 11 | Stress Ramp (150+) | 150 circles in a walled container |
| 12 | [BROKEN] Extreme Velocity | Extreme gravity — sanity check for instability detector |

## Diagnostics (M7)

Each physics step is profiled into 5 phases, shown live in the ImGui panel:

- Apply Forces
- Integrate Velocity
- Resolve Collisions
- Integrate Position
- Detect Instability

The instability detector flags NaN/Inf values and bodies exceeding velocity or
position thresholds. Selecting scenario 12 (deliberately broken) demonstrates
the detector catching a divergent simulation.

## Controls

| Key | Action |
|---|---|
| Space | Pause / Resume |
| . (period) | Single step (when paused) |
| R | Reset current scenario |
| F1 | Toggle debug overlay (velocity vectors, AABBs) |
| F2 | Toggle diagnostics panel |
| Delete | Remove selected body |
| ESC | Close |
| Left click (empty) | Spawn circle or box at cursor |
| Left click (body) | Select body |

ImGui panel: scenario dropdown, spawn type (Circle/Box), Clear All, gravity
slider, global restitution + friction sliders, per-body sliders when
selected.

## Build, Test, and Development

Build with MSBuild (Visual Studio 2022):

```bash
# Debug
MSBuild ImpulseEngine.sln /p:Configuration=Debug /p:Platform=x64

# Release
MSBuild ImpulseEngine.sln /p:Configuration=Release /p:Platform=x64

# ASan (AddressSanitizer)
MSBuild ImpulseEngine.sln /p:Configuration=ASan /p:Platform=x64
```

Run the sandbox (in a terminal, not VS):

```bash
x64\Debug\ImpulseEngine.exe
# or headless 30-frame smoke test:
x64\Debug\ImpulseEngine.exe --frames 30
```

Run tests:

```bash
bin\Debug\ImpulseEngineTests.exe
```

**Test results (M8): 39 tests, all passing (Debug + Release + ASan).**

## Agent Guidance

If you are using a coding agent (Claude Code or similar) to work in this
repository, read [`AGENTS.md`](./AGENTS.md) first. It defines the project
structure, build/test commands, coding conventions, and the milestone-based
workflow the agent should follow.

## License

Choose a license before publishing (MIT or BSD-3-Clause are common choices
for portfolio engine projects). Not yet set — see `LICENSE`.
