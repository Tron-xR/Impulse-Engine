# Impulse Engine

A real-time 2D physics engine developed in C++ with OpenGL visualization. Named after the impulse-based constraint solver at the core of its rigid body dynamics.

## Status

Milestones M0–M4 are complete:

- **M0** — Rendering backbone: OpenGL 3.3 core context, GLFW window/input, GLAD loader, GPU-vertex meshes, window-resize support.
- **M1** — Single rigid body integrates under gravity with semi-implicit Euler (fixed 120 Hz timestep), with an ASan build config.
- **M2** — Static floor (`PlaneShape`) with circle-vs-plane detection and restitution-controlled impulse resolution (`v_out = e·v_in`, mass-independent).
- **M3** — Circle-circle collision: distance narrow-phase, impulse resolution for two moving bodies (momentum-conserving, restitution mixes by `max(eA, eB)`), brute-force O(n²) broad phase within `World`.
- **M4** — Convex polygon shapes via SAT narrow-phase: polygon-vs-plane, polygon-polygon (face/overlap axis), and circle-polygon (closest-point) detection. World solver upgraded to sequential impulse (4 iterations) plus a positional-correction pass (percent 0.8, slop 0.01) so a 4-box stack rests within <1 px.

19 unit tests cover integration, collision detection (circle/circle, circle/plane, polygon/plane, polygon/polygon, circle/polygon), known-answer impulses, conservation, elastic velocity swap, resting/NaN-regression, micro-bounce suppression, and box-stack stability. All pass in both Debug and ASan builds.

## Features

### Current
- OpenGL 3.3 core renderer (compile-time-shader, uniform-driven circle/rect meshes)
- GLFW window management with input handling; ESC / SPACE to exit; `--frames N` auto-close for headless smoke runs
- Physics core `physics/` — deterministic step loop (semi-implicit Euler), headless (no renderer dependency)
- Shapes: `CircleShape`, `PlaneShape` (infinite half-space), `PolygonShape` (convex, CCW vertices, `makeBox`)
- Collision `collision/` — SAT narrow-phase for circle/plane, circle/circle, polygon/plane, polygon/polygon, circle/polygon; two-body impulse resolution + positional correction
- Sequential impulse solver in `World` — 4 iterations + positional correction (percent 0.8, slop 0.01); restitution velocity threshold suppresses resting micro-bounce
- Restitution mixing `max`, mass-independent bounce
- Tests — custom check-based harness, headless-friendly (no modal dialogs)

### In Development / Planned
- **M5** — Friction and solver-iteration tuning (current; iterations + positional correction already required an early pull for the M4 box stack)
- **M6** — Sandbox UI (mouse picking, spawning, non-deterministic scenarios)
- **M7** — Diagnostics & instability detection (contact-point/normal overlay, NaN guardrails)
- **M8** — Performance pass: broad-phase spatial partitioning (grid / BVH), profiled against a benchmark scenario
- **M9** — Test/stress-suite hardening, CI test runner
- **M10** — Polish & writeup, final numbers captured into the docs

## Dependencies

| Library | Version | Purpose |
|---------|---------|---------|
| OpenGL | 3.3 Core | Rendering API |
| GLAD | 0.1.36 | OpenGL function loader |
| GLFW | 3.4 | Window, context, and input |

All dependencies except GLFW are included in the repository. GLFW 3.4 (64-bit) is expected at `C:\glfw-3.4.bin.WIN64\`.

## Build

### Prerequisites
- Windows 10+
- Visual Studio 2022 (v143 toolset, Windows SDK 10.0)
- GLFW 3.4 binaries installed at `C:\glfw-3.4.bin.WIN64\`

### Steps
```bash
# Open the solution
ImpulseEngine.sln

# Build via Visual Studio (F7) or MSBuild:
msbuild ImpulseEngine.sln /p:Configuration=Debug /p:Platform=x64
```

Executables are output to `x64/Debug/ImpulseEngine.exe` (app) and `bin/Debug/ImpulseEngineTests.exe` (tests). An `ASan` configuration builds both with Address Sanitizer.

## Running

```bash
# Physics demo — a 4-box stack plus five circles (varied mass/radius/restitution) plus a falling box bounce and collide on the floor
x64\Debug\ImpulseEngine.exe
x64\Debug\ImpulseEngine.exe --frames 30     # auto-close after 30 frames (smoke test)

# Unit tests (19)
bin\Debug\ImpulseEngineTests.exe

# Sanitized run — catches leaks / UB, prints findings, still exits clean when clean
msbuild ImpulseEngine.sln /p:Configuration=ASan /p:Platform=x64
bin\ASan\ImpulseEngineTests.exe
x64\ASan\ImpulseEngine.exe --frames 30
```

## Controls

| Key | Action |
|-----|--------|
| ESC | Close window |
| SPACE | Close window |

## Project Structure

```
ImpulseEngine/
├── physics/                     # Headless physics core
│   ├── Vec2.h                   # 2D vector math (dot/cross/normalize/rotate)
│   ├── RigidBody.h              # Body: mass/invMass, restitution, angle, Shape (circle/plane/polygon)
│   └── World.h                  # Timestep loop, body registry, pair detection, sequential-impulse solver
├── collision/
│   └── CollisionDetect.h        # Manifold, SAT narrow-phase detectors, impulse + positional correction
├── tests/
│   └── test_physics.cpp         # 19 unit tests (CHECK-macro harness)
├── glad/                        # GLAD loader (generated)
│   ├── include/glad/glad.h
│   └── src/glad.c
├── main.cpp                     # Entry point — render loop, demo scene, input
├── ImpulseEngine.sln            # Visual Studio 2022 solution
├── ImpulseEngine.vcxproj        # App project (Debug / Release / ASan)
├── ImpulseEngineTests.vcxproj   # Test project (Debug / Release / ASan)
└── README.md
```

## License

MIT