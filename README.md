# Impulse Engine

A real-time 2D physics engine developed in C++ with OpenGL visualization. Named after the impulse-based constraint solver at the core of its rigid body dynamics.

## Status

Milestones M0–M3 are complete:

- **M0** — Rendering backbone: OpenGL 3.3 core context, GLFW window/input, GLAD loader, GPU-vertex meshes, window-resize support.
- **M1** — Single rigid body integrates under gravity with semi-implicit Euler (fixed 120 Hz timestep), with an ASan build config.
- **M2** — Static floor (`PlaneShape`) with circle-vs-plane detection and restitution-controlled impulse resolution (`v_out = e·v_in`, mass-independent).
- **M3** — Circle-circle collision: distance narrow-phase, impulse resolution for two moving bodies (momentum-conserving, restitution mixes by `max(eA, eB)`), brute-force O(n²) broad phase within `World`.

14 unit tests cover integration, collision detection, known-answer impulses, conservation, equal-mass elastic velocity swap, and resting/NaN-regression cases. All pass in both Debug and ASan builds.

## Features

### Current
- OpenGL 3.3 core renderer (compile-time-shader, uniform-driven circle/rect meshes)
- GLFW window management with input handling; ESC / SPACE to exit; `--frames N` auto-close for headless smoke runs
- Physics core `physics/` — deterministic step loop (semi-implicit Euler), headless (no renderer dependency)
- Shapes: `CircleShape`, `PlaneShape` (infinite half-space, normal + signed offset + restitution)
- Collision `collision/` — circle-circle and circle-vs-plane narrow phase, two-body impulse resolution
- Restitution mixing `max`, mass-independent bounce
- Tests — custom check-based harness, headless-friendly (no modal dialogs)

### In Development / Planned
- **M4** — Polygon (AABB/rect) shapes + AABB broad-phase culling
- **M5** — Friction, solver iterations, positional correction (stacking)
- **Constraint Solver** — Sequential impulse solver for contacts, joints
- **Broad-Phase** — Spatial partitioning (grid / BVH) for efficient collision culling, profiled against a benchmark scenario
- **Narrow-Phase** — SAT for convex polygons, AABB overlap tests
- **Debug Visualization** — Render collision shapes, contact points, normals, and manifolds
- **Demo Sandbox** — Interactive scene with stacked bodies, falling shapes, and mouse picking

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
# Physics demo — five circles with varied mass/radius/restitution bounce and collide on the floor
x64\Debug\ImpulseEngine.exe
x64\Debug\ImpulseEngine.exe --frames 30     # auto-close after 30 frames (smoke test)

# Unit tests (14)
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
│   ├── RigidBody.h              # Body: mass/invMass, restitution, Shape (circle/plane)
│   └── World.h                  # Timestep loop, body registry, collision resolution
├── collision/
│   └── CollisionDetect.h        # Manifold, narrow-phase detectors, impulse resolution
├── tests/
│   └── test_physics.cpp         # 14 unit tests (CHECK-macro harness)
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