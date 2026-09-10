# Repository Guidelines

## Developers vs. Users

This is a solo/portfolio project, but the distinction is still useful for an
agent: `Developer` mode means modifying the engine itself (`src/engine`,
`src/render`, `src/sandbox`, `src/tests`) and its docs. `User` mode means
someone consuming Impulse Engine as a library from an external project. For
now this repo has no external-consumer template — if that's ever needed,
add a `template_project/` directory following the same pattern Chrono uses
(a minimal external CMake project with `find_package`), but do not build
that until the core engine (see Roadmap) is stable.

## Project Structure & Module Organization

Impulse Engine is a CMake-based C++ project. Production code lives under
`src/`:

- `src/engine/` — physics core: `RigidBody`, `Shape` (`CircleShape`,
  `PolygonShape`), collision (`broadphase/`, `narrowphase/`), the impulse
  solver, and `World`. **This module must have zero OpenGL/rendering
  dependencies** — it needs to build and run headlessly for the test suite.
- `src/render/` — OpenGL renderer: shape drawing, debug overlay (AABBs,
  contact points/normals, velocity vectors).
- `src/sandbox/` — the interactive application: window/context setup, ImGui
  panels, and the scenario library (see `docs/03_USER_FLOW.md` §3 for the
  full scenario list).
- `src/diagnostics/` — frame-time profiler (per-phase timers) and the
  instability detector (NaN/Inf/threshold checks with logging).
- `src/tests/unit_tests/` — Catch2 unit tests for math/collision/solver
  correctness, plus headless scenario/stress tests (no rendering) that run
  every scenario and assert no NaN/escape.
- `data/` — scenario configs, shader source, fonts/assets for ImGui.
- `contrib/` — bundled third-party sources (GLFW, an OpenGL loader such as
  glad, Dear ImGui, Catch2), pulled in as git submodules.
- `docs/` — design docs and this file's companion manuals.

Keep new code near the owning module and update that directory's local
`CMakeLists.txt`. Do not let `src/render` or `src/sandbox` headers leak into
`src/engine` — that boundary is what keeps the physics core testable
headlessly and is load-bearing for the CI test suite (see
`docs/04_TESTING_AND_PROFILING.md` §2.2).

## Build, Test, and Development Commands

In-source builds are not supported; configure into `build/` or another
separate directory.

- `git submodule update --init --recursive`: fetch bundled third-party
  sources (GLFW, ImGui, Catch2). A missing submodule surfaces as a
  confusing "target not found" CMake error, not a clear message — check
  this first if configure fails on a fresh clone.
- `cmake -S . -B build -G Ninja -DBUILD_TESTING=ON -DBUILD_SANDBOX=ON`:
  configure a local development build.
- `cmake --build build -j`: compile the configured targets.
- `ctest --test-dir build --output-on-failure`: run the registered test
  suite (unit tests + headless scenario tests).
- `CMakePresets.json` should carry at least three configurations: `debug`,
  `release`, and `debug-asan` (Debug build + `-fsanitize=address,undefined`
  on Linux/Clang, or `/RTC1` + debug heap checks left on for MSVC). Use
  `--preset=<name>` rather than reconstructing flags by hand, and reach for
  `debug-asan` specifically when chasing crashes that look like memory
  corruption (see `docs/04_TESTING_AND_PROFILING.md` §6).

## Build System Conventions

- **Physics core stays headless.** `src/engine` must compile and link into
  the test binary without pulling in GLFW/OpenGL/ImGui. If a physics change
  seems to require a rendering dependency, that's a sign the abstraction is
  leaking — stop and reconsider rather than adding the include.
- **Fixed-timestep physics, variable-rate render.** The accumulator pattern
  lives in `src/sandbox` (the app loop), not in `src/engine::World` itself —
  `World::step(dt)` takes a fixed dt and has no opinion about wall-clock
  time or frame rate.
- **Scenario configs are data, not hardcoded control flow** once past M6
  (see Roadmap) — prefer a small struct/JSON description of initial bodies
  over a hardcoded function per scenario, so new scenarios don't require
  touching sandbox application code.
- **No external physics library** (Box2D, Chipmunk2D, etc.) in `src/engine`.
  Math-only third-party headers (e.g. a small vector/matrix library) are
  fine if justified, but the collision and solver logic itself is the point
  of the project and must be original.

## Coding Style & Naming Conventions

- C++17 or later. RAII throughout; no raw owning pointers — use
  `std::unique_ptr` / `std::vector` for body and shape storage.
- `PascalCase` for types (`RigidBody`, `PolygonShape`, `World`),
  `camelCase` for functions and methods (`applyImpulse`, `stepWorld`),
  `snake_case` for local variables and file/folder names.
- Format with `clang-format`; commit a `.clang-format` at the repo root
  (Chromium base or LLVM base, 4-space indent, is a reasonable default) and
  format only lines you changed — don't run a whole-file reformat over
  existing code, since that produces noisy, unrelated diffs.
- Keep headers and sources paired in the same module directory.
- Prefix unit test files `utest_` (e.g. `utest_vec2_math.cpp`,
  `utest_impulse_resolution.cpp`) and headless scenario/stress tests
  `stest_` (e.g. `stest_ball_pit_150.cpp`), so `ctest -L unit` and
  `ctest -L stress` can filter by label.

## Testing Guidelines

- Catch2 for C++ unit tests. Register new test files in the local
  `src/tests/unit_tests/CMakeLists.txt`.
- Every new piece of physics math ships with a unit test in the same PR —
  not after. See `docs/04_TESTING_AND_PROFILING.md` §2.1 for the required
  categories (vector math, narrow-phase known-answer tests, impulse
  resolution against analytical results, integration vs. closed-form
  kinematics).
- Headless scenario tests run every scenario in `docs/03_USER_FLOW.md` §3
  for a fixed number of steps and assert no NaN/Inf and no body escaping a
  bounded container. These must stay renderer-free so they can run in CI.
- When a bug is found via manual sandbox testing, add a regression test
  that reproduces it before fixing it, and log it in
  `docs/04_TESTING_AND_PROFILING.md` §5 (edge-case register): symptom, root
  cause, fix, verification.
- Run `ctest --test-dir build --output-on-failure` before considering any
  milestone (see Roadmap) complete.

## Commit & Milestone Guidelines

Recent-history style: short, imperative, capitalized commit subjects — e.g.
`Add SAT narrow-phase for polygon-polygon`, `Fix divide-by-zero in Vec2::normalize`.
Commit at each milestone boundary (see `docs/06_ROADMAP.md`) with a message
describing what became true, not what files changed.

Before starting a milestone, confirm the previous one's done-criteria are
met (build succeeds, relevant tests pass, sandbox runs without crashing).
If a milestone can't be completed without deviating from
`docs/02_ARCHITECTURE.md` or adding a new third-party dependency, stop and
ask rather than proceeding silently.
