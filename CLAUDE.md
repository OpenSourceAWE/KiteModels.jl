<!--
SPDX-FileCopyrightText: 2025 Uwe Fechner
SPDX-License-Identifier: MIT
-->

# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this package is

KiteModels.jl is a Julia package implementing physics-based simulation models of kite power systems
(airborne wind energy). Kites and tethers are modeled as implicit DAE systems:
`residual = f(y, yd)`, a 3D mass-spring-damper system with reel-out, solved with Sundials IDA or an
OrdinaryDiffEq BDF solver. Scientific background: <http://arxiv.org/abs/1406.6218>.

It is one package within "Julia Kite Power Tools", a family of packages maintained under the
OpenSourceAWE / aenarete GitHub orgs. Related packages this repo depends on and interoperates with:
`KiteUtils` (settings, state types, `AbstractKiteModel`), `WinchModels`, `KitePodModels`,
`AtmosphericModels`, and (for the newer symbolic model) `VortexStepMethod` /
`SymbolicAWEModels.jl` (moved out to its own package as of Feb 2026 — do not expect to find it here).

## Architecture

### Two kite models, one shared interface

- `src/KPS3.jl` — one-point model: the kite is a single point mass. Less realistic turning
  dynamics, needs an external turn-rate controller, but cheap.
- `src/KPS4.jl` — four-point model: 4 point masses (KCU=p7, A=p8, B=p9, C=p10, D=p11) connected by
  9 springs (`SPRINGS_INPUT`), with aerodynamic forces on 3 of the 4 points. Has rotational inertia
  in every axis, so it turns realistically; this is the workhorse model.

Both `KPS3` and `KPS4` are mutable structs parameterized as `{S, T, P, Q, SP}` (scalar type, vector
type, #points, #springs, spring type) and subtype `AbstractKiteModel` (defined in `KiteUtils`).
`src/KiteModels.jl` is the top-level module: it declares shared constants/type aliases, re-exports
`KitePodModels`, `WinchModels`, `AtmosphericModels` (via `@reexport`), and `include`s `KPS4.jl`,
`KPS3.jl`, `utils.jl` in that order — there's no separate build step, editing a `src/*.jl` file and
reloading (Revise) is enough.

Shared type aliases (`src/KiteModels.jl`): `SimFloat = Float64` for all sim scalars, `KVec3 =
MVector{3,SimFloat}` (mutable, stack-allocated), `SVec3 = SVector{3,SimFloat}` (immutable).

### Simulation lifecycle (same shape for KPS3 and KPS4)

```julia
set  = load_settings("system.yaml")   # from KiteUtils, reads data/*.yaml
kcu  = KCU(set)                       # KitePodModels
kps4 = KPS4(kcu)                      # or KPS3(kcu)
init!(kps4)                           # or find_steady_state!(kps4)
next_step!(kps4, integrator; set_speed, dt)   # advance one timestep
```

Getters follow a `calc_*`/noun pattern: `pos_kite`, `winch_force`, `lift_drag`, `calc_elevation`,
`calc_azimuth`, `calc_heading`, `calc_course`, `tether_length`, `reel_out_speed`, `cl_cd`, etc. — see
the `export` list at the top of `src/KiteModels.jl` for the full public API.

### Configuration

All physical/numerical parameters live in YAML under `data/` (`settings.yaml`, `system.yaml`, plus
versioned variants like `settings_v9b.yaml`). Key sections: `system` (segments, sample_freq,
sim_time), `solver` (abs_tol, rel_tol, IDA vs DFBDF), `kite`/`kps4` (mass, area, aero coefficients),
`tether` (d_tether, e_tether, damping). `Settings` objects come from `KiteUtils`, not this package.

### Coordinate system

NED (North-East-Down) reference frame for orientation; azimuth is calculated in the wind reference
frame (so it stays correct across wind direction changes during flight).

## Development commands

This is a Julia package developed with a workspace: `Project.toml` declares
`[workspace] projects = ["examples", "examples_3d", "docs", "test"]`, each with its own
`Project.toml`/manifest.

- **Install/setup**: `cd bin && ./install` (installs Julia via juliaup, sets up `Revise` globally,
  runs `setup_env`). `./install --update` refreshes an existing setup.
- **Launch a dev REPL**: `./bin/run_julia` (activates the right project, forwards script args).
- **Build a system image** (much faster startup/time-to-first-plot): `cd bin && ./create_sys_image`
  (can take ~30 min); relaunch via `./bin/run_julia` afterward.
- **Run the full test suite** (from a Julia session, project = `test/`):
  ```julia
  include("test/runtests.jl")
  ```
  This walks `test/` and includes every `test-*.jl` file as its own `@testset`, then (unless
  `BUILD_IS_PRODUCTION_BUILD=false`) also runs `bench3.jl`/`bench4.jl` performance regression
  benchmarks. Full run can take up to ~60 minutes.
- **Run a single test file**: include it directly, e.g.
  `include("test/test-kps4.jl")` from a REPL with the `test/` project active — test files are named
  `test-<topic>.jl` (hyphen, not underscore) so `runtests.jl`'s file-matching regex picks them up.
- **Skip slow benchmarks**: set `ENV["BUILD_IS_PRODUCTION_BUILD"] = "false"` before running
  `runtests.jl`.
- **Aqua QA** (stale deps, ambiguities, piracy): `test/test-aqua.jl`. Note `piracies=false` is
  intentional — `norm` is deliberately type-pirated for performance.
- **Run examples**: `include("examples/menu.jl")` or `include("examples/menu2.jl")` for interactive
  menus, or run a script directly (e.g. `include("examples/reel_out_4p.jl")`). Examples/data files
  are copied into a user project with `KiteModels.install_examples()` /
  `KiteModels.install_examples_3d()` / `KiteModels.copy_bin()`.
- **Build docs locally**:
  ```julia
  using Pkg; Pkg.activate("docs"); include("docs/make.jl"); Pkg.activate(".")
  ```
- **Formatting**: `.JuliaFormatter.toml` sets `indent = 4`, `margin = 92`. Format via JuliaFormatter,
  not by hand-editing whitespace.
- **License linting**: `bin/reuse_lint` runs `pipx run reuse lint` (every file must carry an
  SPDX header or `.license` sidecar file — REUSE.toml lists exceptions).

## Coding style (from `docs/src/advanced.md`)

- Line length limit: 120 characters.
- Avoid hard-coded numeric constants (e.g. `9.81`) — use/define a named constant like `G_EARTH`, or
  read the value from settings.
- Avoid dot-broadcast operators unless actually needed: prefer `norm1 ~ norm(segment)` over
  `norm1 .~ norm(segment)`.
- Use `\cdot` for dot products; space after commas (`force_eqs[j, i]`); space around binary operators
  like `+`/`*` (`0.5 * (s.pos[s.i_C] + s.pos[s.i_D])`), except tight index arithmetic like
  `mass_tether_particle[i-1]`.
- Align `=`/equation signs vertically across related lines for readability.
- Install `Revise` into the **global** Julia environment, never as a project dependency.
- To load settings ad hoc in a script, use `se("system_3l.yaml")` (loads relative to the active
  project's `data/` dir).

## Release checklist (from CONTRIBUTING.md)

Before cutting a release: all tests pass; every example is reachable from the menu and runs; diff
`create_sys_image2.jl` against `create_sys_image.jl` with `meld`; verify
`test_installation`/`test_installation2`/`test_installation3` scripts work; diff `README.md` against
`docs/src/index.md` with `meld` and reconcile; test install on Linux (both supported Julia versions)
and on Windows with a fresh `.julia` folder; run `bin/reuse_lint`.
