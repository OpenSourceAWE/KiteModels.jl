# KiteModels.jl Copilot Instructions

## Project Overview

KiteModels.jl is a Julia package for simulating kite power systems (airborne wind energy). It provides physics-based models of tethered kites using mass-spring-damper systems with aerodynamic forces.

## Architecture

### Core Models (in `src/`)

- **KPS3** (`KPS3.jl`): One-point kite model - kite as single point mass
- **KPS4** (`KPS4.jl`): Four-point kite model - 4 point masses with rotational inertia
- **SymbolicAWEModel** (`symbolic_awe_model.jl`): RAM air kite using ModelingToolkit + VortexStepMethod for aerodynamics

All models inherit from `AbstractKiteModel` (defined in KiteUtils.jl). The main module `KiteModels.jl` re-exports dependencies: KitePodModels, WinchModels, AtmosphericModels.

### Key Data Structures

- `Settings`: Configuration loaded from YAML (`data/settings.yaml`, `system.yaml`)
- `SystemStructure` (`system_structure.jl`): Point-mass topology for SymbolicAWEModel
- `Spring`/`SP`: Spring-damper connections between point masses

### Simulation Flow

1. Load settings: `set = load_settings("system.yaml")`
2. Create KCU and model: `kcu = KCU(set); kps4 = KPS4(kcu)`
3. Initialize: `init!(kps4)` or `find_steady_state!(kps4)`
4. Step: `next_step!(kps4, integrator; set_speed, dt)`

## Development Workflows

### Running Tests

```julia
include("test/runtests.jl")  # Full test suite (~60 min)
```

Skip slow MTK tests: `ENV["NO_MTK"] = "1"` before testing.

### Running Examples

```julia
include("examples/menu.jl")  # Interactive menu
include("examples/reel_out_4p.jl")  # Direct execution
```

### System Image (faster startup)

```bash
cd bin && ./create_sys_image  # Creates precompiled image
./bin/run_julia               # Launch with system image
```

## Code Conventions

### Type Aliases

- `SimFloat = Float64`: All simulation floats
- `KVec3 = MVector{3, SimFloat}`: Mutable 3D vectors (stack-allocated)
- `SVec3 = SVector{3, SimFloat}`: Immutable 3D vectors

### Key Functions Pattern

- `init!(model)`: Initialize/reinitialize model state
- `next_step!(model, integrator; kwargs...)`: Advance simulation
- `find_steady_state!(model)`: Solve for equilibrium
- Getters: `pos_kite()`, `winch_force()`, `lift_drag()`, `calc_elevation()`

### Configuration

Settings are in `data/` as YAML files. Key sections:

- `system`: segments, sample_freq, sim_time
- `solver`: abs_tol, rel_tol, solver type (IDA/DFBDF)
- `kite`/`kps4`: physical parameters (mass, area, aerodynamic coefficients)
- `tether`: d_tether, e_tether, damping

### Coordinate System

- NED (North-East-Down) reference frame for orientation
- Azimuth calculated in wind reference frame
- Elevation from ground to kite

## External Dependencies

- **OrdinaryDiffEqBDF/Sundials**: ODE solvers
- **ModelingToolkit**: Symbolic model generation (SymbolicAWEModel)
- **VortexStepMethod**: Aerodynamics for RAM air kites
- **KiteUtils**: Settings, state types, utilities
- **ControlPlots** (optional extension): Plotting via `ext/KiteModelsControlPlotsExt.jl`

## Testing Notes

- Benchmark tests (`bench3.jl`, `bench4.jl`) verify performance regressions
- Use `BUILD_IS_PRODUCTION_BUILD=false` to skip benchmarks in CI
- Test files in `test/` mirror source structure
