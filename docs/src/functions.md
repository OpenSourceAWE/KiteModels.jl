```@meta
CurrentModule = KiteModels
```

## Introduction

Most of the functions work on a KPS3 or KPS4 object. For this, the variable s is used.
Such a variable can be created with the lines:

```julia
using KiteSimulators
set = load_settings("system.yaml")
s = KPS3(KCU(set))
```

Or, if you want to use the 4 point kite model:

```julia
using KiteSimulators
set = load_settings("system.yaml")
s = KPS4(KCU(set))
```

Functions with an "!" as last character of the function name modify one of more of their
parameters, in this context mostly the variable s.

## Input functions

```@docs
set_depower_steering!
set_v_wind_ground!
calc_turbulent_wind
get_default_turbulence
set_default_turbulence
```

## Output functions

```@docs
unstretched_length
tether_length
pos_kite
calc_height
calc_elevation
calc_azimuth
calc_azimuth_east
calc_azimuth_north
calc_heading
calc_course
cl_cd
reel_out_speed
winch_force
spring_forces
n_springs
lift_drag
lift_over_drag
v_wind_kite
kite_ref_frame
orient_euler
calc_orient_quat
states
SysState
```

## System definition

A `SystemDefinition` from [KiteGeometry](https://github.com/OpenSourceAWE/KiteGeometry.jl) describes the points, segments, tether and winch of a model, so a log can be drawn by anything that reads a definition. Build it after `init!` and pass it to `save_log`, which stores it in the log; `KiteGeometry.definition` reads it back from the loaded log:

```julia
integrator = init!(s)
definition = system_definition(s)
# ... simulate and log! ...
save_log(logger, definition, "sim_log")
KiteGeometry.definition(load_log("sim_log"))
```

```@docs
system_definition
```

## Saving the system state

The system state of one time step can be saved as human-readable YAML file, for example to
inspect it or to compare two simulations. To log a whole simulation, use a `Logger` from
KiteUtils instead.

```@docs
save_state
load_state
```

## High level simulation interface

```@docs
init!
next_step!
```

## Low level simulation interface

```@docs
clear!
find_steady_state!
residual!
```

## Helper functions

```@docs
copy_examples
copy_examples_3d
install_examples_3d
copy_bin
calc_drag
calculate_rotational_inertia!
calc_set_cl_cd!
calc_aero_forces!
calc_aero_forces
calc_aero_force_moment
aero_point_forces
calc_turn_rates
calc_pitch_rate
calc_particle_forces!
calc_spring_force
calc_drag_forces
calc_drag_force
tether_drag_forces
particle_velocities
UNUSED_FIELDS
UNUSED_FIELDS_KPS3
inner_loop!
loop!
make_jac
assemble_system_definition
spring_name
```
