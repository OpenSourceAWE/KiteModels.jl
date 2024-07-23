```@meta
CurrentModule = KiteModels
```
## Introduction
Most of the functions work on a KPS3 or KPS4 object. For this, the variable s is used.
Such a variable can be created with the lines:
```julia
using KiteSimulators
const s = KPS3(KCU(se()))
```
Or, if you want to use the 4 point kite model:
```julia
using KiteSimulators
const s = KPS4(KCU(se()))
```
Functions with an "!" as last character of the function name modify one of more of their
parameters, in this context mostly the variable s.

## Input functions
```@docs
set_depower_steering!
set_v_wind_ground!
```

## Output functions
```@docs
unstretched_length
tether_length
pos_kite
calc_height
calc_elevation
calc_azimuth
calc_heading
calc_course
winch_force
spring_forces
lift_drag
lift_over_drag
v_wind_kite
kite_ref_frame
orient_euler
SysState
```

## High level simulation interface
```@docs
init_sim!
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
copy_bin
calc_drag
calc_set_cl_cd!
calc_aero_forces!
calc_particle_forces!
inner_loop!
loop!
```
