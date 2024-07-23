# KiteModels

[![Stable](https://img.shields.io/badge/docs-stable-blue.svg)](https://ufechner7.github.io/KiteModels.jl/stable)
[![Dev](https://img.shields.io/badge/docs-dev-blue.svg)](https://ufechner7.github.io/KiteModels.jl/dev)
[![CI](https://github.com/ufechner7/KiteModels.jl/actions/workflows/CI.yml/badge.svg)](https://github.com/ufechner7/KiteModels.jl/actions/workflows/CI.yml)
[![Coverage](https://codecov.io/gh/ufechner7/KiteModels.jl/branch/main/graph/badge.svg)](https://codecov.io/gh/ufechner7/KiteModels.jl)

## Kite power system models, consisting of tether and kite
The model has the following subcomponents, implemented in separate packages:
- AtmosphericModel from [AtmosphericModels](https://github.com/aenarete/AtmosphericModels.jl)
- WinchModel from [WinchModels](https://github.com/aenarete/WinchModels.jl) 
- KitePodModel from  [KitePodModels](https://github.com/aenarete/KitePodModels.jl)

This package is part of Julia Kite Power Tools, which consist of the following packages:
<p align="center"><img src="./docs/src/kite_power_tools.png" width="500" /></p>

## What to install
If you want to run simulations and see the results in 3D, please install the meta package  [KiteSimulators](https://github.com/aenarete/KiteSimulators.jl) . If you are not interested in 3D visualization or control you can just install this package.

## Installation
Install [Julia 1.10](http://www.julialang.org) or later using [juliaup](https://github.com/JuliaLang/juliaup), if you haven't already. You can add KiteModels from  Julia's package manager, by typing 
```julia
using Pkg
pkg"add KiteModels"
``` 
at the Julia prompt. You can run the unit tests with the command:
```julia
pkg"test KiteModels"
```

## One point model
This model assumes the kite to be a point mass. This is sufficient to model the aerodynamic forces, but the dynamic concerning the turning action of the kite is not realistic.
When combined with a controller for the turn rate it can be used to simulate a pumping kite power system with medium accuracy.

## Four point model
This model assumes the kite to consist of four-point masses with aerodynamic forces acting on points B, C and D. It reacts much more realistically than the one-point model because it has rotational inertia in every axis.
<p align="center"><img src="./docs/src/4-point-kite.png" width="200" /></p>

## Tether
The tether is modeled as point masses, connected by spring-damper elements. Aerodynamic drag is modeled realistically. When reeling out or in the unstreched length of the spring-damper elements
is varied. This does not translate into physics directly, but it avoids adding point masses at run-time, which would be even worse because it would introduce discontinuities. When using
Dyneema or similar high strength materials for the tether the resulting system is very stiff which is a challenge for the solver.

## Further reading
These models are described in detail in [Dynamic Model of a Pumping Kite Power System](http://arxiv.org/abs/1406.6218).

## Replaying log files
If you want to replay old flight log files in 2D and 3D to understand and explain better how kite power systems work, please have a look at [KiteViewer](https://github.com/ufechner7/KiteViewer) . How new log files can be created and replayed is explained in the documentation of [KiteSimulators](https://github.com/aenarete/KiteSimulators.jl) .

## Licence
This project is licensed under the MIT License. Please see the below WAIVER in association with the license.

## WAIVER
Technische Universiteit Delft hereby disclaims all copyright interest in the package “KiteModels.jl” (models for airborne wind energy systems) written by the Author(s).

Prof.dr. H.G.C. (Henri) Werij, Dean of Aerospace Engineering

## Donations
If you like this software, please consider donating to https://gofund.me/508e041b .

## See also
- [Research Fechner](https://research.tudelft.nl/en/publications/?search=Fechner+wind&pageSize=50&ordering=rating&descending=true) for the scientic background of this code
- The meta-package  [KiteSimulators](https://github.com/aenarete/KiteSimulators.jl)
- the package [KiteUtils](https://github.com/ufechner7/KiteUtils.jl)
- the packages [WinchModels](https://github.com/aenarete/WinchModels.jl) and [KitePodModels](https://github.com/aenarete/KitePodModels.jl) and [AtmosphericModels](https://github.com/aenarete/AtmosphericModels.jl)
- the packages [KiteControllers](https://github.com/aenarete/KiteControllers.jl) and [KiteViewers](https://github.com/aenarete/KiteViewers.jl)

**Documentation** [Stable Version](https://ufechner7.github.io/KiteModels.jl/stable) --- [Development Version](https://ufechner7.github.io/KiteModels.jl/dev)


Author: Uwe Fechner (uwe.fechner.msc@gmail.com)
