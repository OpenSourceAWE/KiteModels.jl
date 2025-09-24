# Copyright (c) 2024, 2025 Bart van de Lint, Uwe Fechner
# SPDX-License-Identifier: MIT

SIMPLE = false
T_REF = 35.0 # AMD Ryzen 7840U, Julia 1.11, 1 thread [s]

using Pkg
if ! ("Test" ∈ keys(Pkg.project().dependencies))
    using TestEnv; TestEnv.activate()
end
using KiteModels, LinearAlgebra, Statistics, Test
include("bench_ref.jl")

# Simulation parameters
dt = 0.05
total_time = 10.0  # Longer simulation to see oscillations
vsm_interval = 3
steps = Int(round(total_time / dt))

# Steering parameters
steering_freq = 1/2  # Hz - full left-right cycle frequency
steering_magnitude = 10.0      # Magnitude of steering input [Nm]

# Initialize model
set = load_settings("system_ram.yaml")
set.segments = 3
set_values = [-50, 0.0, 0.0]  # Set values of the torques of the three winches. [Nm]
set.quasi_static = false
set.physical_model = SIMPLE ? "simple_ram" : "ram"

sam = SymbolicAWEModel(set)
sam.set.abs_tol = 1e-2
sam.set.rel_tol = 1e-2
rm("data/model_1.11_ram_dynamic_3_seg.bin"; force=true)

# Initialize at elevation
set.l_tethers[2] += 0.2
set.l_tethers[3] += 0.2
time_ = init!(sam; remake=false, reload=true, bench=true)
@info "Simplify took $time_ seconds"
rel_performance = (T_REF / rel_cpu_performance())/time_
@info "Relative performance: $rel_performance"
@test rel_performance > 0.8

sys = sam.sys
nothing

# Desktop, AMD Ryzen 9 7950X, Julia 1.11:
# - first  run 34.5 seconds
# - second run 21.1 seconds

# Laptop, AMD Ryzen 7 7840U, Julia 1.11:
# - first  run 35.0 seconds
# - second run 24.0 seconds


