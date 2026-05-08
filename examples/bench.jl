# Copyright (c) 2022, 2024 Uwe Fechner
# SPDX-License-Identifier: MIT
#
# Benchmark example for the KPS3 (one-point) kite model.
# Runs a simulated flight for a configurable number of time steps
# and prints performance statistics at the end.

using Printf
using KiteModels
using KiteUtils: Settings, load_settings

set::Settings = deepcopy(load_settings("system.yaml"))

# the following values can be changed to match your interest
dt::Float64 = 0.05
set.solver="DFBDF"              # IDA or DFBDF
set.linear_solver="GMRES"       # GMRES, LapackDense or Dense
STEPS = 600
PRINT = false
STATISTIC = false
# end of user parameter section #

kcu::KCU = KCU(set)
kps3::KPS3 = KPS3(kcu)
println("Number of states: $(KiteModels.states(kps3))")

function simulate(integrator, steps)
    iter = 0
    for _ = 1:steps
        if PRINT
            lift, drag = KiteModels.lift_drag(kps3)
            @printf "%.2f: " round(integrator.t, digits = 2)
            println("lift, drag  [N]: $(round(lift, digits=2)), $(round(drag, digits=2))")
        end

        next_step!(kps3, integrator; set_speed = 0, dt = dt)
        iter += kps3.iter
    end
    iter / steps
end

integrator = KiteModels.init!(kps3; delta = 0.001, stiffness_factor = 0.1, prn = STATISTIC)

println("\nStarting simulation...")
simulate(integrator, 100)
runtime = @elapsed av_steps = simulate(integrator, STEPS-100)
println("\nTotal simulation time: $(round(runtime, digits=3)) s")
speed = (STEPS-100) / runtime * dt
println("Simulation speed: $(round(speed, digits=2)) times realtime.")
lift, drag = KiteModels.lift_drag(kps3)
println("lift, drag  [N]: $(round(lift, digits=2)), $(round(drag, digits=2))")
println("Average number of callbacks per time step: $(round(av_steps, digits=2))")

# Ryzen 7950X, GMRES solver
# Total simulation time: 0.418 s
# Simulation speed: 59.77 times realtime.
# lift, drag  [N]: 597.54, 129.31
# Average number of callbacks per time step: 234.5

# Ryzen 7950X, LapackDense solver
# Total simulation time: 0.36 s
# Simulation speed: 69.39 times realtime.
# lift, drag  [N]: 597.55, 129.31
# Average number of callbacks per time step: 227.8

# Ryzen 7950X, DFBDF solver
# Total simulation time: 0.105 s
# Simulation speed: 238.6 times realtime.
# lift, drag  [N]: 598.0, 129.43
# Average number of callbacks per time step:  64.0
