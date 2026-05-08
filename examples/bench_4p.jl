# Copyright (c) 2022, 2024 Uwe Fechner
# SPDX-License-Identifier: MIT
#
# Benchmark example for the KPS4 (four-point) kite model.
# Runs a simulated flight with optional plotting and prints
# performance statistics at the end.

using KiteModels: KPS4, init!, lift_drag, next_step!, reactivate_host_app, states, winch_force
using KitePodModels: KCU
using KiteUtils: Settings, load_settings
using Printf
using Pkg
if ! ("ControlPlots" ∈ keys(Pkg.project().dependencies))
    Pkg.activate("examples")
end

using ControlPlots

set::Settings = deepcopy(load_settings("system.yaml"))

# the following values can be changed to match your interest
dt::Float64 = 0.05
set.solver = "DFBDF"              # IDA or DFBDF
set.linear_solver = "GMRES"       # GMRES, LapackDense or Dense
STEPS = 200
PRINT = false
STATISTIC = false
const PLOT = true
# end of user parameter section #

kcu::KCU   = KCU(set)
kps4::KPS4 = KPS4(kcu)
println("Number of states: $(states(kps4))")

v_time  = zeros(STEPS)
v_speed = zeros(STEPS)
v_force = zeros(STEPS)

function simulate(integrator, steps, offset = 0)
    iter = 0
    for i in 1:steps
        acc = 0.0
        if kps4.t_0 > 3.0 + offset
            acc = 0.1
        end
        v_time[i] = kps4.t_0
        v_speed[i] = kps4.v_reel_out
        v_force[i] = winch_force(kps4)
        set_speed = kps4.sync_speed+acc*dt
        if PRINT
            lift, drag = lift_drag(kps4)
            @printf "%.2f: " round(integrator.t, digits = 2)
            println("lift, drag  [N]: $(round(lift, digits=2)), $(round(drag, digits=2))")
        end

        next_step!(kps4, integrator; set_speed, dt = dt)
        iter += kps4.iter
    end
    iter / steps
end

integrator = init!(kps4, delta = 0.001, stiffness_factor = 0.1, prn = STATISTIC)

println("\nStarting simulation...")
simulate(integrator, 100, 100)
runtime = @elapsed av_steps = simulate(integrator, STEPS-100)
println("\nSolver: $(set.solver)")
println("Total simulation time: $(round(runtime, digits=3)) s")
speed = (STEPS-100) / runtime * dt
println("Simulation speed: $(round(speed, digits=2)) times realtime.")
if PLOT
    local p
    p = plotx(
        v_time[1:(STEPS-100)],
        v_speed[1:(STEPS-100)],
        v_force[1:(STEPS-100)];
        ylabels = ["v_reelout  [m/s]", "tether_force [N]"],
        fig = "winch",
    )
    display(p)
    reactivate_host_app()
end
lift, drag = lift_drag(kps4)
println("lift, drag  [N]: $(round(lift, digits=2)), $(round(drag, digits=2))")
println("Average number of callbacks per time step: $av_steps")

# Ryzen 7950X, Solver: DFBDF
# Total simulation time: 0.043 s
# Simulation speed: 116.99 times realtime.
# lift, drag  [N]: 798.46, 314.82
# Average number of callbacks per time step: 126.41

# Ryzen 7950X, Solver: DFBDF, Julia 1.12, 04-03-2026
# Total simulation time: 0.007 s
# Simulation speed: 751.89 times realtime.
# lift, drag  [N]: 498.79, 106.75
# Average number of callbacks per time step: 7.67

# Ryzen 7950X, Solver: DFBDF, Julia 1.11, 04-03-2026
# Total simulation time: 0.006 s
# Simulation speed: 772.5 times realtime.
# lift, drag  [N]: 498.83, 106.73
# Average number of callbacks per time step: 8.21
