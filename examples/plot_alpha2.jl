# Copyright (c) 2022, 2024 Uwe Fechner
# SPDX-License-Identifier: MIT
#
# Simulates the KPS4 kite model and plots the angle of attack (alpha_2)
# alongside reel-out speed and tether force over time. Useful for
# analyzing the kite's aerodynamic angle response during reel-out.

using Printf
using KiteModels
using KiteUtils: Settings, load_settings

using Pkg
if ! ("MakieControlPlots" ∈ keys(Pkg.project().dependencies))
    Pkg.activate("examples")
end
using MakieControlPlots

set::Settings = deepcopy(load_settings("system.yaml"))

# the following values can be changed to match your interest
dt::Float64 = 0.05
set.solver = "DFBDF"              # IDA or DFBDF
set.linear_solver = "GMRES"       # GMRES, LapackDense or Dense
STEPS = 200
PRINT = false
STATISTIC = false
PLOT = true
# end of user parameter section #

kcu::KCU = KCU(set)
kps4::KPS4 = KPS4(kcu)

v_time = zeros(STEPS)
v_speed = zeros(STEPS)
v_force = zeros(STEPS)
alpha_2 = zeros(STEPS)
alpha_2b = zeros(STEPS)

function simulate(integrator, steps, offset=0)
    iter = 0
    for i in 1:steps
        acc = 0.0
        if kps4.t_0 > 3.0 + offset
            acc = 0.1
        end
        v_time[i] = kps4.t_0
        v_speed[i] = kps4.v_reel_out
        v_force[i] = winch_force(kps4)
        alpha_2[i] = kps4.alpha_2
        set_speed = kps4.sync_speed+acc*dt
        if PRINT
            lift, drag = KiteModels.lift_drag(kps4)
            @printf "%.2f: " round(integrator.t, digits=2)
            println("lift, drag  [N]: $(round(lift, digits=2)), $(round(drag, digits=2))")
        end

        next_step!(kps4, integrator; set_speed, dt=dt)
        iter += kps4.iter
    end
    iter / steps
end

integrator = KiteModels.init!(kps4, stiffness_factor=0.5, prn=STATISTIC)

println("\nStarting simulation...")
simulate(integrator, 100, 100)
runtime = @elapsed av_steps = simulate(integrator, STEPS-100)
println("\nSolver: $(set.solver)")
println("Total simulation time: $(round(runtime, digits=3)) s")
speed = (STEPS-100) / runtime * dt
println("Simulation speed: $(round(speed, digits=2)) times realtime.")
if PLOT
    local p
    p = plotx(v_time[1:STEPS-100], v_speed[1:STEPS-100], v_force[1:STEPS-100]; ylabels=["v_reelout  [m/s]","tether_force [N]"], fig="winch")
    p = plot(v_time[1:STEPS-100], alpha_2[1:STEPS-100], fig="alpha_2")
    display(p)
end
lift, drag = KiteModels.lift_drag(kps4)
println("lift, drag  [N]: $(round(lift, digits=2)), $(round(drag, digits=2))")
println("Average number of callbacks per time step: $av_steps")

# Ryzen 7950X, Solver: DFBDF
# Total simulation time: 0.043 s
# Simulation speed: 116.99 times realtime.
# lift, drag  [N]: 798.46, 314.82
# Average number of callbacks per time step: 126.41
