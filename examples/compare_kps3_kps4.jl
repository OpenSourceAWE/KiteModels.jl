# Copyright (c) 2022, 2024 Uwe Fechner
# SPDX-License-Identifier: MIT
#
# Compares the KPS3 (one-point) and KPS4 (four-point) kite models
# side by side. Runs the same simulation with both models and
# prints lift, drag, winch force, and solver statistics.

using Printf
using KiteModels: KPS3, KPS4, init!, lift_drag, next_step!, reactivate_host_app, winch_force
using KitePodModels: KCU
using KiteUtils: Settings, load_settings

set::Settings = deepcopy(load_settings("system.yaml"))
set.abs_tol=0.00006
set.rel_tol=0.000001

# the following values can be changed to match your interest
dt::Float64 = 0.05
ALPHA_ZERO = 8.8            # for KPS4
STEPS = round(0.5*600/dt*0.05)
const PLOT = true
PRINT = false
STATISTIC = false
ZOOM = 1.0
FRONT_VIEW = true
# end of user parameter section #

if PLOT
    using Pkg
    if dirname(Pkg.project().path) != @__DIR__
        Pkg.activate(@__DIR__)
    end
    using MakieControlPlots
end

set.version = 2

kcu::KCU = KCU(set)
kps4::KPS4 = KPS4(kcu)
kps3::KPS3 = KPS3(kcu)

function simulate(s, integrator, steps, plot = false; fig = "")
    iter = 0
    if plot
        z_text = maximum(p[3] for p in s.pos) + 1.5
        plot2d(
            s.pos,
            0.0;
            zoom = ZOOM,
            xlim = (35, 55),
            xy = (52.0, z_text),
            front = FRONT_VIEW,
            fig,
        )
        sleep(0.025)
    end
    for i in 1:steps
        if PRINT
            lift, drag = lift_drag(s)
            @printf "%.2f: " round(integrator.t, digits = 2)
            println("lift, drag  [N]: $(round(lift, digits = 2)), $(round(drag, digits = 2))")
        end

        next_step!(s, integrator; set_speed = 0, dt = dt)
        iter += s.iter

        if plot
            reltime = i * dt
            if mod(i, 5) == 1
                z_text = maximum(p[3] for p in s.pos) + 1.5
                plot2d(
                    s.pos,
                    reltime;
                    zoom = ZOOM,
                    xlim = (35, 75),
                    xy = (52.0, z_text),
                    front = FRONT_VIEW,
                    fig,
                )
                sleep(0.025)
            end
        end
    end
    iter / steps
end

integrator = init!(kps3, delta = 0.001, stiffness_factor = 0.1, prn = STATISTIC)
av_steps = simulate(kps3, integrator, STEPS, true; fig = "kps3")
reactivate_host_app()

lift, drag = lift_drag(kps3)
println("KPS3")
println("lift, drag  [N]: $(round(lift, digits=2)), $(round(drag, digits=2))")
println("winch_force [N]: $(round(winch_force(kps3), digits=2))")
println("Average number of callbacks per time step: $(round(av_steps, digits=2))")

kps4.set.alpha_zero = ALPHA_ZERO
integrator = init!(kps4; delta = 0.003, stiffness_factor = 0.1, prn = STATISTIC)
av_steps = simulate(kps4, integrator, STEPS, true; fig = "kps4")
reactivate_host_app()

lift, drag = lift_drag(kps4)
println("KPS4")
println("lift, drag  [N]: $(round(lift, digits=2)), $(round(drag, digits=2))")
println("winch_force [N]: $(round(winch_force(kps4), digits=2))")
println("Average number of callbacks per time step: $(round(av_steps, digits=2))")

# # KPS3
# # lift, drag  [N]: 730.25, 157.31
# # winch_force [N]: 594.89
# # Average number of callbacks per time step: 40.07
# # KPS4
# # lift, drag  [N]: 734.04, 140.91
# # winch_force [N]: 597.31
# # Average number of callbacks per time step: 101.77
