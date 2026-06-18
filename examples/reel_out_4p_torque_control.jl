# Copyright (c) 2024 Uwe Fechner
# SPDX-License-Identifier: MIT
#
# Simulates the reel-out phase using the KPS4 kite model with a
# torque-controlled winch. Configurable torque, solver settings, and
# optional 2D plotting of the kite trajectory and winch force.

using KiteModels, LinearAlgebra
using KiteUtils: Settings, load_settings
using Printf

set::Settings = deepcopy(load_settings("system.yaml"))

# the following values can be changed to match your interest
dt::Float64 = 0.05/3
set.solver="DFBDF" # IDA or DFBDF
STEPS = 600*3
const PLOT = true
PRINT = false
STATISTIC = false
ALPHA_ZERO = 8.8 
# end of user parameter section #

set.alpha_zero = ALPHA_ZERO
set.version = 2
set.winch_model = "TorqueControlledMachine"

kcu::KCU = KCU(set)
kps4::KPS4 = KPS4(kcu)

if PLOT
    using Pkg
    if dirname(Pkg.project().path) != @__DIR__
        Pkg.activate(@__DIR__)
    end
    using MakieControlPlots
end

v_time = zeros(STEPS)
v_speed = zeros(STEPS)
v_force = zeros(STEPS)
v_elevation = zeros(STEPS)
v_heading = zeros(STEPS)
v_wind_kite = zeros(STEPS)

function simulate(integrator, steps, plot=false)
    iter = 0
    for i in 1:steps
        if PRINT
            lift, drag = KiteModels.lift_drag(kps4)
            @printf "%.2f: " round(integrator.t, digits=2)
            println("lift, drag  [N]: $(round(lift, digits=2)), $(round(drag, digits=2))")
        end
        dforce = 0.0
        if kps4.t_0 > 15.0
            dforce = +4.5
        end
        force = winch_force(kps4)
        r = set.drum_radius
        n = set.gear_ratio
        set_torque = -r/n * force + dforce
        v_time[i] = kps4.t_0
        v_speed[i] = KiteModels.reel_out_speed(kps4)
        v_force[i] = force
        v_elevation[i] = rad2deg(KiteModels.calc_elevation(kps4))
        v_heading[i] = rad2deg(KiteModels.calc_heading(kps4; respos=false))
        v_wind_kite[i] = norm(KiteModels.v_wind_kite(kps4))
        next_step!(kps4, integrator; set_torque, dt)
        iter += kps4.iter
        
        if plot
            reltime = i*dt-dt
            if mod(i, 5) == 1
                if FRONT_VIEW
                    plot2d(kps4.pos, reltime; zoom=ZOOM, front=true,
                                            segments=set.segments, fig="front_view")
                else
                    plot2d(kps4.pos, reltime; zoom=ZOOM, front=false, xlim=(37, 78),
                                            segments=set.segments, fig="side_view")
                end
                sleep(0.025)
            end
        end
    end
    iter / steps
end

integrator = KiteModels.init!(kps4; delta=0.003, stiffness_factor=0.1, prn=STATISTIC)
kps4.sync_speed = 0.0

av_steps = if PLOT
    simulate(integrator, STEPS, true)
else
    println("\nStarting simulation...")
    simulate(integrator, 100)
    runtime = @elapsed av_steps = simulate(integrator, STEPS-100)
    println("\nSolver: $(set.solver)")
    println("Total simulation time: $(round(runtime, digits=3)) s")
    speed = (STEPS-100) / runtime * dt
    println("Simulation speed: $(round(speed, digits=2)) times realtime.")
    av_steps
end
lift, drag = KiteModels.lift_drag(kps4)
println("lift, drag  [N]: $(round(lift, digits=2)), $(round(drag, digits=2))")
println("Average number of callbacks per time step: $(round(av_steps, digits=2))")

if PLOT
    local p
    p = plotx(v_time, v_speed, v_force, v_elevation, v_heading, v_wind_kite; 
    ylabels=["v_reelout  [m/s]","tether_force [N]","elevation [deg]","heading [deg]","wind at kite [m/s]"], fig="winch")
    display(p)
    reactivate_host_app()
end
println("Number of states: $(states(kps4))")
println("Kite mass: $(set.mass) kg")
println("KCU mass:  $(set.kcu_mass) kg")
println("Tether diameter: $(set.d_tether) mm")
# savefig("docs/src/reelout_force_4p.png")

# Solver: DFBDF, reltol=0.000001
# Total simulation time: 1.049 s
# Simulation speed: 23.83 times realtime.
# lift, drag  [N]: 545.24, 102.55
# Average number of callbacks per time step: 625.604

# Solver: DFBDF, reltol=0.001
# Total simulation time: 0.178 s
# Simulation speed: 198.04 times realtime.
# lift, drag  [N]: 633.13, 119.82
# Average number of callbacks per time step: 75.67

# Solver: IDA
# Total simulation time: 0.662 s
# Simulation speed: 37.78 times realtime.
# lift, drag  [N]: 639.63, 121.71
# Average number of callbacks per time step: 379.91
