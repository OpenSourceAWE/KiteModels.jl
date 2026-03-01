# Copyright (c) 2022, 2024 Uwe Fechner
# SPDX-License-Identifier: MIT

using Printf
using KiteModels
using KiteUtils: Settings, load_settings

set::Settings = deepcopy(load_settings("system.yaml"))

# the following values can be changed to match your interest
dt::Float64 = 0.05
set.solver="DFBDF" # IDA or DFBDF
STEPS = 500
const PLOT = true
FRONT_VIEW = false
ZOOM = false
PRINT = false
STATISTIC = false
# end of user parameter section #

kcu::KCU = KCU(set)
kps3::KPS3 = KPS3(kcu)

if PLOT
    using Pkg
    if ! ("ControlPlots" ∈ keys(Pkg.project().dependencies))
        Pkg.activate("examples")
    end
    using ControlPlots
end

v_time = zeros(STEPS)
v_speed = zeros(STEPS)
v_force = zeros(STEPS)

function simulate(integrator, steps, plot=false)
    iter = 0
    last_label_y = 5.0
    lines, sc, txt = nothing, nothing, nothing
    for i in 1:steps
        if PRINT
            lift, drag = KiteModels.lift_drag(kps3)
            @printf "%.2f: " round(integrator.t, digits=2)
            println("lift, drag  [N]: $(round(lift, digits=2)), $(round(drag, digits=2))")
        end
        acc = 0.0
        if kps3.t_0 > 15.0
            acc = 0.1
        end
        set_speed = kps3.sync_speed+acc*dt
        v_time[i] = kps3.t_0
        v_speed[i] = kps3.v_reel_out
        v_force[i] = winch_force(kps3)
        next_step!(kps3, integrator; set_speed, dt)
        iter += kps3.iter
        if plot 
            reltime = i*dt-dt
            if mod(i, 5) == 1
                z_kite = kps3.pos[end][3]
                z_max = maximum(pos[3] for pos in kps3.pos)
                y_label = last_label_y
                if isfinite(z_kite) && isfinite(z_max)
                    y_axis_1 = 0.0
                    y_axis_2 = z_max + 5.0
                    y_low = min(y_axis_1, y_axis_2) + 0.5
                    y_high = max(y_axis_1, y_axis_2) - 0.5
                    y_label = clamp(z_kite - 14.0, y_low, y_high)
                    last_label_y = y_label
                end
                lines, sc, txt = plot2d(kps3.pos, reltime; zoom=ZOOM, front=FRONT_VIEW, 
                                        segments=set.segments, fig="side_view", xlim=(0,120), dx=1.0, xy=(96.0, y_label))
                if !isnothing(txt)
                    txt.set_x(96.0)
                    txt.set_y(y_label)
                    txt.set_visible(true)
                    try
                        txt.set_annotation_clip(false)
                    catch
                    end
                end
            end
        end
    end
    iter / steps
end

integrator = KiteModels.init!(kps3, delta=0.002, stiffness_factor=0.1, prn=STATISTIC)
kps3.sync_speed = 0.0

av_steps = if PLOT
    simulate(integrator, STEPS, true)
else
    println("\nStarting simulation...")
    simulate(integrator, 100)
    runtime = @elapsed av_steps = simulate(integrator, STEPS-100)
    println("\nTotal simulation time: $(round(runtime, digits=3)) s")
    speed = (STEPS-100) / runtime * dt
    println("Simulation speed: $(round(speed, digits=2)) times realtime.")
    av_steps
end
if Sys.isapple()
    plt.show(block = true)
end
lift, drag = KiteModels.lift_drag(kps3)
println("lift, drag  [N]: $(round(lift, digits=2)), $(round(drag, digits=2))")
println("Average number of callbacks per time step: $(round(av_steps, digits=2))")

p1 = plotx(v_time, v_speed, v_force; ylabels=["v_reelout  [m/s]", "tether_force [N]"], fig="winch")
display(p1)
reactivate_host_app()
# savefig("docs/src/reelout_force_1p.png")
