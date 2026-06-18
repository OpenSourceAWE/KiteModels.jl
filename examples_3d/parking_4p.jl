# Copyright (c) 2024 Uwe Fechner
# SPDX-License-Identifier: MIT

# Simulate a parking maneuver for the four-point kite model (KPS4).
# After 10s, the kite is steered to the side, demonstrating basic lateral control. 
# Visualized in 3D.

using Printf

using Pkg
if ! ("KiteViewers" ∈ keys(Pkg.project().dependencies))
    Pkg.activate(@__DIR__)
end
using Timers;
tic()
using KiteModels, KitePodModels, Rotations, StaticArrays
using KiteViewers
using MakieControlPlots: plot, plot2d, plotx
toc()

set::Settings = deepcopy(se())

# the following values can be changed to match your interest
dt::Float64 = 0.05
set.solver="DFBDF"              # IDA or DFBDF
set.linear_solver="GMRES"       # GMRES, LapackDense or Dense
default_turbulence = get_default_turbulence()
if default_turbulence !== nothing
    set.use_turbulence = default_turbulence
end
STEPS = 352
PRINT = false
STATISTIC = false
PLOT = false
UPWIND_DIR2 = -pi/2+deg2rad(10)     # Zero is at north; clockwise positive
ZOOM = true
FRONT_VIEW = true
SHOW_KITE = true
# end of user parameter section #

kcu::KCU = KCU(set)
kps4::KPS4 = KPS4(kcu)
viewer::Viewer3D = Viewer3D(SHOW_KITE)

v_time = zeros(STEPS)
v_speed = zeros(STEPS)
v_force = zeros(STEPS)
heading = zeros(STEPS)
heading_rate = zeros(STEPS)
body_rate = zeros(STEPS)

function simulate(integrator, steps, plot = PLOT)
    iter = 0
    sys_state = SysState(kps4)
    for i = 1:steps
        acc = 0.0
        v_time[i] = kps4.t_0
        v_speed[i] = kps4.v_reel_out
        v_force[i] = winch_force(kps4)
        heading[i] = rad2deg(wrap2pi(calc_heading(kps4)))
        set_speed = kps4.sync_speed+acc*dt
        if PRINT
            lift, drag = KiteModels.lift_drag(kps4)
            @printf "%.2f: " round(integrator.t, digits = 2)
            println("lift, drag  [N]: $(round(lift, digits=2)), $(round(drag, digits=2))")
        end
        steering = 0
        if i > 200
            steering = -0.0482
        end
        set_depower_steering(kps4.kcu, kps4.depower, steering)

        next_step!(kps4, integrator; set_speed, dt, upwind_dir = UPWIND_DIR2)
        iter += kps4.iter
        reltime = i*dt-dt
        if mod(i, 5) == 1
            if plot
                plot2d(
                    kps4.pos,
                    reltime;
                    zoom = true,
                    front = FRONT_VIEW,
                    segments = set.segments,
                    fig = "front_view",
                )
            end
            sleep(0.05)
        end
        update_sys_state!(sys_state, kps4)
        heading_rate[i] = sys_state.heading_rate
        body_rate[i] = sys_state.turn_rates[3]
        KiteViewers.update_system(viewer, sys_state; scale = 0.08, kite_scale = 3)
        if i == 1
            bring_viewer_to_front()
        end
    end
    iter / steps
end
toc()

integrator = KiteModels.init!(kps4, delta = 0.001, stiffness_factor = 0.1, prn = STATISTIC)
toc()

println("\nStarting simulation...")
simulate(integrator, STEPS)
if PLOT
    p = plotx(
        v_time[1:(STEPS-100)],
        v_speed[1:(STEPS-100)],
        v_force[1:(STEPS-100)];
        ylabels = ["v_reelout  [m/s]", "tether_force [N]"],
        fig = "winch",
    )
    display(p)
end
# lift, drag = KiteModels.lift_drag(kps4)
# println("lift, drag  [N]: $(round(lift, digits=2)), $(round(drag, digits=2))")

# println("v_wind: $(kps4.v_wind)")
# println("UPWIND_DIR2: $(rad2deg(UPWIND_DIR2))°")
# pos = pos_kite(kps4)
# println("pos_y: $(round(pos[2], digits=2))")
# # for an  UPWIND_DIR2 of -80°, pos_y must be negative, also v_wind[2] must be negative
# # this is OK

# print heading
println("heading: $(round(heading[STEPS], digits=2))°")
p=plot(v_time, heading; xlabel = "time [s]", ylabel = "heading [°]", fig = "heading")
display(p)
p2 = plot(v_time, [rad2deg.(heading_rate), rad2deg.(body_rate)];
           xlabel = "time [s]", ylabel = "rate [°/s]",
           labels = ["heading_rate", "body_rate"], fig = "rates")
display(p2)
if Sys.isapple()
    KiteModels.reactivate_host_app()
end
