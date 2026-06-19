# SPDX-FileCopyrightText: 2022, 2024, 2025 Uwe Fechner
# SPDX-License-Identifier: MIT

using Printf

let
    using KiteModels, KitePodModels, KiteUtils
    kcu = KCU(se())
    kps4 = KPS4(kcu)
    dt = 0.05
    STATISTIC = false
    FRONT_VIEW = false
    ZOOM = false
    PLOT = true

    if PLOT
        using MakieControlPlots
    end        

    function simulate(integrator, steps, plot=false)
        start = integrator.p.iter
        for i in 1:steps  
            next_step!(kps4, integrator; set_speed=0, dt=dt)      
            if plot
                reltime = i*dt
                if mod(i, 5) == 0
                    plot2d(kps4.pos, reltime; zoom=ZOOM, front=FRONT_VIEW, segments=se().segments)                       
                end
            end
        end
        (integrator.p.iter - start) / steps
    end
    integrator = KiteModels.init!(kps4, prn=STATISTIC)
    kps4.stiffness_factor = 0.04
    simulate(integrator, 100, true)
end

using Pkg
if "KiteViewers" ∈ keys(Pkg.project().dependencies)
    let
        using Pkg
        using KiteModels, KitePodModels, KiteUtils, Rotations, StaticArrays
        using KiteViewers  

        set = deepcopy(se())

        # the following values can be changed to match your interest
        dt = 0.05
        set.solver="DFBDF"              # IDA or DFBDF
        set.linear_solver="GMRES"       # GMRES, LapackDense or Dense
        STEPS = 352
        PRINT = false
        STATISTIC = false
        PLOT=false
        UPWIND_DIR2       = -pi/2+deg2rad(10)     # Zero is at north; clockwise positive
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

        function simulate(integrator, steps, plot=PLOT)
            iter = 0
            for i in 1:steps
                acc = 0.0
                v_time[i] = kps4.t_0
                v_speed[i] = kps4.v_reel_out
                v_force[i] = winch_force(kps4)
                heading[i] = rad2deg(wrap2pi(calc_heading(kps4)))
                set_speed = kps4.sync_speed+acc*dt
                if PRINT
                    lift, drag = KiteModels.lift_drag(kps4)
                    @printf "%.2f: " round(integrator.t, digits=2)
                    println("lift, drag  [N]: $(round(lift, digits=2)), $(round(drag, digits=2))")
                end
                steering = 0
                if i > 200
                    steering = -0.0482
                end
                set_depower_steering(kps4.kcu, kps4.depower, steering)

                next_step!(kps4, integrator; set_speed, dt, upwind_dir=UPWIND_DIR2)
                iter += kps4.iter
                reltime = i*dt-dt
                if mod(i, 5) == 1
                    sleep(0.05)           
                end
                sys_state = SysState(kps4)
                KiteViewers.update_system(viewer, sys_state; scale = 0.08, kite_scale=3)
            end
            iter / steps
        end
        
        integrator = KiteModels.init!(kps4, delta=0.001, stiffness_factor=0.1, prn=STATISTIC)
        
        println("\nStarting simulation...")
        simulate(integrator, STEPS)

        # print heading
        println("heading: $(round(heading[STEPS], digits=2))°")
        plot(v_time, heading; xlabel="time [s]", ylabel="heading [°]", fig="heading")
    end
end

GC.gc(true)
let mem = Sys.free_memory() / 1024^2
    @info "Free memory: $(round(mem; digits=1)) MB"
end

@info "Precompile script has completed execution."