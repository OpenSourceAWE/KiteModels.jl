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
        using ControlPlots
        function plot2d(pos, reltime=0.0; zoom=true, front=false, segments=6, fig="", lines, sc, txt)
            x = Float64[] 
            z = Float64[]
            for i in eachindex(pos)
                if front
                    push!(x, pos[i][2])
                else
                    push!(x, pos[i][1])
                end
                push!(z, pos[i][3])
            end
            x_max = maximum(x)
            z_max = maximum(z)
            xlabel = "x [m]"
            if front xlabel = "y [m]" end
            if isnothing(lines)
                if fig != ""
                    plt.figure(fig)
                end
                lines=[]
                line, = plt.plot(x,z; linewidth="1")
                push!(lines, line)
                sc  = plt.scatter(x, z; s=25, color="red") 
                if zoom
                    txt = plt.annotate("t=$(round(reltime,digits=1)) s",  
                        xy=(x_max, z_max+4.7), fontsize = 14)
                    plt.xlim(x_max-15.0, x_max+20)
                    plt.ylim(z_max-15.0, z_max+8)
                else
                    txt = plt.annotate("t=$(round(reltime,digits=1)) s",  
                    xy=(x_max, z_max+8.0), fontsize = 14)
                    plt.xlim(0, x_max+20)
                    plt.ylim(0, z_max+20)
                end
                if length(pos) > segments+1
                    s=segments
                    line, = plt.plot([x[s+1],x[s+4]],[z[s+1],z[s+4]], linewidth="1"); push!(lines, line) # S6
                    line, = plt.plot([x[s+2],x[s+5]],[z[s+2],z[s+5]], linewidth="1"); push!(lines, line) # S8
                    line, = plt.plot([x[s+3],x[s+5]],[z[s+3],z[s+5]], linewidth="1"); push!(lines, line) # S7
                    line, = plt.plot([x[s+2],x[s+4]],[z[s+2],z[s+4]], linewidth="1"); push!(lines, line) # S2
                    line, = plt.plot([x[s+1],x[s+5]],[z[s+1],z[s+5]], linewidth="1"); push!(lines, line) # S5
                end
                plt.xlabel(xlabel, fontsize=14)
                plt.ylabel("z [m]", fontsize=14)
                plt.grid(true)
                plt.grid(which="major", color="#DDDDDD")
            else
                lines[1].set_xdata(x)
                lines[1].set_ydata(z)
                if length(pos) > segments+1
                    s=segments
                    lines[2].set_xdata([x[s+1],x[s+4]]) # S6
                    lines[2].set_ydata([z[s+1],z[s+4]]) # S6
                    lines[3].set_xdata([x[s+2],x[s+5]]) # S8
                    lines[3].set_ydata([z[s+2],z[s+5]]) # S8
                    lines[4].set_xdata([x[s+3],x[s+5]]) # S7
                    lines[4].set_ydata([z[s+3],z[s+5]]) # S7
                    lines[5].set_xdata([x[s+2],x[s+4]]) # S2
                    lines[5].set_ydata([z[s+2],z[s+4]]) # S2
                    lines[6].set_xdata([x[s+1],x[s+5]]) # S5
                    lines[6].set_ydata([z[s+1],z[s+5]]) # S5
                end
                sc.set_offsets(hcat(x,z))
                txt.set_text("t=$(round(reltime,digits=1)) s")
                plt.gcf().canvas.draw()
            end
            sleep(0.01)
            lines, sc, txt
        end
    end        

    function simulate(integrator, steps, plot=false)
        start = integrator.p.iter
        lines, sc, txt = nothing, nothing, nothing
        for i in 1:steps  
            KiteModels.next_step!(kps4, integrator; set_speed=0, dt=dt)      
            if plot
                reltime = i*dt
                if mod(i, 5) == 0
                    lines, sc, txt = plot2d(kps4.pos, reltime; zoom=ZOOM, front=FRONT_VIEW, segments=se().segments, lines, sc, txt)                       
                end
            end
        end
        (integrator.p.iter - start) / steps
    end
    integrator = KiteModels.init_sim!(kps4, prn=STATISTIC)
    kps4.stiffness_factor = 0.04
    simulate(integrator, 100, true)
end
let 
    using KiteModels, OrdinaryDiffEqCore, OrdinaryDiffEqBDF, OrdinaryDiffEqSDIRK, LinearAlgebra
    using Base: summarysize

    update_settings()
    set = se("system_3l.yaml")
    set.abs_tol = 0.006
    set.rel_tol = 0.01
    steps = 150
    dt = 1/set.sample_freq
    tspan   = (0.0, dt)

    logger = Logger(3*set.segments + 6, steps)

    steering = [5,5,-30.0]

    println("Running models")
    if ! @isdefined mtk_kite; mtk_kite = KPS4_3L(KCU(set)); end
    mtk_integrator = KiteModels.init_sim!(mtk_kite; prn=false, torque_control=true)

    println("compiling")
    total_new_time = 0.0
    for i in 1:5
        total_new_time += @elapsed next_step!(mtk_kite, mtk_integrator; set_values=steering)
    end
    sys_state = KiteModels.SysState(mtk_kite)
    if sys_state.heading > pi
        sys_state.heading -= 2*pi
    end
    log!(logger, sys_state)

    println("stepping")
    total_old_time = 0.0
    total_new_time = 0.0
end

@info "Precompile script has completed execution."