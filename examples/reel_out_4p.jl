using Printf
using KiteModels, KitePodModels, KiteUtils

set = deepcopy(se())

# the following values can be changed to match your interest
dt = 0.05
set.solver="DFBDF" # IDA or DFBDF
STEPS = 600
PLOT = true
FRONT_VIEW = false
ZOOM = true
PRINT = false
STATISTIC = false
ALPHA_ZERO = 8.8 
# end of user parameter section #

set.alpha_zero = ALPHA_ZERO
set.version = 2

kcu::KCU = KCU(set)
kps4::KPS4 = KPS4(kcu)

# if PLOT
    using Pkg
    if ! ("ControlPlots" ∈ keys(Pkg.project().dependencies))
        using TestEnv; TestEnv.activate()
    end
    pkg"add ControlPlots#main"
    using ControlPlots
# end

v_time = zeros(STEPS)
v_speed = zeros(STEPS)
v_force = zeros(STEPS)

function simulate(integrator, steps, plot=false)
    start = integrator.p.iter
    for i in 1:steps
        if PRINT
            lift, drag = KiteModels.lift_drag(kps4)
            @printf "%.2f: " round(integrator.t, digits=2)
            println("lift, drag  [N]: $(round(lift, digits=2)), $(round(drag, digits=2))")
        end
        acc = 0.0
        if kps4.t_0 > 15.0
            acc = 0.1
        end
        v_ro = kps4.sync_speed+acc*dt
        v_time[i] = kps4.t_0
        v_speed[i] = kps4.v_reel_out
        v_force[i] = winch_force(kps4)
        KiteModels.next_step!(kps4, integrator, v_ro = v_ro, dt=dt)
        
        if plot
            reltime = i*dt-dt
            if mod(i, 5) == 1
                plot2d(kps4.pos, reltime; zoom=ZOOM, front=FRONT_VIEW, 
                                        segments=set.segments, fig="side_view")            
            end
        end
    end
    (integrator.p.iter - start) / steps
end

integrator = KiteModels.init_sim!(kps4, stiffness_factor=0.5, prn=STATISTIC)
kps4.sync_speed = 0.0

if PLOT
    av_steps = simulate(integrator, STEPS, true)
else
    println("\nStarting simulation...")
    simulate(integrator, 100)
    runtime = @elapsed av_steps = simulate(integrator, STEPS-100)
    println("\nSolver: $(set.solver)")
    println("Total simulation time: $(round(runtime, digits=3)) s")
    speed = (STEPS-100) / runtime * dt
    println("Simulation speed: $(round(speed, digits=2)) times realtime.")
end
lift, drag = KiteModels.lift_drag(kps4)
println("lift, drag  [N]: $(round(lift, digits=2)), $(round(drag, digits=2))")
println("Average number of callbacks per time step: $av_steps")

if PLOT
    p = plotx(v_time, v_speed, v_force; ylabels=["v_reelout  [m/s]","tether_force [N]"], fig="winch")
    display(p)
end
# savefig("docs/src/reelout_force_4p.png")

# Solver: DFBDF, reltol=0.000001
# Total simulation time: 1.049 s
# Simulation speed: 23.83 times realtime.
# lift, drag  [N]: 545.24, 102.55
# Average number of callbacks per time step: 625.604

# Solver: DFBDF, reltol=0.001
# Total simulation time: 0.178 s
# Simulation speed: 140.08 times realtime.
# lift, drag  [N]: 558.69, 105.3
# Average number of callbacks per time step: 114.64

# Solver: IDA
# Total simulation time: 1.385 s
# Simulation speed: 18.05 times realtime.
# lift, drag  [N]: 543.58, 102.19
# Average number of callbacks per time step: 756.074
