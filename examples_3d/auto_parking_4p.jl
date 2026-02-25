# SPDX-FileCopyrightText: 2024 Uwe Fechner
# SPDX-License-Identifier: MIT

# activate the examples_3d environment if needed
using Pkg
if ! ("KiteViewers" ∈ keys(Pkg.project().dependencies))
    Pkg.activate(joinpath(@__DIR__))
end
using Timers; tic()

using KiteViewers, KiteModels, ControlPlots, Rotations
using KiteUtils: Settings, load_settings

set::Settings = deepcopy(load_settings("system.yaml"))
set.abs_tol=0.00006
set.rel_tol=0.0001
set.v_wind = 10 # v_min1 6-25; v_min2 5.3-30

include("parking_controller.jl")
import .ParkingControllers as pcm
pcs = pcm.ParkingControllerSettings(dt=0.05)

kcu::KCU = KCU(set)
kps4::KPS4 = KPS4(kcu)
@assert set.sample_freq == 20
dt::Float64 = 1/set.sample_freq

MIN_DEPOWER, DISTURBANCE = if KiteUtils.PROJECT == "system.yaml"
    # result of tuning
    pcs.kp_tr = 0.07
    pcs.ki_tr = 0.0012
    pcs.kp = 15
    pcs.ki = 0
    pcs.c1 = 0.149
    pcs.c2 = 0 # has no big effect, can also be set to zero c1 = 0.149 c2 = 5.428
    0.22, 0.1
else 
    # result of tuning
    println("not system.yaml")
    pcs.kp_tr = 0.05
    pcs.ki_tr = 0.0024
    pcs.kp = 30
    pcs.ki = 1.0
    pcs.c1 = 0.048
    pcs.c2 = 0    # has no big effect, can also be set to zero
    0.4, 0.4
end
@info "pcs.kp_tr=$(pcs.kp_tr), pcs.ki_tr=$(pcs.ki_tr), pcs.kp=$(pcs.kp), pcs.ki=$(pcs.ki), MIN_DEPOWER=$(MIN_DEPOWER)"
pc = pcm.ParkingController(pcs)

# the following values can be changed to match your interest
MAX_TIME::Float64 =  60 # was 60
TIME_LAPSE_RATIO  =  6
SHOW_KITE         = true
# end of user parameter section #

viewer::Viewer3D = Viewer3D(SHOW_KITE, "WinchON")

T::Vector{Float64} = zeros(Int64(MAX_TIME/dt))
AZIMUTH::Vector{Float64}       = zeros(Int64(MAX_TIME/dt))
HEADING::Vector{Float64}       = zeros(Int64(MAX_TIME/dt))
SET_STEERING::Vector{Float64}  = zeros(Int64(MAX_TIME/dt))
STEERING::Vector{Float64}      = zeros(Int64(MAX_TIME/dt))
AoA::Vector{Float64}           = zeros(Int64(MAX_TIME/dt))
PSI_DOT::Vector{Float64}       = zeros(Int64(MAX_TIME/dt))
PSI_DOT_SET::Vector{Float64}   = zeros(Int64(MAX_TIME/dt))
NDI_GAIN::Vector{Float64}      = zeros(Int64(MAX_TIME/dt))
V_APP::Vector{Float64}         = zeros(Int64(MAX_TIME/dt))

function simulate(integrator, sys_state)
    start_time_ns = time_ns()
    clear_viewer(viewer)
    i=1; j=0; k=0
    GC.gc()
    max_time = 0
    t_gc_tot = 0
    while true
        steering = 0.0
        if i >= 100
            if i == 100
                pc.last_heading = sys_state.heading
            end
            chi_set = pcm.navigate(pc, sys_state.azimuth, sys_state.elevation)
            steering, ndi_gain, psi_dot, psi_dot_set = pcm.calc_steering(pc, sys_state.heading, chi_set; 
                                                                         sys_state.elevation, v_app = sys_state.v_app)
            PSI_DOT[i] = psi_dot
            PSI_DOT_SET[i] = psi_dot_set
            NDI_GAIN[i] = ndi_gain
            V_APP[i] = sys_state.v_app
            time = i * dt
            # disturbance
            if time > 20 && time < 21
                steering = DISTURBANCE
            end            
            set_depower_steering(kps4.kcu, MIN_DEPOWER, steering)
        end
        SET_STEERING[i] = steering
        STEERING[i] = get_steering(kps4.kcu)/set.cs_4p
        AoA[i] = kps4.alpha_2
        # execute winch controller
        v_ro = 0.0
        t_sim = @elapsed KiteModels.next_step!(kps4, integrator; set_speed=v_ro, dt=dt)
        if t_sim < 0.3*dt
            t_gc_tot += @elapsed GC.gc(false)
        end
        sys_state = SysState(kps4)
        T[i] = dt * i
        AZIMUTH[i] = sys_state.azimuth
        HEADING[i] = wrap2pi(sys_state.heading)
        if mod(i, TIME_LAPSE_RATIO) == 0
            if KiteUtils.PROJECT == "system.yaml"
                KiteViewers.update_system(viewer, sys_state; scale = 0.08, kite_scale=3)
            else
                KiteViewers.update_system(viewer, sys_state; scale = 0.08*0.5, kite_scale=3)
            end
            wait_until(start_time_ns + 1e9*dt, always_sleep=true) 
            mtime = 0
            if i > 10/dt 
                # if we missed the deadline by more than 5 ms
                mtime = time_ns() - start_time_ns
                if mtime > dt*1e9 + 5e6
                    print(".")
                    j += 1
                end
                k +=1
            end
            if mtime > max_time
                max_time = mtime
            end            
            start_time_ns = time_ns()
            t_gc_tot = 0
        end
        if ! isopen(viewer.fig.scene) break end
        if i*dt >= MAX_TIME break end
        i += 1
    end
    misses = j/k * 100
    println("\nMissed the deadline for $(round(misses, digits=2)) %. Max time: $(round((max_time*1e-6), digits=1)) ms")
    return div(i, TIME_LAPSE_RATIO)
end

function play()
    integrator = KiteModels.init!(kps4; delta=0.001, stiffness_factor=0.1)
    sys_state = SysState(kps4)
    toc()
    simulate(integrator, sys_state)
    GC.enable(true)
end

function play1()
    if viewer.stop
        play()
        stop(viewer)
    end
end

on(viewer.btn_PLAY.clicks) do _; play1(); end

play()
stop(viewer)
p = plotx(T, rad2deg.(AZIMUTH), rad2deg.(HEADING), [100*(SET_STEERING), 100*(STEERING)],
             [rad2deg.(PSI_DOT), rad2deg.(PSI_DOT_SET)], NDI_GAIN, V_APP; 
          xlabel="Time [s]", 
          ylabels=["Azimuth [°]", "Heading [°]", "steering [%]", "psi_dot [°/s]", "NDI_GAIN", "v_app [m/s]"],   
          labels=["azimuth", "heading", ["set_steering", "steering"], ["psi_dot", "psi_dot_set"], "NDI_GAIN", "v_app"],  
          fig="Azimuth, heading, steering and more")
display(p)
