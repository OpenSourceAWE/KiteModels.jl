# SPDX-FileCopyrightText: 2024 Uwe Fechner
# SPDX-License-Identifier: MIT

# Park the kite while the wind direction changes.
#
# After 16 s simulation time, the system state is written to
# output/sys_state_parking_wind_dir.yaml, with the script name, the simulation time and
# the date and time of saving as meta data. If LOAD_SYS_STATE is true, the simulation
# continues from that file instead of starting at zero, and the state is not saved.

using Pkg
if dirname(Pkg.project().path) != @__DIR__
    Pkg.activate(@__DIR__)
end
using Timers; tic()

using KiteViewers
using KiteModels, LinearAlgebra, MakieControlPlots, Rotations, Statistics

set::Settings = if haskey(ENV, "USE_V9")
    deepcopy(load_settings("system_v9.yaml"))
else
    deepcopy(load_settings("system.yaml"))
end
set.abs_tol=0.00006
set.rel_tol=0.0001
set.sample_freq = 20
default_turbulence = get_default_turbulence()
if default_turbulence !== nothing
    set.use_turbulence = default_turbulence
end

include("parking_controller.jl")
import .ParkingControllers as pcm
pcs = pcm.ParkingControllerSettings(dt=0.05)

kcu::KCU = KCU(set)
kps4::KPS4 = KPS4(kcu)
@assert set.sample_freq == 20
dt::Float64 = 1/set.sample_freq


MIN_DEPOWER = if KiteUtils.PROJECT == "system.yaml"
    # result of tuning
    pcs.kp_tr=0.15
    pcs.ki_tr=0.003
    pcs.kp = 1.0
    pcs.ki = 0.025
    pcs.kd = 2.5
    pcs.kd_N = 2
    pcs.c1 = 0.048
    pcs.c2 = 0 # has no big effect, can also be set to zero
    pcs.max_turn_rate_set = 0.20
    pcs.max_turn_rate_cmd = 0.50
    pcs.max_steering = 0.45
    pcs.max_steering_rate = 1.0
    0.22
else
    # result of tuning
    println("not system.yaml")
    pcs.kp_tr=0.035
    pcs.ki_tr=0.0015
    pcs.kp = 1.0
    pcs.ki = 0.025
    pcs.kd = 2.5
    pcs.kd_N = 2
    pcs.c1 = 0.048
    pcs.c2 = 0    # has no big effect, can also be set to zero
    pcs.max_turn_rate_set = 0.20
    pcs.max_turn_rate_cmd = 0.50
    pcs.max_steering = 0.45
    pcs.max_steering_rate = 1.0
    0.4
end
pc = pcm.ParkingController(pcs)

# the following values can be changed to match your interest
MAX_TIME::Float64 = 120
TIME_LAPSE_RATIO  =  6
SHOW_KITE         = true
# For position and velocity vectors of the model the ENU (East North Up) 
UPWIND_DIR        = -pi/2 # the direction the wind is coming from.
UPWIND_DIR2       = -pi/2+deg2rad(90)     # Zero is at north; clockwise positive
INTERPOLATE       = true  # interpolate the turbulence between the grid points of the wind field
SAVE_TIME         = 16    # simulation time [s] after which the system state is saved
PLOT_HEADING_RATE = false
LOAD_SYS_STATE    = true  # continue from the state saved at SAVE_TIME instead of starting at zero
# end of user parameter section #

viewer::Viewer3D = Viewer3D(SHOW_KITE, "WinchON")

steps = 0
T::Vector{Float64}             = zeros(Int64(MAX_TIME/dt))
AZIMUTH::Vector{Float64}       = zeros(Int64(MAX_TIME/dt))
AZIMUTH_EAST::Vector{Float64}  = zeros(Int64(MAX_TIME/dt))
UPWIND_DIR_::Vector{Float64}   = zeros(Int64(MAX_TIME/dt))
AV_UPWIND_DIR::Vector{Float64} = zeros(Int64(MAX_TIME/dt))
V_WIND_KITE::Vector{Float64}   = zeros(Int64(MAX_TIME/dt))
HEADING::Vector{Float64}       = zeros(Int64(MAX_TIME/dt))
SET_STEERING::Vector{Float64}  = zeros(Int64(MAX_TIME/dt))
STEERING::Vector{Float64}      = zeros(Int64(MAX_TIME/dt))
HEADING_RATE::Vector{Float64}  = zeros(Int64(MAX_TIME/dt))
BODY_RATE::Vector{Float64}     = zeros(Int64(MAX_TIME/dt))

SCRIPT_NAME = splitext(basename(@__FILE__))[1]
STATE_FILE  = joinpath(dirname(@__DIR__), "output", "sys_state_$(SCRIPT_NAME).yaml")

# save a copy of sys_state with the time of the state; SysState.time is the start time of the
# last time step
function save_sys_state(sys_state, t_sim, time_)
    sys_state = deepcopy(sys_state)
    sys_state.t_sim = t_sim
    sys_state.time = time_
    mkpath(dirname(STATE_FILE))
    meta_data = Dict("script" => SCRIPT_NAME, "time" => sys_state.time,
                     "date_time" => Libc.strftime("%Y-%m-%d_%H:%M:%S", time()))
    save_state(STATE_FILE, sys_state; meta_data)
end

# initial_state is the loaded SysState, or nothing when starting at zero; the step i
# continues from the time of the integrator
function sim_parking(integrator, initial_state)
    upwind_dir=UPWIND_DIR
    start_time_ns = time_ns()
    clear_viewer(viewer)
    i = round(Int, integrator.t/dt) + 1; j=0; k=0
    GC.gc()
    max_time = 0
    t_gc_tot = 0
    # continuing with the loaded SysState keeps the rates, e.g. heading_rate, correct
    sys_state = isnothing(initial_state) ? SysState(kps4) : deepcopy(initial_state)
    println("Kite position at start: ", round.(pos_kite(kps4), digits=2))
    if KiteUtils.PROJECT == "system.yaml"
        KiteViewers.update_system(viewer, sys_state; scale = 0.08, kite_scale=3)
    else
        KiteViewers.update_system(viewer, sys_state; scale = 0.08*0.5, kite_scale=3)
    end
    bring_viewer_to_front()
    while true
        time = i * dt 
        steering = 0.0
        if i > 100
            elevation = sys_state.elevation
            chi_set = pcm.navigate(pc, sys_state.azimuth, elevation)
            steering, _, _, _ = pcm.calc_steering(pc, sys_state.heading, sys_state.heading_rate, chi_set; 
                                                                         elevation, v_app = sys_state.v_app)
            set_depower_steering(kps4.kcu, MIN_DEPOWER, steering)
        end  
        SET_STEERING[i] = steering
        STEERING[i] = get_steering(kps4.kcu) / set.cs_4p
        # execute winch controller
        v_ro = 0.0
        if time > 20
            upwind_dir += deg2rad(0.04*2)
            if upwind_dir > UPWIND_DIR2
                upwind_dir = UPWIND_DIR2
            end
            UPWIND_DIR_[i] = upwind_dir
            # av_upwind_dir = moving_average(UPWIND_DIR_[1:i], 400)
            av_upwind_dir = upwind_dir
        else
            upwind_dir=UPWIND_DIR
            UPWIND_DIR_[i] = upwind_dir
            av_upwind_dir = upwind_dir
        end
        t_sim = @elapsed KiteModels.next_step!(kps4, integrator; set_speed=v_ro, dt, upwind_dir=av_upwind_dir,
                                               interpolate=INTERPOLATE)
        AV_UPWIND_DIR[i] = av_upwind_dir
        V_WIND_KITE[i] = norm(v_wind_kite(kps4))
        if t_sim < 0.3*dt
            t_gc_tot += @elapsed GC.gc(false)
        end
        update_sys_state!(sys_state, kps4)
        if !LOAD_SYS_STATE && i == round(Int, SAVE_TIME/dt)
            save_sys_state(sys_state, t_sim, integrator.t)
        end
        T[i] = dt * i
        AZIMUTH[i] = sys_state.azimuth
        AZIMUTH_EAST[i] = calc_azimuth_east(kps4)
        HEADING[i] = wrap2pi(sys_state.heading)
        HEADING_RATE[i] = sys_state.heading_rate
        BODY_RATE[i] = fromKA2KS_body(sys_state.turn_rates)[3]
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
    println("Kite position at end:   ", round.(pos_kite(kps4), digits=2))
    println("\nMissed the deadline for $(round(misses, digits=2)) %. Max time: $(round((max_time*1e-6), digits=1)) ms")
    return div(i, TIME_LAPSE_RATIO)
end

# returns the first step of the simulation
function play_parking()
    initial_state = LOAD_SYS_STATE ? first(load_state(STATE_FILE)) : nothing
    saved_turbulence = kps4.set.use_turbulence
    kps4.set.use_turbulence = 0.0
    integrator = KiteModels.init!(kps4; initial_state)
    kps4.set.use_turbulence = saved_turbulence
    i_start = round(Int, integrator.t/dt) + 1
    sys_state = SysState(kps4)
    if KiteUtils.PROJECT == "system.yaml"
        KiteViewers.update_system(viewer, sys_state; scale = 0.08, kite_scale=3)
    else
        KiteViewers.update_system(viewer, sys_state; scale = 0.08*0.5, kite_scale=3)
    end
    toc()
    try
        sim_parking(integrator, initial_state)
    catch e
        if isa(e, AssertionError)
            println("AssertionError! Halting simulation.")
        else
            println("Exception! Halting simulation.")
            throw(e) 
        end
    end
    GC.enable(true)
    i_start
end

i_start = play_parking()
stop(viewer)
# plot only the simulated steps
for vec in (T, AZIMUTH, AZIMUTH_EAST, UPWIND_DIR_, AV_UPWIND_DIR, V_WIND_KITE, HEADING, SET_STEERING,
            STEERING, HEADING_RATE, BODY_RATE)
    deleteat!(vec, 1:i_start-1)
end
v = filter(!=(0.0), V_WIND_KITE)
ti = std(v) / mean(v) * 100
println("Turbulence intensity (wind speed magnitude): $(round(ti, digits=2)) %")
p=plotx(T, rad2deg.(AZIMUTH), rad2deg.(AZIMUTH_EAST),[rad2deg.(UPWIND_DIR_), rad2deg.(AV_UPWIND_DIR)],
         rad2deg.(HEADING), [100*(SET_STEERING), 100*(STEERING)], V_WIND_KITE; 
         xlabel="Time [s]", 
         ylabels=["Azimuth [°]", "azimuth_east [°]", "upwind_dir [°]", "Heading [°]", "Steering [%]", "v_wind_kite [m/s]"],
         labels=["azimuth", "azimuth_east", ["upwind_dir", "filtered_upwind_dir"], "heading", ["set_steering", "steering"], "v_wind_kite"],
         ysize=12, fig="Parking with changing wind direction, TI: $(round(ti, digits=2)) %")
display(p)
if PLOT_HEADING_RATE
    p2 = plot(T, [rad2deg.(HEADING_RATE), rad2deg.(BODY_RATE)];
            xlabel = "time [s]", ylabel = "rate [°/s]",
            labels = ["heading_rate", "body_rate"], fig = "rates")
    display(p2)
end
KiteModels.reactivate_host_app()
