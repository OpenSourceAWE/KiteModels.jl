# Copyright (c) 2022, 2024 Uwe Fechner
# SPDX-License-Identifier: MIT

# Test the initialization of the one-point kite model (KPS3).
# Initializes the model from a steady-state solution, runs it for 80 time steps and prints
# the lift, drag, elevation, orientation, angle of attack and wind speeds of the kite.
#
# After 80 time steps, the system state is written to
# output/sys_state_test_init_1p.yaml, with the script name, the simulation time and
# the date and time of saving as meta data.

using Printf
using Pkg
if dirname(Pkg.project().path) != @__DIR__
    Pkg.activate(@__DIR__)
end
using KiteModels, KitePodModels, LinearAlgebra, Rotations

set::Settings = deepcopy(load_settings("system.yaml"))

set.abs_tol=0.00006
set.rel_tol=0.000001
V_WIND = 14.5

# the following values can be changed to match your interest
dt::Float64 = 0.05
set.solver="DFBDF" # IDA or DFBDF
set.v_reel_out = 1.0 # initial reel-out speed [m/s]
STEPS = 1
STATISTIC = false

elev = set.elevation
set.v_wind = V_WIND # 25

kcu::KCU = KCU(set)
kps3::KPS3 = KPS3(kcu)
logger = Logger(set.segments + 1, STEPS; segments=n_springs(kps3))
integrator = KiteModels.init!(kps3; prn=STATISTIC, trim=true)
t_sim = 0.0 # wall-clock time of the last time step [s]
for _ in 1:80
    global t_sim = @elapsed next_step!(kps3, integrator; set_speed=kps3.set.v_reel_out,
                                       upwind_dir=deg2rad(set.upwind_dir), dt)
end
lift, drag = lift_drag(kps3)
sys_state::SysState = KiteModels.SysState(kps3)
sys_state.t_sim = t_sim
# SysState.time is the start time of the last time step; save the time of the state
sys_state.time = integrator.t

output_dir = joinpath(dirname(@__DIR__), "output")
mkpath(output_dir)
script_name = splitext(basename(@__FILE__))[1]
meta_data = Dict("script" => script_name, "time" => sys_state.time,
                 "date_time" => Libc.strftime("%Y-%m-%d_%H:%M:%S", time()))
save_state(joinpath(output_dir, "sys_state_$(script_name).yaml"), sys_state; meta_data,
           exclude=KiteModels.UNUSED_FIELDS_KPS3)
log!(logger, sys_state)
elev = rad2deg(logger.elevation_vec[end])
println("Lift: $lift, Drag: $drag, elev: $elev, Iterations: $(kps3.iter)")

q = QuatRotation(calc_orient_quat(kps3; viewer=false))
roll, pitch, yaw = rad2deg.(quat2euler(q))
println("--> orient_quat:       roll: ", roll, " pitch: ", pitch, "  yaw: ", yaw)
roll, pitch, yaw = rad2deg.(orient_euler(kps3))
println("--> orient_euler:      roll: ", roll, " pitch: ", pitch, " yaw:  ", yaw)
q = QuatRotation(calc_orient_quat(kps3; viewer=true))
roll, pitch, yaw = rad2deg.(quat2euler(q))
println("--> orient_quat (viewer): roll: ", roll, " pitch: ", pitch, "   yaw: ", yaw)

x, y, z = kite_ref_frame(kps3)
println("x:", x) # from trailing edge to leading edge in ENU reference frame
println("y:", y) # to the right looking in flight direction
println("z:", z) # down
println("v_reel_out: ", round(kps3.v_reel_out, digits = 3), " m/s")
azimuth = calc_azimuth(kps3)
println("azimuth: ", round(rad2deg(azimuth), digits = 2), "°")

# print alpha2, alpha3, alpha4
# println("alpha2, alpha3, alpha4: ", kps3.alpha_2, " ", kps3.alpha_3, " ", kps3.alpha_4)
println("heading: ", round(rad2deg(calc_heading(kps3)), digits = 2), "°")

ss = SysState(kps3)
println("AoA: ", rad2deg(ss.AoA), "°")
println("CL2: ", ss.CL2, " CD2: ", ss.CD2)
println("v_wind_gnd: ", ss.v_wind_gnd)
println("v_wind_200m: ", ss.v_wind_200m)
println("v_wind_kite: ", ss.v_wind_kite)

# output on main branch
# Lift: 1047.1795339611076, Drag: 281.39765463928745, elev: 72.77014, Iterations: 1552
# roll: -0.0 pitch: 5.755691707832273 yaw: 90.00000250447816
# x:[-0.9949585608372032, 0.0, 0.10028689952711267]
# y:[0.0, 1.0, 0.0]
# z:[-0.10028689952711267, -2.8981421002141166e-19, -0.9949585608372032]
# alpha2, alpha3, alpha4: 7.546125780476343 10.000000853773646 10.000000853773646
