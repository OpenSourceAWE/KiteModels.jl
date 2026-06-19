# Copyright (c) 2022, 2024 Uwe Fechner
# SPDX-License-Identifier: MIT

# plot the lift and drag coefficients as function of angle of attack
using Printf
using Pkg
if dirname(Pkg.project().path) != @__DIR__
    Pkg.activate(@__DIR__)
end
using KiteModels, KitePodModels, LinearAlgebra, Rotations

set::Settings = deepcopy(load_settings("system.yaml"))

using MakieControlPlots

set.abs_tol=0.00006
set.rel_tol=0.000001
V_WIND = 14.5

# the following values can be changed to match your interest
dt::Float64 = 0.05
set.solver="DFBDF" # IDA or DFBDF
set.v_reel_out = 1.0 # initial reel-out speed [m/s]
STEPS = 1
const PLOT = true
UPWIND_DIR = - 90.0-pi/2+deg2rad(10) # Zero is at north; clockwise positive
# end of user parameter section #

elev = set.elevation
set.v_wind = V_WIND # 25
set.upwind_dir = UPWIND_DIR
logger = Logger(set.segments + 5, STEPS)

kcu::KCU = KCU(set)
kps4::KPS4 = KPS4(kcu)
integrator = KiteModels.init!(kps4; delta=0.001, stiffness_factor=0.1, prn=STATISTIC)
for _ in 1:80
    next_step!(kps4, integrator; set_speed=kps4.set.v_reel_out, upwind_dir=deg2rad(UPWIND_DIR), dt)
end
lift, drag = lift_drag(kps4)
sys_state::SysState = KiteModels.SysState(kps4)
log!(logger, sys_state)
elev = rad2deg(logger.elevation_vec[end])
println("Lift: $lift, Drag: $drag, elev: $elev, Iterations: $(kps4.iter)")

q = QuatRotation(calc_orient_quat(kps4; viewer=false))
roll, pitch, yaw = rad2deg.(quat2euler(q))
println("--> orient_quat:       roll: ", roll, " pitch: ", pitch, "  yaw: ", yaw)
roll, pitch, yaw = rad2deg.(orient_euler(kps4))
println("--> orient_euler:      roll: ", roll, " pitch: ", pitch, " yaw:  ", yaw)
q = QuatRotation(calc_orient_quat(kps4; viewer=true))
roll, pitch, yaw = rad2deg.(quat2euler(q))
println("--> orient_quat (viewer): roll: ", roll, " pitch: ", pitch, "   yaw: ", yaw)

println("x:", kps4.x) # from trailing edge to leading edge in ENU reference frame
println("y:", kps4.y) # to the right looking in flight direction
println("z:", kps4.z) # down
azimuth = calc_azimuth(kps4)
println("azimuth: ", round(rad2deg(azimuth), digits = 2), "°")
println("azimuth_north: ", round(rad2deg(KiteUtils.azimuth_north(pos_kite(kps4))), digits = 2), "°")

# print point C and point D
pos_C, pos_D = kps4.pos[kps4.set.segments+4], kps4.pos[kps4.set.segments+5]

# print alpha2, alpha3, alpha4
println("alpha2, alpha3, alpha4: ", kps4.alpha_2, " ", kps4.alpha_3, " ", kps4.alpha_4)
println("heading: ", round(rad2deg(calc_heading(kps4)), digits = 2), "°")

ss = SysState(kps4)
println("AoA: ", rad2deg(ss.AoA), "°")
println("CL2: ", ss.CL2, " CD2: ", ss.CD2)
println("v_wind_gnd: ", ss.v_wind_gnd)
println("v_wind_200m: ", ss.v_wind_200m)
println("v_wind_kite: ", ss.v_wind_kite)

# output on main branch
# Lift: 855.4784722062811, Drag: 192.0536202491188, elev: 69.45377, Iterations: 69
# --> orient_quat:       roll: -0.001221503359884115 pitch: 7.507464596247422  yaw: -80.00459886809945
# --> orient_euler:      roll: -0.001221503359884115 pitch: 7.507464596247422 yaw:  -80.00459886809945
# --> orient_quat (viewer): roll: 82.4925322579098 pitch: 0.0012073499097366438   yaw: -99.9955599945749
# x:[-0.9763796400403286, 0.1720812646439843, 0.13065541272074638]
# y:[0.1735718785102554, 0.9848212033363726, 2.1201897232929594e-5]
# z:[-0.1286685950297495, 0.02269867772223979, -0.9914278403811002]
# azimuth: -0.0°
# azimuth_north: -100.0°
# alpha2, alpha3, alpha4: 10.315549445214474 10.006140602413751 10.007820124746754
# heading: 359.99°
# AoA: 10.315549°
# CL2: 0.58924925 CD2: 0.2437712
# v_wind_gnd: Float32[14.279713, -2.5178986, 0.0]
# v_wind_200m: Float32[19.261637, -3.3963463, 0.0]
# v_wind_kite: Float32[18.914501, -3.335137, 0.0]

